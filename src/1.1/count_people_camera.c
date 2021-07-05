/*
 * Author: Ta Luc Gia Hoang
 * Date: Nov 25, 2017
 *
 * Version 1.1: Port to Linux OS, clean and make code easier to read
 * Version 1.0: First release for Windows OS
 *
 * Notes: 
 * This program uses OpenCV 2.4.13 library, which is an old version supporting for C language.
 * Compile and build on Linux Ubuntu 16
 */


#include <opencv/cv.h>
#include <opencv/highgui.h>
//#include <opencv/cvaux.h>// nho uncomment
#include <string.h>
#include <time.h>

/* Linux */
/* opnecv 2.4.13 */
#include <opencv2/core/types_c.h>
#include <opencv2/legacy/legacy.hpp>   /* Background codebook */
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
#include <stdbool.h>  // true false
//#include <opencv2/video/background_segm.hpp>


//#define CAM_WIDTH 320//640
//#define CAM_HEIGHT 240//480
#define CHANNELS 3
#define UPDATE_PERIOD 2000
#define CLEAR_PERIOD  4000
#define LEARNING_TIME 100
#define NUM_OF_PIXEL_ON_LINE 500
#define WIDTH_CENTER_LINE	10

#define RED    cvScalar(0, 0, 255, 0)
#define GREEN  cvScalar(0, 255, 0, 0)
#define BLUE   cvScalar(255, 0, 0, 0)
#define WHITE  cvScalar(255, 255, 255, 0)
#define BLACK  cvScalarAll(0)
#define YELLOW cvScalar(0, 255, 255, 0)

int CamWidth = 0;// CAM_WIDTH;
int CamHeight = 0;// CAM_HEIGHT;
int center_x = 0;// CamWidth / 2;
int center_y = 0;// CamHeight / 2;
int nframes = 0, nframesToLearnBG = 0;
bool useAVIfile = false;
bool firstInMode = true;
int pos = 0;
int fps = 0;
CvCapture *capture;
int maxFrame = 0;


//typedef struct _Line {
//	CvPoint pt1;
//	CvPoint pt2;
//	CvPoint center = cvPoint((int)((pt1.x + pt2.x) / 2), (int)((pt1.y + pt2.y) / 2));
//	int width;
//} Line;


int count_in = 0, count_out = 0;
int mode = 2;	//HORIZONTAL MODE (DEFAULT)

//VARIABLES for CODEBOOK METHOD:
IplImage* tempImage = 0;
IplImage* rawImage = 0, *hsvImage = 0; //yuvImage is for codebook method
IplImage *ImaskCodeBook = 0, *ImaskCodeBookCC = 0;

CvBGCodeBookModel *model = 0;
int modMin = 50, modMax = 10;	//codebook configuration
void configureBGCodeBookModel(CvBGCodeBookModel *model)
{
	//Set color thresholds to default values
	model->cbBounds[0] = model->cbBounds[1] = model->cbBounds[2] = 10;	//10
	model->modMin[0] = 70;	model->modMax[0] = 70;          //H
	model->modMin[1] = 10;	model->modMax[1] = 40;	        //S
	model->modMin[2] = modMin;	model->modMax[2] = modMax;	//V
}
const int NCHANNELS = 3;

IplImage* circleImage = 0;

// various tracking parameters (in seconds)
const double MHI_DURATION = 0.5;//1;
const double MAX_TIME_DELTA = 0.5;
const double MIN_TIME_DELTA = 0.05;
int duration = 1000;
int seg_thresh = 1500;


// temporary images
IplImage* motion = 0;

IplImage* silh = 0;// 8U, 1-channel
IplImage *mhi = 0; // MHI 32F, 1-channel
IplImage *orient = 0; // orientation 32F, 1-channel
IplImage *mask = 0; // valid orientation mask 8U, 1-channel
IplImage *segmask = 0; // motion segmentation map 32F, 1-channel

int idx = 0;
int saiso = 40;

typedef struct _Tracking {
	bool value;
	double exist_time;	// life time
	double start_time;
	double time_count;
}Tracking;
Tracking *countOnLine;// [NUM_OF_PIXEL_ON_LINE];//[CAM_HEIGHT];//[CamHeight];

static struct _Tracking createTracking(void)
{
	struct _Tracking trk;
	trk.value =  false;
	trk.exist_time = 10;	//	exist_time < line_w/2 ,  exist_time ~= update
	trk.start_time = (double)clock() / CLOCKS_PER_SEC;	//get current time (second);
	trk.time_count = 0;
	return trk;
}

struct _Tracking* initCountLine(unsigned int count)
{
	Tracking* arr;
	arr = (Tracking*)malloc(count);
//	printf("arr %p\n", arr);s
	for(int i = 0; i<count; i++)
	{
		arr[i] = createTracking();
	}
	return arr;
}

void releaseTracking (Tracking* track)
{
	if(track)
		free(track);
	track = NULL;
}

/////////////////////////////////////////////
//void max_mod_Trackbar(int pos);
//void min_mod_Trackbar(int pos);

int track_object = 0;

CvPoint origin;



void on_mouse(int event, int x, int y, int flags, void* param);




int h_min  = 10, h_max  = 155;	//hue mau sac
int s_min  = 0,  s_max  = 255;	//saturation do tuong phan
int v_min  = 0,  v_max  = 255;		//value do sang

void threshold_trackbar(const char* name);
void control_trackbar(const char* name);
void onTrackbarSlide(int n) {
	cvSetCaptureProperty(capture, CV_CAP_PROP_POS_FRAMES, n);
}

CvFont g_font;


int poly1Hull0 = 0;
int line_w = 20;

int min_len  = 650;
int max_len  = 5000;			//< 2000 - 5000
int min_area = 6000;
int max_area = 70000;//640 * 480;	// < 30 000 - 50 000
CvPoint center [50];


CvMemStorage* storage = 0; // temporary storage

//function in mhi_update()
/*
 * Return:
 *    1 = in, 2 = out, 0 = NULL
 */
static int detect_object(
	int detectable_direction, int object_center,
	int center_of_detect_line, int num_of_pixel_on_line,
	int direction_point,
	Tracking *track_line,
	int index_of_detected_object,
	double exist_time	//	exist_time < line_w/2 ,  exist_time ~= update
) {

	int _index = index_of_detected_object;	
	bool _isDetected = false;
	
	for (int i = 0; i < num_of_pixel_on_line; i++) {
		if (track_line[i].value) {	//kiem tra cac diem cu vat da di qua
			//if (useAVIfile) {
				track_line[i].time_count += 1;
				if (track_line[i].time_count > track_line[i].exist_time) {
					track_line[i].time_count = 0;
					track_line[i].value = false;
				}
			//}
			/*else {
				track_line[i].time_count = (double)clock() / CLOCKS_PER_SEC;	//get current time (second)
				//if (track_line[i].time_count - track_line[i].exist_time > 3.0) {
				if (track_line[i].time_count - track_line[i].start_time  > track_line[i].exist_time > 3.0) {
					track_line[i].time_count = 0;
					track_line[i].value = false;
				}
			}*/
		}
	}
	if (center_of_detect_line - line_w < detectable_direction  && detectable_direction < center_of_detect_line + line_w) {
		_index = object_center;	//luu toa do diem moi bi phat hien
		if (track_line[_index].value == false) {
			for (int i = -saiso; i < saiso; i++) {
				if ((_index + i <0) || (_index + i >= num_of_pixel_on_line))
					continue;	// tranh truong hop bi loi cac diem nam ben ngoai khung hinh
				
				// khoi tao cac diem duoc danh dau xuat hien vat
				track_line[_index + i].value = true;
				track_line[_index + i].time_count = 0;	//bat dau dem thoi gian ton tai
				
				//if (useAVIfile) 			
					track_line[_index + i].exist_time = exist_time;
				/*else {
					// use camera
					//track_line[_index + i].exist_time = (double)clock() / CLOCKS_PER_SEC;	//get current time (second)
					track_line[_index + i].exist_time = 2.0;	//thoi gian ton tai (second)
					track_line[_index + i].start_time = (double)clock() / CLOCKS_PER_SEC;	//get current time (second)
				}*/
				_isDetected = true;
			}
		}
		// Confirm moving orientation
		if (_isDetected) {
			if (direction_point - detectable_direction > 0) { return 1; }	//input
			else { return 2; }	//output
		}
	}
	return 0;
}

// number of cyclic frame buffer used for motion detection
// (should, probably, depend on FPS)
const int N = 4;

// ring image buffer
IplImage **buf = 0;
int last = 0;

IplImage** createIplImageBuf(const int numOfBuf, CvSize size)
{
	IplImage** arr;
	int i;
	if (arr == 0) {
		arr = (IplImage**)malloc(numOfBuf * sizeof(arr[0]));
		memset(arr, 0, numOfBuf * sizeof(arr[0]));
	}
	for (i = 0; i < numOfBuf; i++) {	//4 images
		arr[i] = cvCreateImage(size, IPL_DEPTH_8U, 1); //cvCreateImage(size, IPL_DEPTH_8U, 1);
		cvZero(arr[i]);
	}	
	return arr;
}

void releaseIplImageBuf(const int numOfBuf)
{
	for (int i = 0; i < numOfBuf; i++) 	//4 images
		if(buf[i])
			cvReleaseImage(&buf[i]);
	
}

void update_mhi(IplImage* img, IplImage* dst, int diff_threshold) {

	double timestamp = (double)clock() / CLOCKS_PER_SEC; // get current time in seconds
	int i;
	//bo sung lai
	double seg_thresh, duration;
	if (useAVIfile) {
		seg_thresh = 1.5;
		duration = 1.0;
	}else{
		seg_thresh = 1.75;		//use camera
		duration = 1.5;

	}
	
	if (img->nChannels != 1)
		cvCvtColor(img, buf[last], CV_BGR2GRAY); // convert frame to grayscale
	else
		cvCopyImage(img, buf[last]);
	int idx1 = last;
	int idx2 = (last + 1) % N; // index of (last - (N-1))th frame
	last = idx2;
	silh = buf[idx2];
	cvAbsDiff(buf[idx1], buf[idx2], silh); // get difference between frames
	cvThreshold(silh, silh, 50, 255, CV_THRESH_BINARY); // and threshold it
	// ket thuc bo sung

	//cvThreshold(ImaskCodeBookCC, silh, 50, 255, CV_THRESH_BINARY); // and threshold it
	cvUpdateMotionHistory(silh, mhi, timestamp, 1.0); //MHI_DURATION);// update MHI	= 1ms	//thoi gian luu cac bong
	cvCvtScale(mhi, mask, 255, 0);//((double)(duration / 1000) - timestamp) * 255);// convert MHI to blue 8u image
	cvZero(dst);
	cvMerge(mask, 0, 0, 0, dst);
	// calculate motion gradient orientation and valid orientation mask
	cvCalcMotionGradient(mhi, mask, orient, MAX_TIME_DELTA, MIN_TIME_DELTA, 3);
	if (!storage)
		storage = cvCreateMemStorage(0);
	else
		cvClearMemStorage(storage);

	// segment motion: get sequence of motion components
	// segmask is marked motion components map. It is not used further
	CvSeq* seq;

	
	seq = cvSegmentMotion(mhi, segmask, storage, timestamp, seg_thresh);//(double)(seg_thresh / 1000));//seg_thresh = MAX_TIME_DELTA = 0.5;//	seg_thresh = 1.5 times the average difference in sillouete time stamps

	// iterate through the motion components,
	// One more iteration (i == -1) corresponds to the whole image (global motion)
	CvRect comp_rect;
	double count;
	double angle;
	//CvPoint momentcenter;
	double magnitude = 40;;
	for (i = -1; i < seq->total; i++) {

		if (i < 0) { // case of the whole image
			continue;
		}
		else { // i-th motion component
			comp_rect = ((CvConnectedComp*)cvGetSeqElem(seq, i))->rect;	//take the bounding rectangle for each motion
		}
		// select component ROI
		cvSetImageROI(silh, comp_rect);
		cvSetImageROI(mhi, comp_rect);
		cvSetImageROI(orient, comp_rect);
		cvSetImageROI(mask, comp_rect);
		//calculate moment
		//cvMoments(silh, &moments, 1);
		//double M00 = cvGetSpatialMoment(&moments, 0, 0);
		//double M01 = cvGetSpatialMoment(&moments, 0, 1);
		//double M10 = cvGetSpatialMoment(&moments, 1, 0);
		//momentcenter.x = (int)(M10 / M00);
		//momentcenter.y = (int)(M01 / M00);
		// calculate orientation
		
		angle = cvCalcGlobalOrientation(orient, mask, mhi, timestamp, duration);	//calculate orientation for each object
		angle = 360.0 - angle;  // adjust for images with top-left origin
		count = cvNorm(silh, 0, CV_L1, 0); // calculate number of points within silhouette ROI
		cvResetImageROI(mhi);
		cvResetImageROI(orient);
		cvResetImageROI(mask);
		cvResetImageROI(silh);
		// check for the case of little motion
		if (count < comp_rect.width*comp_rect.height * 0.05)
			continue;
		CvPoint directionPoint;
		//momentcenter = cvPoint((comp_rect.x + momentcenter.x), (comp_rect.y + momentcenter.y));
		/*directionPoint = cvPoint(cvRound(momentcenter.x + magnitude*cos(angle*CV_PI / 180)),
			cvRound(momentcenter.y - magnitude*sin(angle*CV_PI / 180)));*/
		directionPoint = cvPoint(cvRound(center[i].x + magnitude*cos(angle*CV_PI / 180)),
			cvRound(center[i].y - magnitude*sin(angle*CV_PI / 180)));

		//cvCircle(dst, momentcenter, 2, CV_RGB(255, 0, 0), -1, 8, 0);
		cvRectangleR(dst, comp_rect, CV_RGB(255, 0, 0), 1, 8, 0);
		cvLine(rawImage, center[i], directionPoint, CV_RGB(100, 255, 0), 2, 8 ,0);

		int isDetected = 0;
		if (mode == 1) {
			//vertical detective line
			if (firstInMode) {
				countOnLine = initCountLine(CamHeight);
				firstInMode = false;
			}			
			isDetected = detect_object(
				//momentcenter.x, momentcenter.y,
				center[i].x, center[i].y,
				CamWidth / 2, CamHeight,
				directionPoint.x,
				countOnLine,
				idx,
				line_w / 2	);
		}
		
		if (mode == 2) {			
			//horizontal detective line
			if (firstInMode) {
				countOnLine = initCountLine(CamWidth);
				firstInMode = false;
			}
			isDetected = detect_object(
				//momentcenter.y, momentcenter.x,
				center[i].y, center[i].x,
				CamHeight / 2, CamWidth,
				directionPoint.y,
				countOnLine,
				idx,
				line_w / 2	);
		}

		if (isDetected == 1) {
			//input 	
			count_in++;		printf("IN: %d\n", count_in);
		}
		if (isDetected == 2) {
			//output	
			count_out++;	printf("OUT: %d\n", count_out);
		}
	}

}

CvRect selection;
int select_object = 0;

void setMouseCallback(const char* name, CvMouseCallback on_mouse)
{
	cvNamedWindow(name, CV_WINDOW_AUTOSIZE);
	cvSetMouseCallback(name, on_mouse, 0);
}

void on_mouse(int event, int x, int y, int flags, void* param)
{
	if (select_object)
	{
		selection.x = MIN(x, origin.x);
		selection.y = MIN(y, origin.y);
		selection.width = selection.x + CV_IABS(x - origin.x);
		selection.height = selection.y + CV_IABS(y - origin.y);

		selection.x = MAX(selection.x, 0);	// = |x| or 0
		selection.y = MAX(selection.y, 0);
		selection.width = MIN(selection.width, rawImage->width);
		selection.height = MIN(selection.height, rawImage->height);
		selection.width -= selection.x;
		selection.height -= selection.y;
		printf("(%d,%d) %dx%d \n", selection.x, selection.y, selection.width, selection.height);
	}

	switch (event)
	{
	case CV_EVENT_LBUTTONDOWN:
		origin = cvPoint(x, y);
		selection = cvRect(x, y, 0, 0);
		select_object = 1;
		break;
	case CV_EVENT_LBUTTONUP:
		select_object = 0;
		if (selection.width > 0 && selection.height > 0)
			track_object = -1;
		break;
	}
}

void drawMouseSelection(IplImage* img, CvScalar color)
{
	if (select_object && selection.width > 0 && selection.height > 0)
	{
		cvRectangleR(img, selection, color, 3, 8, 0);
		cvSetImageROI(img, selection);
		cvXorS(img, cvScalarAll(125), img, 0);
		cvResetImageROI(img);
	}
}


/*
 * rawImage        : IPL_DEPTH_8U, RGB image
 * hsvImage        : IPL_DEPTH_8U, HSV image
 * ImaskCodeBook   : IPL_DEPTH_8U, gray-scale image
 * ImaskCodeBookCC : IPL_DEPTH_8U, gray-scale image
 * circleImage     : IPL_DEPTH_8U, gray-scale image
 */

void InitImages(CvSize size)
{
	if(rawImage)
	{
		//	SILHOUTTE REDUCTION
		tempImage = cvCloneImage(rawImage);

		// CODEBOOK METHOD ALLOCATION
		hsvImage = cvCreateImage(size, IPL_DEPTH_8U, 3);//cvCloneImage(rawImage);
		ImaskCodeBook = cvCreateImage(size, IPL_DEPTH_8U, 1);
		ImaskCodeBookCC = cvCreateImage(size, IPL_DEPTH_8U, 1);
		cvSet(ImaskCodeBook, WHITE, NULL );

		circleImage = cvCreateImage(size, IPL_DEPTH_8U, 1);

		silh = cvCreateImage(size, IPL_DEPTH_8U, 1);	// 8U, 1-channel
		mhi = cvCreateImage(size, IPL_DEPTH_32F, 1);	// 32F, 1-channel
		cvZero(mhi); // clear MHI at the beginning
		orient = cvCreateImage(size, IPL_DEPTH_32F, 1);
		segmask = cvCreateImage(size, IPL_DEPTH_32F, 1);
		mask = cvCreateImage(size, IPL_DEPTH_8U, 1);

		motion = cvCreateImage(size, IPL_DEPTH_8U, 3);//cvCloneImage(rawImage);
		//cvZero(motion);
	}
	else
		printf("InitImages NULL input\n");

}

void ReleaseAllImages()
{
	if(tempImage) cvReleaseImage(&tempImage);
	if(silh) cvReleaseImage(&silh);
	if(mhi)	cvReleaseImage(&mhi);
	if(orient) cvReleaseImage(&orient);
	if(segmask)	cvReleaseImage(&segmask);
	if(motion)	cvReleaseImage(&motion);
	if(hsvImage) cvReleaseImage(&hsvImage);
	if(ImaskCodeBook) cvReleaseImage(&ImaskCodeBook);
	if(ImaskCodeBookCC)	cvReleaseImage(&ImaskCodeBookCC);
	if(circleImage)	cvReleaseImage(&circleImage);
}

//#define RECORD_DETAILS
CvVideoWriter* writer = 0;
#ifdef RECORD_DETAILS
CvVideoWriter *writer0 = 0, *writer1 = 0, *writer2 = 0, *writer3 = 0, *writer4 = 0;
#endif
void releaseAllVideos(void)
{
	if(writer)  cvReleaseVideoWriter(&writer);
#ifdef RECORD_DETAILS
	if(writer0)	cvReleaseVideoWriter(&writer0);
	if(writer1)	cvReleaseVideoWriter(&writer1);
	if(writer2)	cvReleaseVideoWriter(&writer2);
	if(writer3)	cvReleaseVideoWriter(&writer3);
	if(writer4)	cvReleaseVideoWriter(&writer4);
#endif
}

void createVideo(double fcc, double fps, CvSize sz)
{
	writer = cvCreateVideoWriter("PeopleCounter.avi", fcc, 24, sz, 1);
#ifdef RECORD_DETAILS
	writer0 = cvCreateVideoWriter("foreground.avi", fcc, 24, sz, 1);
	writer1 = cvCreateVideoWriter("connected-component.avi", fcc, 24, sz, 1);
	writer2 = cvCreateVideoWriter("circle.avi", fcc, 24, sz, 1);
	writer3 = cvCreateVideoWriter("motion.avi", fcc, 24, sz, 1);
	writer4 = cvCreateVideoWriter("hsv.avi", fcc, 24, sz, 1);
#endif
}

int recordVideo(IplImage* img, CvVideoWriter* video)
{
	IplImage* tmp;
	int r = 0;
	if(!img) return -1;
	if(!video) return -1;
	if(img->nChannels == 1)
	{
		tmp = cvCreateImage(cvGetSize(img), 8, 3);
		cvCvtColor(img, tmp, CV_GRAY2BGR);
		r = cvWriteFrame(video, tmp);
		cvReleaseImage(&tmp);
	}
	else if(img->nChannels == 3)
	{
		r = cvWriteFrame(video, img);
	}
	return r;
}

int silhoutte_reduction(IplImage* src, IplImage* dst, int codeCvt )
{
	int code;
	CvSize size;
	IplImage *h, *s, *v;

	if(!src)
		return -1;
	if(!dst)
		return -1;
	if(src->nChannels != 3 || dst->nChannels != 3)
		return -1;
	
	if (codeCvt != CV_BGR2HSV)  code = codeCvt;
	else code = CV_BGR2HSV;
	size = cvGetSize(src);
	h = cvCreateImage(size, 8, 1);
	s = cvCreateImage(size, 8, 1);
	v = cvCreateImage(size, 8, 1);

	cvCvtColor(src, dst, CV_BGR2HSV);
	cvSplit(dst, h, s, v, 0);
	cvThreshold(h, h, h_min, h_max, CV_THRESH_TOZERO);	//CV_THRESH_TOZERO = 3,  /* value = value > threshold ? value : 0           */
	cvThreshold(h, h, h_max, h_max, CV_THRESH_TOZERO_INV);	//CV_THRESH_TRUNC = 2,  /* value = value > threshold ? threshold : value   */
	cvThreshold(s, s, s_min, s_max, CV_THRESH_TOZERO);	//CV_THRESH_TOZERO_INV  =4,  /* value = value > threshold ? 0 : value           */
	cvThreshold(s, s, s_max, s_max, CV_THRESH_TOZERO_INV);
	cvThreshold(v, v, v_min, v_max, CV_THRESH_TOZERO);
	cvThreshold(v, v, v_max, v_max, CV_THRESH_TOZERO_INV);
	cvMerge(h, s, v, 0, dst);
	cvReleaseImage(&h);
	cvReleaseImage(&s);
	cvReleaseImage(&v);
	return 0;
}

/*
 * Check perimeter
 * Return 1 if true, 0 if false
 */
static int checkContourPerimeter(CvSeq* c, double min, double max)
{
	double c_len = cvContourPerimeter(c);
	if ((double)min < c_len || c_len < (double)max)
		return 1;
	return 0;
}

/*
 * Check area
 * Return 1 if true, 0 if false
 */
static int checkContourAre(CvSeq* c, double min, double max)
{
	double c_area = cvContourArea(c, CV_WHOLE_SEQ, 0);
	if ((double)min < c_area || c_area < (double)max)
		return 1;
	return 0;
}

/*
 * Calculate center point of rectangle
 */
static CvPoint calcMomentCenterPoint(IplImage* image, CvRect rect)
{
	CvPoint center;
	CvMoments moments;
	cvSetImageROI(image, rect);
	cvMoments(image, &moments, 1);
	double M00 = cvGetSpatialMoment(&moments, 0, 0);
	double M01 = cvGetSpatialMoment(&moments, 0, 1);
	double M10 = cvGetSpatialMoment(&moments, 1, 0);
	center.x = (int)(M10 / M00);	// x of ROI
	center.y = (int)(M01 / M00);	// y of ROI
	center = cvPoint((rect.x + center.x), (rect.y + center.y));  // (x,y) of image
	cvResetImageROI(image);
	return center;
}


/*
 *
 */
static void drawCoordinate(CvArr* img, CvPoint point, const CvFont* font, CvScalar color)
{
	char text[50];
	sprintf(text, "%d,%d", point.x, point.y);
	cvPutText(img, text, point, font, color);
}

/*
 * Filter: reject too small and too large object
 * Return -1 if false
 */
int reject_small_and_large_object(
		IplImage* src, IplImage* dst,	// 
		IplImage* circleImage,	/* output */
		CvPoint* center,
		int /*&numOfCenters*/ *numOfCenters,	/* output */
		CvMemStorage* storage) 
{
	IplImage *temp;

//	CvSeq* contours = 0;
	if (!storage)
		storage = cvCreateMemStorage(0);
	else cvClearMemStorage(storage);

	if(!src || !dst || !circleImage)
	{
		printf("ERROR: reject_small_and_large_object NULL input\n");
		return -1;
	}
	
	cvZero(dst);			//output image
	cvZero(circleImage);
	// Check channel
	if(src->nChannels != 1 && dst->nChannels != 1 && circleImage->nChannels != 1)
	{
		printf("ERROR: nChannels != 1\n");
		return -1;
	}

	temp = cvCreateImage(cvGetSize(src), 8, 1);
	cvCopyImage(src, temp);	//input image

	
//	IplConvKernel *element = cvCreateStructuringElementEx(10, 30, 5, 14, CV_SHAPE_RECT, 0);
	cvMorphologyEx(temp, temp, 0, 0, CV_MOP_OPEN, 2);
	cvMorphologyEx(temp, temp, 0, 0, CV_MOP_CLOSE, 2);

	CvContourScanner scanner = cvStartFindContours(temp, storage, sizeof(CvContour), 0, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0));
	CvSeq *c;
	int numContour = 0;
	
	while ((c = cvFindNextContour(scanner)) != NULL) {
		if ( checkContourPerimeter(c, min_len, max_len) == false 
			|| checkContourAre(c, min_area, max_area) == false){
			continue;	// ignore
		}
		else {
			cvDrawContours(dst, c, WHITE, BLACK, 0, -1, 8, cvPoint(0, 0));
			CvRect rect = cvBoundingRect(c, 0);
			center[numContour] = calcMomentCenterPoint(dst, rect);
			
			cvCircle(circleImage, center[numContour], 20, WHITE, -1, 8, 0);  // gray-scale
			cvCircle(rawImage, center[numContour], 10, BLUE, -1, 8, 0);	// color
			cvRectangleR(rawImage, rect, WHITE, 1, 8, 0);	//  color
			drawCoordinate(rawImage, center[numContour], &g_font, BLUE);

			numContour++;
			if (numContour > *numOfCenters) {
				numContour = *numOfCenters;
				break;
			}
		}
	}
	*numOfCenters = numContour;
	cvEndFindContours(&scanner);
	if(temp) cvReleaseImage(&temp);
	return 0;
}

/*
 * Return display mode
 */
int selectDisplay (IplImage* display, int mode)
{
	cvZero(display);
	switch (mode)
	{
	case 1:
		cvMerge(ImaskCodeBook, 0, 0, 0, display);
		break;
	case 2:
		cvMerge(0, 0, ImaskCodeBookCC, 0, display);
		break;
	case 3:
		cvMerge(0, circleImage, 0, 0, display);
		break;
	case 4:
		cvCopyImage(motion, display);
		break;
	case 5:
		cvCopyImage(hsvImage, display);
		break;
	case 0:
	default:
		cvCopyImage(rawImage, display);
		mode = 0;
		break;
	}
	return mode;
}

int window_mode = 0;
bool pause = false;
bool temp_image = false;
bool color_window = true;
bool auto_update = false;
bool update = false, clear = false;
int update_duration = 6, clear_duration = 60;// seconds

static time_t start_update_time, start_clear_time, current_time;

bool updateBackgroundCodeBook(bool update_flag, CvBGCodeBookModel* model, const CvArr* image)
{
	bool flag = update_flag;
	if(flag)
	{
		flag = false;
		cvBGCodeBookUpdate(model, image, cvRect(0,0,0,0), 0);
		start_update_time = (double)clock() / CLOCKS_PER_SEC;
		printf("update codebook\n");		
	}
	return flag;
}

bool clearBackgroundCodeBook(bool clear_flag, CvBGCodeBookModel* model, int staleThresh)
{
	bool flag = clear_flag;
	if(flag)
	{
		flag = false;
		cvBGCodeBookClearStale(model, staleThresh /*model->t / 2*/, cvRect(0,0,0,0), 0);
		start_clear_time = (double)clock() / CLOCKS_PER_SEC;
		printf("clear codebook\n");	
	}
	return flag;
}

void trainBackgroundCodeBook(CvBGCodeBookModel* model, const CvArr* image, unsigned int count, unsigned int max_count)
{
	if(count < max_count){
		cvBGCodeBookUpdate(model, image, cvRect(0,0,0,0), 0);
		printf("training update codebook\n");
	}

	if(count == max_count){
		cvBGCodeBookClearStale(model, model->t / 2, cvRect(0,0,0,0), 0);
		printf("training clear codebook\n");
	}
	start_update_time = (double)clock() / CLOCKS_PER_SEC;
	start_clear_time = (double)clock() / CLOCKS_PER_SEC;
}

void autoUpdateBackgroundCodeBook (void)
{
	current_time = (double)clock() / CLOCKS_PER_SEC; // get current time in seconds
	if (current_time - start_update_time >= update_duration)
	{
		start_update_time = (double)clock() / CLOCKS_PER_SEC;
		update = true;
	}

	if (current_time - start_clear_time >= clear_duration)
	{
		start_clear_time = (double)clock() / CLOCKS_PER_SEC;
		clear = true;
	}
}

/*
 * Create control table to:
 * name = "Control"
 * - "update(s)"   : modify time to update codebook (in seconds)
 * - "clear(s)"    : modify time to clear codebook (in seconds)
 * - "poly1_hull0" : switch to Poly(1) or Hull(0) for cvSegmentFGMask function
 * - "min len" "max len"   : modify perimeter condition of object recognition (in pixels)
 * - "min area" "max area" : modify area condition of object recognition (in pixels)
 */
void control_trackbar(const char* name)
{
	cvNamedWindow(name, CV_WINDOW_AUTOSIZE);
	cvResizeWindow(name, 300, 540);
	cvCreateTrackbar("update(s)", "Control", &update_duration, 120, NULL);	//seconds
	cvCreateTrackbar("clear(s)", "Control", &clear_duration, 5*60, NULL);		//seconds
	cvCreateTrackbar("poly1_hull0", "Control", &poly1Hull0, 1, NULL);
	cvCreateTrackbar("width", "Control", &line_w, 80, NULL);
	//cvCreateTrackbar("duration (ms)", "Control", &duration, 2000, NULL);
	//cvCreateTrackbar("seg_thresh (ms)", "Control", &seg_thresh, 2000, NULL);
	cvCreateTrackbar("min len", "Control", &min_len, 10000, NULL);
	cvCreateTrackbar("max len", "Control", &max_len, 10000, NULL);
	cvCreateTrackbar("min area", "Control", &min_area, 320*240, NULL);
	cvCreateTrackbar("max area", "Control", &max_area, 480*640, NULL);
}

/*
 * Create threshold table to:
 * name = "Set YUV Background"
 * - "modMin" "modMax" : modify max/min of codebook configuration
 * - "h_min" "h_max"   : modify max/min threshold on H-plane
 * - "s_min" "s_max"   : modify max/min threshold on S-plane
 * - "v_min" "v_max"   : modify max/min threshold on V-plane
 */
void threshold_trackbar(const char* name)
{
	cvNamedWindow(name , CV_WINDOW_AUTOSIZE);
	cvResizeWindow(name, 260, 450);
	cvCreateTrackbar("modMin", name, &modMin, 255, NULL);
	cvCreateTrackbar("modMax", name, &modMax, 255, NULL);
	cvCreateTrackbar("h_min", name, &h_min, 255, NULL);
	cvCreateTrackbar("h_max", name, &h_max, 255, NULL);
	cvCreateTrackbar("s_min", name, &s_min, 255, NULL);
	cvCreateTrackbar("s_max", name, &s_max, 255, NULL);
	cvCreateTrackbar("v_min", name, &v_min, 255, NULL);
	cvCreateTrackbar("v_max", name, &v_max, 255, NULL);
}


static void drawItem(CvArr* img, char* name, int value, CvPoint point, const CvFont* font, CvScalar color)
{
	char text[50];
	sprintf(text, "%s: %d  ", name, value);
	cvPutText(img, text, point, font, color);
}

/*
 *
 */
static void drawLines (CvArr* img, CvPoint pt1, CvPoint pt2, CvScalar color, int dx, int dy )
{
	/* Main line */
	cvLine(img, pt1, pt2, color, 1, 8, 0);
	/* 2 sub parallel line */
	cvLine(img, cvPoint(pt1.x - dx, pt1.y - dy), cvPoint(pt2.x - dx, pt2.y - dy), color, 1, 8, 0);
	cvLine(img, cvPoint(pt1.x + dx, pt1.y + dy), cvPoint(pt2.x + dx, pt2.y + dy), color, 1, 8, 0);
}

/*
 * Draw information: frame, number of In/Out people, detection line
 * drawMode1: draw vertical line
 * drawMode2: draw horizontal line
 */
static void drawMode1(IplImage* image, const CvFont* font,CvScalar color)
{
	drawLines(image, cvPoint(center_x, 0), cvPoint(center_x, CamHeight), color, line_w, 0);
	drawItem(image, "frame", nframes, cvPoint(0, 30), font, color);
	drawItem(image, "OUT", count_out, cvPoint(0, 60), font, color);
	drawItem(image, "IN", count_in, cvPoint(CamWidth - 90, 60), font, color);
}

static void drawMode2(IplImage* image,  const CvFont* font,CvScalar color)
{
	drawLines(image, cvPoint(0, center_y), cvPoint(CamWidth, center_y), color, 0, line_w);
	drawItem(image, "frame", nframes, cvPoint(0, 30), font, color);
	drawItem(image, "OUT", count_out, cvPoint(0, 60), font, color);
	drawItem(image, "IN", count_in, cvPoint(0, CamHeight - 20), font, color);
}

void displayMainWindow(const char* name, IplImage* image, int mode, int on1_off0)
{
	// draw information
	if (mode == 1)	drawMode1(image, &g_font, RED);	 //VERTICAL MODE
	if (mode == 2)	drawMode2(image, &g_font, RED);		//HORIZONTAL MODE
	// display main window
	if (on1_off0)	cvShowImage(name, image);
	else cvDestroyWindow(name);
}

static void saveAllImages(void)
{
	if(ImaskCodeBook) cvSaveImage("ImaskCodeBook.jpg", ImaskCodeBook, 0);
	if(ImaskCodeBookCC)	cvSaveImage("ImaskCodeBookCC.jpg", ImaskCodeBookCC, 0);
	if(circleImage)	cvSaveImage("circleImage.jpg", circleImage, 0);
	if(hsvImage)	cvSaveImage("hsvImage.jpg", hsvImage, 0);
	if(rawImage)	cvSaveImage("rawImage.jpg", rawImage, 0);
}

void printHelp(void)
{
	printf("Usage:\n"
	        "\t- a : switch on/off auto update\n"
			"\t- c : clear codebook\n"
			"\t- u : switch on/off auto update\n"
			"\t- 1 : foreground image\n"
			"\t- 2 : connected component image\n"
			"\t- 3 : circle image\n"
			"\t- 4 : motion image\n"
			"\t- 5 : hsv image\n"
			"\t- 6 : color image\n"
			"\t- s : save images\n"
			"\t- r : reset\n"
			"\t- t : control tables\n"
			"\t- q : Process Window\n"
			"\t- w : display on/off main window\n"
			"\t- v : vertical\n"
			"\t- h : horizontal\n"
			"\t- p : pause\n"
			"\t- 'ESC' : exit\n"	);
}

void controllerPrint(char key)
{
	switch (key) {
		printf("key: %c\t", key);
	case 'l':
		printHelp();
		break;
	case 'a':
		printf("auto update\n");
		break;
	case 'c':
		// printf("clear codebook\n");
		break;
	case 'u':
		// printf("update codebook\n");
		break;
	case '1':
		printf("foreground image\n");
		break;
	case '2':
		printf("connected component image\n");
		break;
	case '3':
		printf("circle image\n");
		break;
	case '4':
		printf("motion image\n");
		break;
	case '5':
		printf("hsv image\n");
		break;
	case '6':
		printf("color image\n");
		break;
	case 's':
		printf("save frame\n");
		break;
	case 'r':
		printf("reset\n");
		break;
	case 't':
		printf("control tables\n");
		break;
	case 'q':
		printf("Process Window\n");
		break;
	case 'w':
		printf("color_window\n");
		break;
	case 'v':
		printf("mode=vertical\n");
		break;
	case 'h':	//DEFAULT
		printf("mode=horizontal\n");
		break;
	case 'p':
		printf("pause\n");
		break;
	case 27:
		printf("exit\n");
		break;
	}
}

void controllerExecute(char key)
{
	switch (key) {
		printf("key: %c\n", key);
	case 'a':
		auto_update = !auto_update;
		printf("(%d)\n", auto_update);
		break;
	case 'c':
		clear = true;
		clear = clearBackgroundCodeBook(clear, model, model->t / 2);
		break;
	case 'u':
		update = true;
		update = updateBackgroundCodeBook(update, model, hsvImage);
		break;
	case '1':
		window_mode = 1;
//		printf("foreground image\n");
		break;
	case '2':
		window_mode = 2;
//		printf("connected component image\n");
		break;
	case '3':
		window_mode = 3;
//		printf("circle image\n");
		break;
	case '4':
		window_mode = 4;
//		printf("motion image\n");
		break;
	case '5':
		window_mode = 5;
//		printf("hsv image\n");
		break;
	case '6':
		window_mode = 0;
//		printf("color image\n");
		break;
	case 's':
		saveAllImages();
//		printf("save frame\n");
		break;
	case 'r':	// reset
		cvBGCodeBookClearStale(model, 0, cvRect(0,0,0,0), 0 );
		if(storage)	cvClearMemStorage(storage);
		ReleaseAllImages();
		nframes = 0;
		releaseTracking(countOnLine);
		break;
	case 't':
		control_trackbar("Control");
		threshold_trackbar("Set YUV Background");
		break;
	case 'q':
		temp_image = !temp_image;
		break;
	case 'w':
		color_window = !color_window;
		break;
	case 'v':
		mode = 1;
		releaseTracking(countOnLine);
		firstInMode = true;
//		printf("mode: %d-vertical\n", mode);
		break;
	case 'h':	//DEFAULT
		mode = 2;
		releaseTracking(countOnLine);
		firstInMode = true;
//		printf("mode: %d-horizontal\n", mode);
		break;
	case 'p':
		pause = !pause;
		break;
	}
}

int main(int argc, char *argv[]) {

	CvSize sz;
	capture = cvCreateCameraCapture(0);
	if (capture){
		// Capture from Camera
		//cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH, CamWidth);
		//cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT, CamHeight);*/
		;
	}
	else {
		// Capture from Video.avi
		capture = cvCreateFileCapture("sample-video.avi");
		maxFrame = (int)cvGetCaptureProperty(capture, CV_CAP_PROP_FRAME_COUNT);
		useAVIfile = true;
		fps = cvGetCaptureProperty(capture, CV_CAP_PROP_FPS);
		printf("fps: %d\n", fps);
		cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH, 320);
		cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT, 240);

		rawImage = cvQueryFrame(capture);
		sz = cvGetSize(rawImage);
	}

	nframes = 0;
	nframesToLearnBG = LEARNING_TIME;

	//initialize codebook
	model = cvCreateBGCodeBookModel();

	// Initialize global variables
	g_font = cvFont(2.0, 2);
	CamWidth = (int)cvGetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH);
	CamHeight = (int)cvGetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT);
	center_x = CamWidth / 2;
	center_y = CamHeight / 2;

	buf = createIplImageBuf(4, sz);
	
	setMouseCallback("Camera", on_mouse);
//	control_trackbar("Control");
//	threshold_trackbar("Set YUV Background");
	printf("sdfasd\n");
	int fcc = CV_FOURCC('D', 'I', 'V', '3');

	while (1)
	{
		printf("sdf\n");
//		if (!pause)
		{
			rawImage = cvQueryFrame(capture);
			if (!rawImage) break;
			++nframes;
		}

		if (nframes == 1) {
			InitImages(sz);
			selection.height = sz.height/2; // mouse click
			selection.width = sz.width/2;	// mouse click
			configureBGCodeBookModel(model);
			count_in = 0;
			count_out = 0;
			createVideo(fcc, 24, sz);
		}

		if (rawImage)
		{
			silhoutte_reduction(rawImage, hsvImage, CV_BGR2HSV);	//BGR -> HSV

			if (auto_update) {
				autoUpdateBackgroundCodeBook();
				update = updateBackgroundCodeBook(update, model, hsvImage);
				clear = clearBackgroundCodeBook(clear, model, model->t / 2);
			}

			if(nframes - 1 <= nframesToLearnBG)
				trainBackgroundCodeBook(model, hsvImage, nframes - 1, nframesToLearnBG);


			//Find the foreground if any
			if (nframes - 1 >= nframesToLearnBG)
			{

				// Find foreground by codebook method
				cvBGCodeBookDiff(model, hsvImage, ImaskCodeBook, cvRect(0,0,0,0));
				int numOfObject = 50;
#ifdef BOX
				//reject_small_and_large_object(ImaskCodeBook, ImaskCodeBook, circleImage, center, &numOfObject, storage);
				// This part just to visualize bounding boxes and centers if desired
				//cvCopy(ImaskCodeBook, ImaskCodeBookCC);
				//cvSegmentFGMask(ImaskCodeBookCC, poly1Hull0, 4.0, 0, cvPoint(0, 0));
#else
				//reject_small_and_large_object(ImaskCodeBook, ImaskCodeBookCC, circleImage, center, &numOfObject, storage);
//				buf = createIplImageBuf(4, sz);
//				releaseIplImageBuf(4);
				update_mhi(circleImage, motion, 50);	// fix bug
#endif
			}



			// on-mouse click handling
			drawMouseSelection(rawImage, YELLOW);

			// show on screen
			window_mode = selectDisplay(tempImage, window_mode);
			if (temp_image)	cvShowImage("Process Window", tempImage);
			else cvDestroyWindow("Process Window");


			displayMainWindow("Camera", rawImage, mode, color_window);
			/* Record video */
			recordVideo(rawImage, writer);          // BGR
#ifdef RECORD_DETAILS
			recordVideo(ImaskCodeBook, writer0);	// gray-scale --> BGR , "foreground.avi"
			recordVideo(ImaskCodeBookCC, writer1);  // gray-scale --> BGR , "connected component.avi"
			recordVideo(circleImage, writer2);      // gray-scale --> BGR , "circle.avi"
			recordVideo(motion, writer3);           // BGR , "motion.avi"
			recordVideo(hsvImage, writer4);         // BGR , "hsv.avi"
#endif



		}

		if (useAVIfile) {
			//pos = cvGetCaptureProperty(capture, CV_CAP_PROP_POS_FRAMES);
			//cvCreateTrackbar("frame", "Camera", &pos, maxFrame, onTrackbarSlide);
		}

		// User input:
		int key = cvWaitKey(10);
		controllerPrint(key);
		controllerExecute(key);

		if ((char)key == 27){
			break;	//end processing on 'ESC'=27
		}
	}

	releaseAllVideos();
	printf("done\n");
	return 0;
}

