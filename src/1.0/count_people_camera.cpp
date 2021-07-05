/*
 * Author: Ta Luc Gia Hoang
 * Date: June 9, 2017
 *
 * Version: 1.0
 *
 * Notes: 
 * This program uses OpenCV 2.4.13 library, which is an old version supporting for C language.
 * Compile and build on Windows 7, by Visual Studio IDE
 */
 
#include <opencv\cv.h>
#include <opencv\highgui.h>
#include <opencv\cvaux.h>
#include <string>
#include <time.h>

using namespace std;
//using namespace cv;

string NumberToString(int Number)
{
	stringstream ss;
	ss << Number;
	return ss.str();
}

//#define CAM_WIDTH 320//640
//#define CAM_HEIGHT 240//480
#define CHANNELS 3
#define UPDATE_PERIOD 2000
#define CLEAR_PERIOD  4000
#define LEARNING_TIME 100
#define NUM_OF_PIXEL_ON_LINE 500
#define WIDTH_CENTER_LINE	10

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
CvFont font = cvFont(2.0, 2);

typedef struct _Line {	
	CvPoint pt1;
	CvPoint pt2;
	CvPoint center = cvPoint((int)((pt1.x + pt2.x) / 2), (int)((pt1.y + pt2.y) / 2));
	int width;
	//struct _Line *subLine1;
	//struct _Line *subLine2;
} Line;


int count_in = 0, count_out = 0;
int mode = 2;	//HORIZONTAL MODE (DEFAULT)

//VARIABLES for CODEBOOK METHOD:
IplImage* tempImage = 0;
IplImage* rawImage = 0, *hsvImage = 0; //yuvImage is for codebook method
IplImage *ImaskCodeBook = 0, *ImaskCodeBookCC = 0;
CvBGCodeBookModel *model = 0;
const int NCHANNELS = 3;

IplImage* circleImage = 0;

// various tracking parameters (in seconds)
const double MHI_DURATION = 0.5;//1;
const double MAX_TIME_DELTA = 0.5;
const double MIN_TIME_DELTA = 0.05;
int duration = 1000;
int seg_thresh = 1500;
// number of cyclic frame buffer used for motion detection
// (should, probably, depend on FPS)
const int N = 4;

// ring image buffer
IplImage **buf = 0;
int last = 0;


// temporary images
IplImage* motion = 0;

IplImage* silh = 0;// 8U, 1-channel
IplImage *mhi = 0; // MHI 32F, 1-channel
IplImage *orient = 0; // orientation 32F, 1-channel
IplImage *mask = 0; // valid orientation mask 8U, 1-channel
IplImage *segmask = 0; // motion segmentation map 32F, 1-channel

CvMemStorage* storage = 0; // temporary storage
void  update_mhi(IplImage* img, IplImage* dst, int diff_threshold);


int idx = 0;
int saiso = 40;

typedef struct _Tracking {
	bool value = false;
	double exist_time = 10;		//	exist_time < line_w/2 ,  exist_time ~= update
	double start_time = (double)clock() / CLOCKS_PER_SEC;	//get current time (second);
	double time_count = 0;
}Tracking;
Tracking *countOnLine;// [NUM_OF_PIXEL_ON_LINE];//[CAM_HEIGHT];//[CamHeight];

/////////////////////////////////////////////
//void max_mod_Trackbar(int pos);
//void min_mod_Trackbar(int pos);

int select_object = 0;
int track_object = 0;

CvPoint origin;
CvRect selection;
CvRect track_window;
CvBox2D track_box;
void on_mouse(int event, int x, int y, int flags, void* param);

void InitImages(CvSize sz);
void ReleaseImages();

int modMin = 50, modMax = 10;	//codebook configuration
int h_min  = 10, h_max  = 155;	//hue mau sac
int s_min  = 0,  s_max  = 255;	//saturation do tuong phan
int v_min  = 0,  v_max  = 255;		//value do sang

void threshold_trackbar();
void silhoutte_reduction(IplImage* src, IplImage* dst, int codeCvt = CV_BGR2HSV);
void control_trackbar();
void onTrackbarSlide(int n) {
	cvSetCaptureProperty(capture, CV_CAP_PROP_POS_FRAMES, n);
}
void mode1(IplImage* image);
void mode2(IplImage* image);
//int update_period = 200, clear_peridod = 300;
int update_duration = 6, clear_duration = 60;// seconds
time_t start_update_time, start_clear_time, current_time;


int poly1Hull0 = 0;
int line_w = 20;

int min_len  = 650;
int max_len  = 5000;			//< 2000 - 5000
int min_area = 6000;
int max_area = 70000;//640 * 480;	// < 30 000 - 50 000
CvPoint center [50];

void reject_small_and_large_object(IplImage* src, IplImage* dst, IplImage* circleImage, CvPoint* center, int &numOfCenters, CvMemStorage* storage);


int main(int argc, int *argv[]) {
	if (capture = cvCreateCameraCapture(0)){
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
	}
	nframes = 0;
	nframesToLearnBG = LEARNING_TIME;
	
	//initialize codebook	
	model = cvCreateBGCodeBookModel();
	
	bool auto_update = false;
	bool update = false, clear = false;
	bool temp_image = false;
	bool reset = false;
	bool hsv = false;
	bool trackbar = true;
	int window_mode = 0;
	bool pause = false;
	bool save = false;
	bool color_window = true;

	cvNamedWindow("Camera");
	cvSetMouseCallback("Camera", on_mouse, 0);
	control_trackbar();
	threshold_trackbar();

	CvVideoWriter* writer = 0;
	//CvVideoWriter  *writer0 = 0,		*writer1 = 0,		*writer2 = 0,		*writer3 = 0,		*writer4 = 0;
	//IplImage	*tempImage0 = 0, *tempImage1 = 0, *tempImage2 = 0;// *tempImage3 = 0, *tempImage4 = 0;
	int fcc = CV_FOURCC('D', 'I', 'V', '3');
	
	while (1)
	{
		if (!pause) {
			rawImage = cvQueryFrame(capture);
			if (!rawImage) break;
			++nframes;
		}

		if (nframes == 1) {
			CvSize sz = cvGetSize(rawImage);
			InitImages(sz);

			//Set color thresholds to default values
			model->cbBounds[0] = model->cbBounds[1] = model->cbBounds[2] = 10;	//10
			model->modMin[0] = 70;	model->modMax[0] = 70;//H			
			model->modMin[1] = 10;	model->modMax[1] = 40;	//S
			model->modMin[2] = modMin;	model->modMax[2] = modMax;	//V
						
			selection.height = sz.height/2;
			selection.width = sz.width/2;

			CamWidth = (int)cvGetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH);
			CamHeight = (int)cvGetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT);
			center_x = CamWidth / 2;
			center_y = CamHeight / 2;

			count_in = 0;
			count_out = 0;

			if (writer) {
				cvReleaseVideoWriter(&writer);
				printf("start record new video\n");
			}
			writer = cvCreateVideoWriter("Video Record.avi", fcc, 24, sz);
			
			// record processing steps
			/*
			writer0 = cvCreateVideoWriter("foreground.avi", fcc, 24, sz);
			writer1 = cvCreateVideoWriter("connected component.avi", fcc, 24, sz);
			writer2 = cvCreateVideoWriter("circle.avi", fcc, 24, sz);
			writer3 = cvCreateVideoWriter("motion.avi", fcc, 24, sz);
			writer4 = cvCreateVideoWriter("hsv.avi", fcc, 24, sz);

			tempImage0 = cvCreateImage(sz, 8, 3);
			tempImage1 = cvCreateImage(sz, 8, 3);
			tempImage2 = cvCreateImage(sz, 8, 3);
			tempImage3 = cvCreateImage(sz, 8, 3);
			tempImage4 = cvCreateImage(sz, 8, 3);
			*/
		}

		if (rawImage)
		{							
			cvCvtColor(rawImage, hsvImage, CV_BGR2HSV);		//BGR -> HSV
			silhoutte_reduction(hsvImage, hsvImage, CV_BGR2HSV);

			if (update || (nframes - 1 < nframesToLearnBG)) {
				cvBGCodeBookUpdate(model, hsvImage);
				update = false;										//update codebook
				start_update_time = (double)clock() / CLOCKS_PER_SEC;
			}

			if (clear || (nframes - 1 == nframesToLearnBG)) {
				cvBGCodeBookClearStale(model, model->t / 2);
				clear = false;										//clear codebook
				start_clear_time = (double)clock() / CLOCKS_PER_SEC;
			}

			//Find the foreground if any
			if (nframes - 1 >= nframesToLearnBG) 
			{
				
				// Find foreground by codebook method
				cvBGCodeBookDiff(model, hsvImage, ImaskCodeBook);
								
				int numOfObject = 50;
				//reject_small_and_large_object(ImaskCodeBook, ImaskCodeBook, circleImage, center, numOfObject, storage);
				// This part just to visualize bounding boxes and centers if desired
				//cvCopy(ImaskCodeBook, ImaskCodeBookCC);				
				//cvSegmentFGMask(ImaskCodeBookCC, poly1Hull0, 4.0, 0, cvPoint(0, 0));							
				reject_small_and_large_object(ImaskCodeBook, ImaskCodeBookCC, circleImage, center, numOfObject, storage);
				update_mhi(circleImage, motion, 50);


			}

			if (select_object && selection.width > 0 && selection.height > 0)
			{
				cvRectangleR(rawImage, selection, cvScalar(0, 255, 255), 3);
				cvSetImageROI(rawImage, selection);
				cvXorS(rawImage, cvScalarAll(125), rawImage, 0);
				cvResetImageROI(rawImage);
			}

			//show on screen
			cvZero(tempImage);
			switch (window_mode)
			{
			case 1:
				cvMerge(ImaskCodeBook, 0, 0, 0, tempImage);
				break;
			case 2:
				cvMerge(0, 0, ImaskCodeBookCC, 0, tempImage);
				break;
			case 3:
				cvMerge(0, circleImage, 0, 0, tempImage);
				break;
			case 4:
				cvCopyImage(motion, tempImage);
				break;
			case 5:
				cvCopyImage(hsvImage, tempImage);
				break;
			case 0:
			default:
				cvCopyImage(rawImage, tempImage);
				window_mode = 0;
				break;
			}

			if (temp_image)	cvShowImage("Process Window", tempImage);
			else cvDestroyWindow("Process Window");
			
			
			// draw information
			if (mode == 1)	mode1(rawImage);	 //VERTICAL MODE
				
			if (mode == 2)	mode2(rawImage);		//HORIZONTAL MODE
		
			if (color_window)	cvShowImage("Camera", rawImage);
			else cvDestroyWindow("Camera");
			
			cvWriteFrame(writer, rawImage);
			
						
			/*
			cvCvtColor(ImaskCodeBook, tempImage0, CV_GRAY2BGR);
			cvCvtColor(ImaskCodeBookCC, tempImage1, CV_GRAY2BGR);
			cvCvtColor(circleImage, tempImage2, CV_GRAY2BGR);
			cvWriteFrame(writer0, tempImage0);// ImaskCodeBook);
			cvWriteFrame(writer1, tempImage1);// ImaskCodeBookCC);
			cvWriteFrame(writer2, tempImage2);// circleImage);
			cvWriteFrame(writer3, motion);
			cvWriteFrame(writer4, hsvImage);
			*/
			
			if (auto_update) {
				current_time = (double)clock() / CLOCKS_PER_SEC; // get current time in seconds
				//std::cout << current_time << endl;
				if (current_time - start_update_time >= update_duration)
				{
					//std::cout << current_time - start_update_time << endl;
					start_update_time = (double)clock() / CLOCKS_PER_SEC;
					update = true;
					printf("update\n");
				}
				
				if (current_time - start_clear_time >= clear_duration)
				{
					//std::cout << current_time - start_clear_time << endl;
					start_clear_time = (double)clock() / CLOCKS_PER_SEC;
					clear = true;
					printf("clear\n");
				}
	
			}

		}

		if (useAVIfile) {
			//pos = cvGetCaptureProperty(capture, CV_CAP_PROP_POS_FRAMES);
			//cvCreateTrackbar("frame", "Camera", &pos, maxFrame, onTrackbarSlide);
		}
		// User input:
		int key = cvWaitKey(10);
		//key = tolower(key);
		if (key == 27)	 break;	//end processing on ESC
		switch (key) {
			printf("key: %c\n", key);
		case 'a':
			auto_update = !auto_update;
			printf("auto update: %d\n", auto_update);
			break;
		case 'c':
			clear = true;
			printf("clear codebook\n");
			break;
		case 'u':
			update = true;
			printf("update codebook\n");
			break;
		case '1':
			window_mode = 1;
			printf("foreground image\n");
			break;
		case '2':
			window_mode = 2;
			printf("connected component image\n");
			break;
		case '3':
			window_mode = 3;
			printf("circle image\n");
			break;
		case '4':
			window_mode = 4;
			printf("motion image\n");
			break;
		case '5':
			window_mode = 5;
			printf("hsv image\n");
			break;
		case '6':
			window_mode = 0;
			printf("color image\n");
			break;
		case 's':
			cvSaveImage("ImaskCodeBook.jpg", ImaskCodeBook, 0);
			cvSaveImage("ImaskCodeBookCC.jpg", ImaskCodeBookCC, 0);
			cvSaveImage("circleImage.jpg", circleImage, 0);
			cvSaveImage("hsvImage.jpg", hsvImage, 0);
			cvSaveImage("rawImage.jpg", rawImage, 0);
			printf("save frame\n");
			break;
		case 'r':
			cvBGCodeBookClearStale(model, 0);
			//cvClearMemStorage(storage);
			ReleaseImages();
			nframes = 0;
			printf("reset\n");
			break;
		case 't':
			control_trackbar();
			threshold_trackbar();
			break;
		case 'q':
			temp_image = !temp_image;
			printf("Process Window\n");
			break;
		case 'w':
			color_window = !color_window;
			printf("color_window = %d\n", color_window);
			break;
		case 'v':
			mode = 1;
			if (countOnLine != NULL)
				delete[] countOnLine;
			firstInMode = true;
			printf("mode: %d-vertical\n", mode);
			break;
		case 'h':	//DEFAULT
			mode = 2;
			if (countOnLine != NULL)	
				delete[] countOnLine;
			firstInMode = true;
			printf("mode: %d-horizontal\n", mode);
			break;
		case 'p':
			pause = !pause;
			printf("pause = %d\n",pause);
			break;
		}

	}
	
	cvReleaseVideoWriter(&writer);
	/*
	cvReleaseVideoWriter(&writer0);
	cvReleaseVideoWriter(&writer1);
	cvReleaseVideoWriter(&writer2);
	cvReleaseVideoWriter(&writer3);
	cvReleaseVideoWriter(&writer4);
	*/
	return 0;
}

void  update_mhi(IplImage* img, IplImage* dst, int diff_threshold) {

	double timestamp = (double)clock() / CLOCKS_PER_SEC; // get current time in seconds
	CvSize size = cvGetSize(img); // get current frame size
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
	

	
	int detect_object(
		///	return 1: in	2: out		0: null  
		int detectable_direction,
		int object_center,
		int center_of_detect_line,
		int num_of_pixel_on_line,
		int direction_point,
		Tracking *track_line,
		int index_of_detected_object,
		double exist_time	//	exist_time < line_w/2
	);

	// iterate through the motion components,
	// One more iteration (i == -1) corresponds to the whole image (global motion)
	CvRect comp_rect;
	double count;
	double angle;
	CvMoments moments;
	//CvPoint momentcenter;
	double magnitude = 40;;
	for (i = -1; i < seq->total; i++) {

		if (i < 0) { // case of the whole image
			continue;
		}
		else { // i-th motion component
			comp_rect = ((CvConnectedComp*)cvGetSeqElem(seq, i))->rect;	//take the bounding rectangle for each motion
			/*if ((comp_rect.width + comp_rect.height < 100)// reject very small components
				//|| (comp_rect.width > selection.width) || (comp_rect.width < selection.width / 3)
				//|| (comp_rect.height > selection.height) || (comp_rect.height < selection.height / 3)
				)
				//continue;			
				printf("<100\n");*/
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

		//cvCircle(dst, momentcenter, 2, CV_RGB(255, 0, 0), -1);
		cvRectangleR(dst, comp_rect, CV_RGB(255, 0, 0));
		cvLine(rawImage, center[i], directionPoint, CV_RGB(100, 255, 0), 2);
		
		int isDetected = 0;
		if (mode == 1) {
			//vertical detective line
			if (firstInMode) {
				countOnLine = new Tracking[CamHeight];
				firstInMode = false;
			}			
			isDetected = detect_object(
				//momentcenter.x,
				//momentcenter.y,
				center[i].x,
				center[i].y,
				CamWidth / 2,
				CamHeight,
				directionPoint.x,
				countOnLine,
				idx,
				line_w / 2
			);	
		}
		
		if (mode == 2) {			
			//horizontal detective line
			if (firstInMode) {
				countOnLine = new Tracking[CamWidth];
				firstInMode = false;
			}
			isDetected = detect_object(
				//momentcenter.y,
				//momentcenter.x,
				center[i].y,
				center[i].x,
				CamHeight / 2,
				CamWidth,
				directionPoint.y,
				countOnLine,
				idx,
				line_w / 2
			);		
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
		printf("%d,%d w: %d h: %d \n", selection.x, selection.y, selection.width, selection.height);
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

void InitImages(CvSize sz)
{
	//	SILHOUTTE REDUCTION	
	tempImage = cvCloneImage(rawImage);

	// CODEBOOK METHOD ALLOCATION
	hsvImage = cvCloneImage(rawImage);
	cvZero(hsvImage);
	ImaskCodeBook = cvCreateImage(cvGetSize(rawImage), IPL_DEPTH_8U, 1);
	ImaskCodeBookCC = cvCreateImage(cvGetSize(rawImage), IPL_DEPTH_8U, 1);
	cvSet(ImaskCodeBook, cvScalar(255));
	
	circleImage = cvCreateImage(cvGetSize(rawImage), IPL_DEPTH_8U, 1);

	silh = cvCreateImage(cvGetSize(rawImage), IPL_DEPTH_8U, 1);	// 8U, 1-channel
	mhi = cvCreateImage(cvGetSize(rawImage), IPL_DEPTH_32F, 1);
	cvZero(mhi); // clear MHI at the beginning
	orient = cvCreateImage(cvGetSize(rawImage), IPL_DEPTH_32F, 1);
	segmask = cvCreateImage(cvGetSize(rawImage), IPL_DEPTH_32F, 1);
	mask = cvCreateImage(cvGetSize(rawImage), IPL_DEPTH_8U, 1);

	if (buf == 0) {
		buf = (IplImage**)malloc(N * sizeof(buf[0]));
		memset(buf, 0, N * sizeof(buf[0]));

	}
	for (int i = 0; i < 4; i++) {	//4 images
		cvReleaseImage(&buf[i]);
		buf[i] = cvCreateImage(cvGetSize(rawImage), IPL_DEPTH_8U, 1); //cvCreateImage(size, IPL_DEPTH_8U, 1);
		cvZero(buf[i]);
	}

	motion = cvCloneImage(rawImage);
	cvZero(motion);
}

void ReleaseImages()
{
	cvReleaseImage(&tempImage);
	
	cvReleaseImage(&silh);
	cvReleaseImage(&mhi);
	cvReleaseImage(&orient);
	cvReleaseImage(&segmask);
	/*for (int i = 0; i < 4; i++) {	//4 images
		cvReleaseImage(&buf[i]);
	}*/
	if(motion)	cvReleaseImage(&motion);
	cvReleaseImage(&hsvImage);
	cvReleaseImage(&ImaskCodeBook);
	cvReleaseImage(&ImaskCodeBookCC);

	cvReleaseImage(&circleImage);
}

void silhoutte_reduction(IplImage* src, IplImage* dst, int codeCvt )
{
	if (codeCvt != CV_BGR2HSV)
		cvCvtColor(src, src, codeCvt);
	static IplImage* h = cvCreateImage(cvGetSize(src), 8, 1);
	static IplImage* s = cvCreateImage(cvGetSize(src), 8, 1);
	static IplImage* v = cvCreateImage(cvGetSize(src), 8, 1);
	cvSplit(src, h, s, v, 0);
	
	cvThreshold(h, h, h_min, h_max, CV_THRESH_TOZERO);	//CV_THRESH_TOZERO = 3,  /* value = value > threshold ? value : 0           */
	cvThreshold(h, h, h_max, h_max, CV_THRESH_TOZERO_INV);	//CV_THRESH_TRUNC = 2,  /* value = value > threshold ? threshold : value   */
	cvThreshold(s, s, s_min, s_max, CV_THRESH_TOZERO);	//CV_THRESH_TOZERO_INV  =4,  /* value = value > threshold ? 0 : value           */
	cvThreshold(s, s, s_max, s_max, CV_THRESH_TOZERO_INV);
	cvThreshold(v, v, v_min, v_max, CV_THRESH_TOZERO);
	cvThreshold(v, v, v_max, v_max, CV_THRESH_TOZERO_INV);

	cvMerge(h, s, v, 0, dst);
}

void mode1(IplImage* image)
{
	//vertical detective line	
	CvScalar textColor = cvScalar(0, 0, 255);
	cvLine(image, cvPoint(center_x, 0), cvPoint(center_x, CamHeight), CV_RGB(255, 0, 0), 1);
	cvPutText(image, ("frame:" + NumberToString(nframes)).c_str(), cvPoint(0, 30), &font, textColor);
	cvPutText(image, ("OUT: " + NumberToString(count_out)).c_str(), cvPoint(0, 60), &font, textColor);
	cvPutText(image, ("IN: " + NumberToString(count_in)).c_str(), cvPoint(CamWidth - 90, 60), &font, textColor);

	cvLine(image, cvPoint(center_x - line_w, 0), cvPoint(center_x - line_w, CamHeight), CV_RGB(255, 0, 0), 1);
	cvLine(image, cvPoint(center_x + line_w, 0), cvPoint(center_x + line_w, CamHeight), CV_RGB(255, 0, 0), 1);
}

void mode2(IplImage* image)
{
	//horizontal detective line	
	CvScalar textColor = cvScalar(0, 0, 255);
	cvLine(image, cvPoint(0, center_y), cvPoint(CamWidth, center_y), CV_RGB(255, 0, 0), 1);
	cvPutText(image, ("frame:" + NumberToString(nframes)).c_str(), cvPoint(0, 30), &font, textColor);
	cvPutText(image, ("OUT: " + NumberToString(count_out)).c_str(), cvPoint(0, 60), &font, textColor);
	cvPutText(image, ("IN: " + NumberToString(count_in)).c_str(), cvPoint(0, CamHeight - 20), &font, textColor);

	cvLine(image, cvPoint(0, center_y - line_w), cvPoint(CamWidth, center_y - line_w), CV_RGB(255, 0, 0), 1);
	cvLine(image, cvPoint(0, center_y + line_w), cvPoint(CamWidth, center_y + line_w), CV_RGB(255, 0, 0), 1);
}

void control_trackbar()
{
	cvNamedWindow("Control");
	cvResizeWindow("Control", 300, 540);
	cvCreateTrackbar("update(s)", "Control", &update_duration, 120);	//seconds
	cvCreateTrackbar("clear(s)", "Control", &clear_duration, 5*60);		//seconds
	cvCreateTrackbar("poly1_hull0", "Control", &poly1Hull0, 1);
	cvCreateTrackbar("width", "Control", &line_w, 80);
	//cvCreateTrackbar("duration (ms)", "Control", &duration, 2000);
	//cvCreateTrackbar("seg_thresh (ms)", "Control", &seg_thresh, 2000);
	cvCreateTrackbar("min len", "Control", &min_len, 10000);
	cvCreateTrackbar("max len", "Control", &max_len, 10000);
	cvCreateTrackbar("min area", "Control", &min_area, 320*240);
	cvCreateTrackbar("max area", "Control", &max_area, 480*640);

}

void threshold_trackbar()
{
	std::string name = "Set YUV Background";
	cvNamedWindow(name.c_str());
	cvResizeWindow(name.c_str(), 260, 450);
	cvCreateTrackbar("modMin", name.c_str(), &modMin, 255);
	cvCreateTrackbar("modMax", name.c_str(), &modMax, 255);
	cvCreateTrackbar("h_min", name.c_str(), &h_min, 255);
	cvCreateTrackbar("h_max", name.c_str(), &h_max, 255);
	cvCreateTrackbar("s_min", name.c_str(), &s_min, 255);
	cvCreateTrackbar("s_max", name.c_str(), &s_max, 255);
	cvCreateTrackbar("v_min", name.c_str(), &v_min, 255);
	cvCreateTrackbar("v_max", name.c_str(), &v_max, 255);
}

void reject_small_and_large_object(IplImage* src, IplImage* dst, IplImage* circleImage, CvPoint* center, int &numOfCenters,CvMemStorage* storage) {
	CvSeq* contours = 0;
	if (storage == NULL)
		storage = cvCreateMemStorage(0);
	else cvClearMemStorage(storage);
	IplImage *temp;
	temp = cvCreateImage(cvGetSize(src), 8, 1);
	cvCopyImage(src, temp);	//input image
	cvZero(dst);			//output image
	cvZero(circleImage);
	
	IplConvKernel *element = cvCreateStructuringElementEx(10, 30, 5, 14, CV_SHAPE_RECT, 0);
	cvMorphologyEx(temp, temp, 0, 0, CV_MOP_OPEN, 2);
	cvMorphologyEx(temp, temp, 0, 0, CV_MOP_CLOSE, 2);

	CvContourScanner scanner = cvStartFindContours(temp, storage, sizeof(CvContour), 0, CV_CHAIN_APPROX_SIMPLE);
	CvSeq *c;
	int numContour = 0;
	/*if (center == NULL)
		//center = new CvPoint[numOfCenters];	//(IplImage**)malloc(N * sizeof(buf[0]));
		//center = (CvPoint*)malloc(numOfCenters * sizeof(center[0]));
	*/
	
	while ((c = cvFindNextContour(scanner)) != NULL) {
		double c_len = cvContourPerimeter(c);
		double c_area = cvContourArea(c);
		if (c_len < (double)min_len || c_area < (double)min_area
			|| c_len >(double)max_len || c_area >(double)max_area
			) {
			continue;
		}
		else {
			//printf("perimeter: %f\t area: %f\n", c_len, c_area);
			cvDrawContours(dst, c, cvScalarAll(255), cvScalarAll(0), 0, -1, 8, cvPoint(0, 0));
			CvMoments moments;
			CvRect rect = cvBoundingRect(c, 0);
			cvSetImageROI(dst, rect);
			cvMoments(dst, &moments, 1);
			double M00 = cvGetSpatialMoment(&moments, 0, 0);
			double M01 = cvGetSpatialMoment(&moments, 0, 1);
			double M10 = cvGetSpatialMoment(&moments, 1, 0);
			center[numContour].x = (int)(M10 / M00);
			center[numContour].y = (int)(M01 / M00);
			cvResetImageROI(dst);
			center[numContour] = cvPoint((rect.x + center[numContour].x), (rect.y + center[numContour].y));
			
			cvDrawCircle(circleImage, center[numContour], 20, cvScalarAll(255), -1, 8, 0);
			cvDrawCircle(rawImage, center[numContour], 10, cvScalar(255,0,0), -1, 8, 0);
			cvRectangleR(rawImage, rect, cvScalar(255, 255, 255), 1, 8, 0);
			cvPutText(rawImage, (NumberToString(center[numContour].x) + "," + NumberToString(center[numContour].y)).c_str(), center[numContour], &font, CV_RGB(255, 0, 0));
			
			numContour++;
			if (numContour > numOfCenters) {
				numContour = numOfCenters;
				break;
			}
		}
	}
	numOfCenters = numContour;
	cvEndFindContours(&scanner);
	cvReleaseImage(&temp);
	//delete[] center;
}
//function in mhi_update()
int detect_object(
	int detectable_direction,
	int object_center,
	int center_of_detect_line,
	int num_of_pixel_on_line,
	int direction_point,
	Tracking *track_line,
	int index_of_detected_object,
	double exist_time	//	exist_time < line_w/2 ,  exist_time ~= update
) {
	/*
	*	return 1: in	2: out		0: null
	*/
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
		///xac dinh huong di chuyen
		if (_isDetected) {
			if (direction_point - detectable_direction > 0) { return 1; }	//input
			else { return 2; }	//output
		}
	}
	return 0;
}