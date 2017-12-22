#pragma once

//#include "Open_Kinect.h"

//#include <cv.h>
//#include <cxcore.h>
//#include <highgui.h>
#include <opencv/cvaux.h>//背景建模的头文件
//#include <windows.h>
#include <vector>
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/video/background_segm.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <time.h>

//#include <opencv2\opencv.hpp>
#include "Entity.h"
#include "EntityTracker.h"


using namespace cv;
using namespace std;

/* REPLACED BY Entity
class Person// each one of these represents a person.
{
public:

	vector<RECT> inFrame;//the frames in which our person is "visual"
	vector<ushort> heights;// in millimeters
	int countDown; // countdown the number of frames until this person becomes "unaccounted for"
	bool inView;// was the variable in view in the last frame?
	Person();
	ushort Height();

	long In; //count of people who have entered
	long Out; //count of poeple who have left

};
*/

typedef struct _RECT {
    long left;
    long top;
    long right;
    long bottom;
} RECT;


class CWaterFill
{
public:
    void Initialise();

    Mat WaterDrop ( Mat InputImg, int num_of_drop );
    Mat WaterFilter ( Mat inputImg, int mimNum );

    Mat HalfSizeImg ( Mat SrcImg );
    Mat Sixth2Eight ( Mat SrcImg, int ratio );


    void GMM2 ( Mat InputImg, double learnRate );
    void AndOpera ( Mat SrcImg, Mat* DestImg );
    Mat ContourFilter ( Mat Img, int minSize );
    void MergeBlack ( Mat* pImg );
    void GetHead_Min ( Mat Img, vector<RECT>* detectBox );
    void GetHead ( Mat Img, vector<RECT>* detectBox );

    void Water ( int nFrame, int threshold, vector<RECT>* detectBox, int stype=1 );
    void showImage ( Mat map,string name );


    void Image2File ( Mat map, string name );

    void TrainBgSub ( Mat train ); // train the background subtractor


    Mat depthMap;
    Mat rgbImage;
    Mat minRGB;
    Mat save_minRGB;
    Mat minImg;
    Mat waterImg;
    Mat img_16bit;
    Mat img_8bit;
    Mat waterImg_8bit;
    Mat BgImg;


    EntityTracker headTracker;

    time_t theTime = time ( NULL );
    struct tm *aTime = localtime ( &theTime );
    //int hour = aTime->tm_hour;
    //int min = aTime->tm_min;



protected:
    //BackgroundSubtractorMOG2 m_bg_model;//定义高斯背景模型

    Ptr<BackgroundSubtractor> m_bg_model;
    //Ptr<BackgroundSubtractor> test_sub;

    //Ptr<BackgroundSubtractorMOG2> m_bg_model; //working method :)

    Mat m_fgmask;//背景建模之后形成的前景目标
};

