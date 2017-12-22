
#ifndef ENTITY_H
#define ENTITY_H

#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include <iostream>
using namespace std;

class Entity
{
public:
    vector<cv::Point> currentContour;

    cv::Rect currentBoundingRect;

    vector<cv::Point> centerPositions;
    vector<int64_t> heights;// taken from the centerpositions x,y respectively.

    double dblCurrentDiagonalSize;
    double dblCurrentAspectRatio;

    bool blnCurrentMatchFoundOrNewEntity;

    bool blnStillBeingTracked;

    int intNumOfConsecutiveFramesWithoutAMatch;

    cv::Point predictedNextPosition;

    //funcs
    Entity ( cv::Rect bdRect,ushort floorDist,cv::Mat img );
    int64_t MeanHeight();
    void predictNextPosition ( void );

    unsigned int ID;
    // inner and outer circle crossed?
    bool outerCir;
    bool innerCir;

};

#endif   
