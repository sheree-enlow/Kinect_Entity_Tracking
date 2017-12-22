#pragma once
#include "Entity.h"
#include <string>

using namespace std;
using namespace cv;

/* THE PIPE FILE DESCRIPTORS */
extern int fd[2];// pipe file descriptors.


const cv::Scalar SCALAR_BLACK = cv::Scalar(0.0, 0.0, 0.0);
const cv::Scalar SCALAR_WHITE = cv::Scalar(255.0, 255.0, 255.0);
const cv::Scalar SCALAR_YELLOW = cv::Scalar(0.0, 255.0, 255.0);
const cv::Scalar SCALAR_GREEN = cv::Scalar(0.0, 200.0, 0.0);
const cv::Scalar SCALAR_RED = cv::Scalar(0.0, 0.0, 255.0);

class EntityTracker
{
public:
	std::vector<Entity> currentFrameEntitys;
	std::vector<Entity> existingEntitys;

	EntityTracker();
	void HeadTrack();
	void drawLine(Mat &image);


	void matchCurrentFrameEntitysToExistingEntitys(std::vector<Entity> &existingEntitys, std::vector<Entity> &currentFrameEntitys);
	void addEntityToExistingEntitys(Entity &currentFrameEntity, std::vector<Entity> &existingEntitys, int &intIndex);
	void addNewEntity(Entity &currentFrameEntity, std::vector<Entity> &existingEntitys);
	double distanceBetweenPoints(cv::Point point1, cv::Point point2);
	void drawAndShowContours(cv::Size imageSize, std::vector<std::vector<cv::Point> > contours, std::string strImageName);
	void drawAndShowContours(cv::Size imageSize, std::vector<Entity> blobs, std::string strImageName);
	bool checkIfEntitysCrossedTheLine(std::vector<Entity> &blobs, int &intHorizontalLinePosition);
	void drawEntityInfoOnImage(std::vector<Entity> &blobs, cv::Mat &imgFrame2Copy);
	void drawInOutCountOnImage(int &InCount, int &OutCount, cv::Mat &imgFrame2Copy);
	void drawEntityPaths(Mat &image);
	
	void drawCircles ( Mat &image, bool cross );
	bool checkIfEntitysCrossedTheCircle(std::vector<Entity> &blobs);// new func
	
	
    string CASAS_HEADCOUNT();// deliver data about the height location etc of all of the blobs still being tracked
    string CASAS_HeadIn();// delivers the incount
    string CASAS_HeadOut(); // delivers the outcount
    
    
	bool firstFrame;
	cv::Point crossingLine[2];
	int InCount; //decide in, out, and totals later
	int OutCount;

	unsigned int EntityID; // so we can distinguish each blob from one another.
	ushort distance2Floor; // distance to the floor;
	const int radius = 260; // the radius of the first circle "crossing line"
	bool IN1;// did we hit the first circle radius
	bool IN2;// did we hit the second circle radius

};
