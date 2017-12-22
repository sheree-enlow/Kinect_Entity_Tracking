#include "EntityTracker.h"

//constructor
EntityTracker::EntityTracker()
{
        //zero these out so we don't have random memory location value
        distance2Floor = EntityID = InCount = OutCount = 0;
        firstFrame = true;
}

void EntityTracker::HeadTrack()
{
//

        if ( firstFrame == true ) { // if we dont have two frames to compare too
                for ( auto & entityx : currentFrameEntitys ) {
                        addNewEntity ( entityx,existingEntitys );
                }
                firstFrame = false;
        } else {
                matchCurrentFrameEntitysToExistingEntitys ( existingEntitys, currentFrameEntitys );
        }


}

void EntityTracker::drawEntityPaths ( Mat &image )
{
        for ( auto entity : existingEntitys ) { // for each existing entity we need to draw a path
                for ( int i = 1; i < entity.centerPositions.size(); i++ ) {
                        line ( image, Point ( entity.centerPositions[i - 1].x, entity.centerPositions[i - 1].y ), Point ( entity.centerPositions[i].x, entity.centerPositions[i].y ), SCALAR_GREEN, 2 );
                }
        }
}


void EntityTracker::matchCurrentFrameEntitysToExistingEntitys ( std::vector<Entity> &existingEntitys, std::vector<Entity> &currentFrameEntitys )
{

        for ( auto &existingEntity : existingEntitys ) {

                existingEntity.blnCurrentMatchFoundOrNewEntity = false;

                existingEntity.predictNextPosition();
        }

        for ( auto &currentFrameEntity : currentFrameEntitys ) {

                int intIndexOfLeastDistance = 0;
                double dblLeastDistance = 100000.0;

                for ( unsigned int i = 0; i < existingEntitys.size(); i++ ) {

                        if ( existingEntitys[i].blnStillBeingTracked == true ) {

                                double dblDistance = distanceBetweenPoints ( currentFrameEntity.centerPositions.back(), existingEntitys[i].predictedNextPosition );

                                if ( dblDistance < dblLeastDistance ) {
                                        dblLeastDistance = dblDistance;
                                        intIndexOfLeastDistance = i;
                                }
                        }
                }
                //.5
                if ( dblLeastDistance < currentFrameEntity.dblCurrentDiagonalSize * .75 ) {
                        addEntityToExistingEntitys ( currentFrameEntity, existingEntitys, intIndexOfLeastDistance );
                        // Here we can relay information about already in view entitys
                } else {

                        addNewEntity ( currentFrameEntity, existingEntitys );
                        // here we can talk about
                }

        }

        for ( int i = 0; i < existingEntitys.size(); i++ ) {
                if ( existingEntitys[i].blnCurrentMatchFoundOrNewEntity == false ) {
                        existingEntitys[i].intNumOfConsecutiveFramesWithoutAMatch++;
                        // we couldnt find a match for our current entity. This accounts for entitys dissappearing
                        // on our barrier and persisting there for 10 frames (causing an incorrect count).
                        
                        existingEntitys[i].blnStillBeingTracked = false;
                }

                if ( existingEntitys[i].intNumOfConsecutiveFramesWithoutAMatch >= 10 ) {
                        // trying this out lets just delete this so we can keep the vector size small.
                        existingEntitys.erase ( existingEntitys.begin() + i-- );
                }

        }

}

void EntityTracker::addEntityToExistingEntitys ( Entity &currentFrameEntity, std::vector<Entity> &existingEntitys, int &intIndex )
{

        existingEntitys[intIndex].currentContour = currentFrameEntity.currentContour;
        existingEntitys[intIndex].currentBoundingRect = currentFrameEntity.currentBoundingRect;

        existingEntitys[intIndex].centerPositions.push_back ( currentFrameEntity.centerPositions.back() );
        existingEntitys[intIndex].heights.push_back ( currentFrameEntity.heights.back() ); // add our heights.
        existingEntitys[intIndex].dblCurrentDiagonalSize = currentFrameEntity.dblCurrentDiagonalSize;
        existingEntitys[intIndex].dblCurrentAspectRatio = currentFrameEntity.dblCurrentAspectRatio;

        existingEntitys[intIndex].blnStillBeingTracked = true;
        existingEntitys[intIndex].blnCurrentMatchFoundOrNewEntity = true;
        existingEntitys[intIndex].intNumOfConsecutiveFramesWithoutAMatch = 0;
}

void EntityTracker::addNewEntity ( Entity &currentFrameEntity, std::vector<Entity> &existingEntitys )
{

        currentFrameEntity.blnCurrentMatchFoundOrNewEntity = true;

        if ( EntityID >= UINT_MAX-5 ) { // just in case we have some issues objects that become part of the background.
                EntityID = 0;
        }

        currentFrameEntity.ID = ++EntityID;

        existingEntitys.push_back ( currentFrameEntity );
}

double EntityTracker::distanceBetweenPoints ( cv::Point point1, cv::Point point2 )
{

        int intX = abs ( point1.x - point2.x );
        int intY = abs ( point1.y - point2.y );

        return ( sqrt ( pow ( intX, 2 ) + pow ( intY, 2 ) ) );
}

void EntityTracker::drawAndShowContours ( cv::Size imageSize, std::vector<std::vector<cv::Point> > contours, std::string strImageName )
{
        cv::Mat image ( imageSize, CV_8UC3, SCALAR_BLACK );

        cv::drawContours ( image, contours, -1, SCALAR_WHITE, -1 );

        cv::imshow ( strImageName, image );
}

void EntityTracker::drawAndShowContours ( cv::Size imageSize, std::vector<Entity> entitys, std::string strImageName )
{

        cv::Mat image ( imageSize, CV_8UC3, SCALAR_BLACK );

        std::vector<std::vector<cv::Point> > contours;

        for ( auto &entity : entitys ) {
                if ( entity.blnStillBeingTracked == true ) {
                        contours.push_back ( entity.currentContour );
                }
        }

        cv::drawContours ( image, contours, -1, SCALAR_WHITE, -1 );

        cv::imshow ( strImageName, image );
}

bool EntityTracker::checkIfEntitysCrossedTheLine ( std::vector<Entity> &entitys, int &intHorizontalLinePosition )
{
        bool blnAtLeastOneEntityCrossedTheLine = false;

        for ( auto entity : entitys ) {

                if ( entity.blnStillBeingTracked && entity.centerPositions.size() >= 2 ) {
                        int prevFrameIndex = ( int ) entity.centerPositions.size() - 2;
                        int currFrameIndex = ( int ) entity.centerPositions.size() - 1;
                        // checking to see if we came from the bottom to top. Think of image as a matrix not Cartesian plane
                        if ( entity.centerPositions[prevFrameIndex].y > intHorizontalLinePosition &&
                                        entity.centerPositions[currFrameIndex].y <= intHorizontalLinePosition ) {
                                InCount++;

                                blnAtLeastOneEntityCrossedTheLine = true;
                        }
                        // came from top to bottom
                        else if ( entity.centerPositions[prevFrameIndex].y < intHorizontalLinePosition &&
                                        entity.centerPositions[currFrameIndex].y >= intHorizontalLinePosition ) {
                                OutCount++;

                                blnAtLeastOneEntityCrossedTheLine = true;
                        }
                }

        }

        return blnAtLeastOneEntityCrossedTheLine;
}


void EntityTracker::drawInOutCountOnImage ( int &InCount, int &OutCount, cv::Mat &imgFrame2Copy )
{

        int intFontFace = CV_FONT_HERSHEY_SIMPLEX;
        double dblFontScale = ( imgFrame2Copy.rows * imgFrame2Copy.cols ) / 30000.0; // took away 2 zeros
        int intFontThickness = ( int ) std::round ( dblFontScale * 1.5 );

        cv::Size textSize = cv::getTextSize ( std::to_string ( InCount ), intFontFace, dblFontScale, intFontThickness, 0 );
        cv::Size textSize2 = cv::getTextSize ( std::to_string ( OutCount ), intFontFace, dblFontScale, intFontThickness, 0 );

        cv::Point ptInPosition;
        cv::Point ptOutPosition;

        ptInPosition.x = imgFrame2Copy.cols - 1 - ( int ) ( ( double ) textSize.width * 1.25 );
        ptInPosition.y = ( int ) ( ( double ) textSize.height * 1.25 + 10 );

        ptOutPosition.x = imgFrame2Copy.cols - 1 - ( int ) ( ( double ) textSize2.width * 1.25 );
        ptOutPosition.y = ( int ) ( ( double ) textSize.height * 1.25 + 90 );

        cv::putText ( imgFrame2Copy, std::to_string ( InCount ), ptInPosition, intFontFace, dblFontScale, SCALAR_GREEN, intFontThickness );
        cv::putText ( imgFrame2Copy, std::to_string ( OutCount ), ptOutPosition, intFontFace, dblFontScale, SCALAR_GREEN, intFontThickness );



}


string EntityTracker::CASAS_HEADCOUNT() // deliver data about the height location etc of all of the entitys still being tracked
{
// THIS ASSUMES THAT THERE IS ACTUAL HEAD COUNT DATA. Will break otherwise size == 0
        /*
        by="KinectAgent"
        packagetype="Kinect"
        sensortype="Entitys"
        serial="12345678"
        target="MainDoorEntitys"
        message="[{'entityid':'2', 'x':'56', 'y':'83', 'height':'9000'},{'entityid':'6', 'x':'22', 'y':'90', 'height':'12'}]" "[]"
        category="entity"
        channel="
        Casas::publishData(const std::string& by, const std::string& packagetype,
        		const std::string& sensortype, const std::string& serial,
        		const std::string& target, const std::string& message,
        		const std::string& category, const std::string& channel)
        */
        string  msg = "[";

        if ( currentFrameEntitys.size() >0 ) {

                // well double check the case where nothings in the frame
                // but not needed for our implementations since we already track

                for ( auto & entity: existingEntitys ) {
                        if ( entity.blnCurrentMatchFoundOrNewEntity == true ) {
                                msg+="{";// start of value;
                                msg+="\"id\":"+to_string ( entity.ID ) +",";
                                msg+="\"x\":" + to_string ( entity.centerPositions.back().x ) + ",";
                                msg+="\"y\":" + to_string ( entity.centerPositions.back().y ) + ",";
                                msg+="\"height\":" + to_string ( entity.MeanHeight() );
                                msg+="},";// start of value;
                        }
                }

                msg[msg.size()-1] = ']';// get rid of the last comma
        } else {
                msg+="]";
        }

        return msg;
}

string EntityTracker::CASAS_HeadIn() // delivers the incount
{
        return ( to_string ( InCount ) );
}


string EntityTracker::CASAS_HeadOut() // delivers the outcount
{
        return ( to_string ( OutCount ) );
}


// cross = did it cross the line?
void EntityTracker::drawCircles ( Mat & image, bool cross )
{
        //circle(Mat& img, Point center, int radius, const Scalar& color, int thickness=1, int lineType=8, int shift=0)
        //circle ( image, Point ( 120, 320 ), radius, Scalar ( 0, 0, 255 ), 1, 0 );
        //circle ( image, Point ( 120, 320-20 ), radius+20, Scalar ( 255, 0, 255 ), 1, 0 );
// 240, 320,
        cvtColor ( image, image, CV_GRAY2RGB );

        if ( cross ) {
                circle ( image, Point ( 160,430 ), radius, SCALAR_YELLOW, 6, 0 );
                circle ( image, Point ( 160,430-30 ), radius+20, SCALAR_YELLOW, 6, 0 );
        } else {
                circle ( image, Point ( 160,430 ), radius,SCALAR_RED, 6, 0 );
                circle ( image, Point ( 160,430-30 ), radius+20,SCALAR_RED, 6,cv::LINE_8 ,0 );
        }

}

bool EntityTracker::checkIfEntitysCrossedTheCircle ( std::vector<Entity> &entitys ) // new func
{


        bool blnAtLeastOneEntityCrossedTheLine = false;

        for ( auto & entity : entitys ) {

                if ( entity.blnStillBeingTracked  && entity.centerPositions.size() >= 2 ) {
                        int prevFrameIndex = ( int ) entity.centerPositions.size() - 2;
                        int currFrameIndex = ( int ) entity.centerPositions.size() - 1;


                        // hhhz to see if we came from the bottom to top. Think of image as a matrix not Cartesian plane
                        if ( distanceBetweenPoints ( entity.centerPositions[prevFrameIndex],  Point ( 160,430 ) ) > radius &&
                                        distanceBetweenPoints ( entity.centerPositions[currFrameIndex],  Point ( 160,430 ) ) <= radius ) {



                                if ( entity.outerCir ) {
                                        entity.outerCir= false;
                                        OutCount++;
                                        blnAtLeastOneEntityCrossedTheLine = true;
                                        //cout << entity.innerCir << entity.outerCir << InCount<< OutCount << endl;
                                }

                                entity.innerCir = true;



                        } else if ( distanceBetweenPoints ( entity.centerPositions[prevFrameIndex],  Point ( 160,430 ) ) < radius &&
                                        distanceBetweenPoints ( entity.centerPositions[currFrameIndex],  Point ( 160,430 ) ) >= radius ) {

                                entity.innerCir = true;

                        } else if ( distanceBetweenPoints ( entity.centerPositions[prevFrameIndex],  Point ( 160,430-30 ) ) > radius+20 &&
                                        distanceBetweenPoints ( entity.centerPositions[currFrameIndex],  Point ( 160,430-30 ) ) <= radius+20 ) {



                                entity.outerCir = true;

                        } else if ( distanceBetweenPoints ( entity.centerPositions[prevFrameIndex],  Point ( 160,430-30 ) ) < radius+20 &&
                                        distanceBetweenPoints ( entity.centerPositions[currFrameIndex],  Point ( 160,430-30 ) ) >= radius+20 ) {

                                if ( entity.innerCir ) {
                                        entity.innerCir = false;
                                        InCount++;
                                        blnAtLeastOneEntityCrossedTheLine = true;
                                        // cout << entity.innerCir << entity.outerCir << InCount<< OutCount << endl;
                                }

                                entity.outerCir = true;


                        }

                }

        }

        return blnAtLeastOneEntityCrossedTheLine;

}

