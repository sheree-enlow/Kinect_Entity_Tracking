#include <string>
#include <iostream>
#include <vector>
#include <cmath>
#include <pthread.h>
#include <cxcore.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include "WaterFill.h"

#include <iostream>
#include <string>
#include <map>

using namespace cv;
using namespace std;




int main ( int argc, char **argv )
{
        bool die ( false );
        // Casas * cbot;// gloox XMMP communication.
        CWaterFill alg;
        alg.Initialise();

        //the macaddr of our computer. The interface needs to be changed manually
        //string MACADDR = MacAddr();

        vector<RECT> contourBox;

        int i_snap ( 0 ),iter ( 0 );

        Mat depthMat ( Size ( 640,480 ),CV_16UC1 );




        alg.depthMap = imread ( "data_CASAS/data"+to_string ( iter ) +".png",CV_LOAD_IMAGE_ANYDEPTH );

        while ( iter  <300 ) { // train background subtractor

                alg.TrainBgSub ( alg.depthMap );
                iter ++;
        }


        vector<ushort> floorDist;


        for ( int i =0; i < 30; i++ ) {
                // GET THE INITIAL DISTANCE TO THE FLOOR FOR HEIGHT CALCULATIONS
                cvWaitKey ( 5 );
                // 480 rows and 640 columns

                for ( int j = 237; j < 243 ; j++ ) {
                        const ushort* Mi =  alg.depthMap.ptr<ushort> ( j );
                        for ( int x = 317; x < 322; x++ ) {
                                if ( Mi[x]!= 0 ) {
                                        floorDist.push_back ( Mi[x] );
                                }
                        }

                }



        }


        //Go over the entire image and only add distances that are within 1 std's away from our intialization
        for ( int i =0; i < 30; i++ ) {
                // GET THE INITIAL DISTANCE TO THE FLOOR FOR HEIGHT CALCULATIONS

                double sum = std::accumulate ( std::begin ( floorDist ), std::end ( floorDist ), 0.0 );
                double m =  sum / floorDist.size();

                double accum = 0.0;
                std::for_each ( std::begin ( floorDist ), std::end ( floorDist ), [&] ( const double d ) {
                        accum += ( d - m ) * ( d - m );
                } );

                double stdev = sqrt ( accum / ( floorDist.size()-1 ) );

                cvWaitKey ( 5 );


                // Find the max distance from the kinect and assume its floor distance
                for ( int j = 0; j < alg.depthMap.rows; j++ ) {
                        const ushort* Mi =  alg.depthMap.ptr<ushort> ( j );
                        for ( int x = 0; x <  alg.depthMap.cols; x++ ) {

                                if ( ! ( Mi[x] >  m + stdev ) && ! ( Mi[x] < m-stdev ) ) {
                                        floorDist.push_back ( Mi[x] );
                                }

                        }
                }


        }

        alg.headTracker.distance2Floor = ( ( accumulate ( begin ( floorDist ),end ( floorDist ), 0.0 ) ) /floorDist.size() );

        std:: cout << alg.headTracker.distance2Floor << endl;

        iter = 1;

        unordered_map<int,vector<int>> heightDict;
        int in1 =0;
        int out1 = 0;
        bool slow = false;
//2140-3744
        while ( !die ) {




                char k;

                if ( slow ) {

                        k = cvWaitKey ( 1000 );
                } else {
                        k = cvWaitKey ( 5 );
                }

                if ( iter == 4907 ) {
                        cvWaitKey ( 10000 );
                        cout<<"In: " + to_string ( in1 ) +" Out:  "+to_string ( out1 ) << endl;
                }

                if ( k == 'p' ) {
                        cout<< iter <<endl;

                        cvWaitKey ( 3000 );
                } else if ( k =='r' ) {
                        iter -= 10;
                        cvWaitKey ( 500 );
                } else if ( k == 'w' ) {
                        iter = 501;
                } else if ( k == ']' ) {
                        out1++;
                        cout<<"In: " + to_string ( in1 ) +" Out:  "+to_string ( out1 ) << endl;
                } else if ( k == ']' )

                {
                        out1++;
                        cout<<"In: " + to_string ( in1 ) +" Out:  "+to_string ( out1 ) << endl;
                } else if ( k == 's' ) {
                        slow = true;
                }

                iter++;

                if ( ( 2129 <= iter && iter <=3754 ) ) {
                        iter = 3755;
                }

                alg.depthMap = imread ( "data_CASAS/data"+to_string ( iter ) +".png",CV_LOAD_IMAGE_ANYDEPTH );

                alg.Water ( iter,12,&contourBox,1 );

        }

        return 0;
}


























