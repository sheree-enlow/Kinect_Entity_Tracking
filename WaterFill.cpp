#include "WaterFill.h"


void CWaterFill::Initialise()
{
        m_bg_model = createBackgroundSubtractorMOG2();//createBackgroundSubtractorKNN(); //jared wants this somewhere else
        //test_sub = createBackgroundSubtractorMOG2();
        depthMap.create ( 480,640,CV_16UC1 );
        minRGB.create ( 240, 320, CV_8UC3 );
        save_minRGB.create ( 240, 320, CV_8UC3 );
        minImg.create ( 240, 320, CV_16UC1 );
        waterImg.create ( 240, 320, CV_16UC1 );
        img_16bit.create ( 240,320, CV_16UC1 );
        img_8bit.create ( 240, 320, CV_8UC1 );
        waterImg_8bit.create ( 240, 320, CV_8UC1 );
        BgImg.create ( 240, 320, CV_8UC1 );

}

// find the countours around waterfilled areas.
void CWaterFill::GetHead ( Mat Img, vector<RECT>* detectBox ) // Where to add in code
{
        CvMemStorage* m_storage;
        m_storage = cvCreateMemStorage ( 0 );

        // clear out our current frameentitys since we're starting a new frame
        headTracker.currentFrameEntitys.clear();

        CvSeq *m_first_seq = NULL;
        CvSeq *m_prev_seq = NULL;
        CvSeq *m_seq = NULL;

        CvPoint pt1,pt2;

        Mat inputcopy;
        Img.copyTo ( inputcopy );
        IplImage* pp = &IplImage ( inputcopy );

        
        cvClearMemStorage ( m_storage );
        //m_first_seq = cvCreateSeq(0, sizeof(CvSeq), sizeof(CvPoint), m_storage); ///sheree put this here

        int head = cvFindContours ( pp, m_storage, &m_first_seq, sizeof ( CvContour ), CV_RETR_LIST );
        for ( m_seq = m_first_seq; m_seq; m_seq = m_seq->h_next ) {
                CvContour* cnt = ( CvContour* ) m_seq;

                /* hacky having too draw two rectangles */



                pt1.x = ( cnt->rect.x );
                pt1.y = ( cnt->rect.y );
                pt2.x = ( cnt->rect.x + cnt->rect.width );
                pt2.y = ( cnt->rect.y + cnt->rect.height );
		
		if(cnt->rect.width * 1.5 >= cnt->rect.height && cnt->rect.width <= 1.5 *cnt->rect.height)
		{
                Rect potentialEntity ( cnt->rect ); // current potential entity for head tracking
		

                headTracker.currentFrameEntitys.push_back ( Entity ( potentialEntity,headTracker.distance2Floor,minImg ) );

                RECT head;
                head.left = pt1.x;
                head.right = pt2.x;
                head.top = pt1.y;
                head.bottom = pt2.y;
                detectBox->push_back ( head );

                rectangle ( img_8bit, pt1, pt2, CV_RGB ( 255,255,255 ), -1 );
		}

        }

        //new code... adding line to image and tracking heads (adding entitys to vector)

        
	  headTracker.drawCircles(img_8bit,headTracker.checkIfEntitysCrossedTheCircle(headTracker.existingEntitys));
        // if there's any entitys in the frame track their movements to ensure in/out.
        // Send data to the CASAS about any new participants to the frame and their heights.


        // now that we've matched entitys from previous frames (or created new ones
        // This is where head frame data is shipped off to the CASAS via Brain's XMPP CODE

        headTracker.HeadTrack();


        headTracker.drawEntityPaths ( img_8bit );
        headTracker.drawInOutCountOnImage ( headTracker.InCount, headTracker.OutCount, img_8bit );



        //send message about each of the entitys and about height
        //hit's midnight reset
        time_t rawtime;
        struct tm * timeinfo;

        time ( &rawtime );
        timeinfo = localtime ( &rawtime );

        if ( timeinfo->tm_hour == 0 &&  timeinfo->tm_min == 0 ) {
                headTracker.InCount = 0;
                headTracker.OutCount =0;
                headTracker.EntityID = 0;
        }
       

        cvReleaseMemStorage ( &m_storage );






}



//on areas where nothing was read or in gaps we take the mean of the surronding depth pixels
// and substitute that in instead.
void CWaterFill::MergeBlack ( Mat* pImg )
{
        typedef ushort imgType;
        int radius= 30;

        double pData = 0;
        int num = 0;
        int staPoint_x = 0;
        int staPoint_y = 0;
        int endPoint_x = 0;
        int endPoint_y = 0;
        int width = pImg->cols;
        int height = pImg->rows;

        imgType* m_a = NULL;
        imgType* m_b = NULL;

        m_a = ( imgType* ) pImg->data;
        for ( int y=0; y<height; ++y ) { // loop through each row
                for ( int x=0; x<width; ++x, ++m_a ) { // each column
                        if ( *m_a == 0 ) {
                                if ( y-radius/2 <= 0 ) {
                                        staPoint_y = 0;
                                } else {
                                        staPoint_y = y-radius/2;
                                }
                               
                                if ( x-radius/2 <= 0 ) {
                                        staPoint_x = 0;
                                } else {
                                        staPoint_x = x-radius/2;
                                }
                               
                                if ( y+radius/2 >= height ) {
                                        endPoint_y = height;
                                } else {
                                        endPoint_y = y+radius/2;
                                }
                               //
                                if ( x+radius/2 >= width ) {
                                        endPoint_x = width;
                                } else {
                                        endPoint_x = x+radius/2;
                                }
                               //

                                m_b = ( imgType* ) pImg->data;
                                m_b += staPoint_y*width;
                                for ( int i=staPoint_y; i<endPoint_y; ++i,m_b+=width ) {
                                        for ( int j=staPoint_x; j<endPoint_x; ++j ) {
                                                if ( m_b[j] !=1 && m_b[j] !=0 ) {
                                                        pData += m_b[j];
                                                        ++num;
                                                }
                                        }
                                }
                                pData = pData/num;// average intensity that isn't 1 or a zero
                                pImg->at<imgType> ( y,x ) = ( imgType ) pData;
                                pData = 0;
                                num = 0;
                        }
                }
        }
}


//filter out countour boxes that are too small
Mat CWaterFill::ContourFilter ( Mat Img, int minSize )
{
        int maxValue = 255;

        CvMemStorage* m_storage;
        m_storage = cvCreateMemStorage ( 0 );

        int m_region_count = 0;
        CvSeq *m_first_seq = NULL;
        CvSeq *m_prev_seq = NULL;
        CvSeq *m_seq = NULL;

        IplImage* ppp = new IplImage ( Img );
        //printf("Passed -excetion flag\n");
        cvMorphologyEx ( ppp, ppp, 0, 0, CV_MOP_OPEN , 1 );
        cvMorphologyEx ( ppp, ppp, 0, 0, CV_MOP_CLOSE, 1 );

        //
        cvClearMemStorage ( m_storage );
        cvFindContours ( ppp, m_storage, &m_first_seq, sizeof ( CvContour ), CV_RETR_LIST );
        for ( m_seq = m_first_seq; m_seq; m_seq = m_seq->h_next ) {
                CvContour* cnt = ( CvContour* ) m_seq;
                if ( cnt->rect.width * cnt->rect.height < minSize) {
                        m_prev_seq = m_seq->h_prev;
                        if ( m_prev_seq ) {
                                m_prev_seq->h_next = m_seq->h_next;
                                if ( m_seq->h_next ) m_seq->h_next->h_prev = m_prev_seq;
                        } else {
                                m_first_seq = m_seq->h_next;
                                if ( m_seq->h_next ) m_seq->h_next->h_prev = NULL;
                        }
                } else {
                        m_region_count++;
                }
        }

        cvZero ( ppp );

        cvDrawContours ( ppp, m_first_seq, CV_RGB ( 0, 0, maxValue ), CV_RGB ( 0, 0, maxValue ), 10, -1 );
       

        Mat bgImg = cvarrToMat ( ppp,false );

        cvReleaseMemStorage ( &m_storage );
        return bgImg;
}

//background subtractor
void CWaterFill::GMM2 ( Mat InputImg, double learnRate )
{

        m_bg_model->apply ( InputImg, m_fgmask,learnRate );


}


Mat CWaterFill::WaterDrop ( Mat InputImg, int num_of_drop )
{
        typedef ushort imgType;
        imgType maxValue = 65535;

        imgType* m_a = NULL;
        imgType* m_b = NULL;
        imgType* m_c = NULL;
        int LocX;
        int LocY;

        int minP = maxValue;
        int picWidth = InputImg.cols;
        int picHeight = InputImg.rows;

        Mat outputImg ( picHeight,picWidth, InputImg.type() );
        memset ( outputImg.data, 0, picWidth * picHeight*sizeof ( imgType ) );
        //m_c = (imgType*)outputImg.data;

        uchar*  Label_P = new uchar[picWidth * picHeight];
        memset ( Label_P, 0, picWidth * picHeight );
        uchar* m_d = NULL;

        Mat inputcopy ( picHeight,picWidth, InputImg.type() );
        InputImg.copyTo ( inputcopy );
        m_a = ( imgType* ) inputcopy.data;
        m_b = ( imgType* ) inputcopy.data;

        int index = 0;

        // make our background the maxvalue
        for ( int i=0; i<picWidth*picHeight; ++i,++m_a ) {
                if ( *m_a < 2 ) {
                        *m_a = maxValue;
                }
        }

        m_a = ( imgType* ) inputcopy.data;

        int waterWay = 0;
        bool isNext = false;
//for(int k = 0 ; k <3;k++)
//{
        
        /* m_d is to make sure we dont drop in the same spot more then once.
            m_c is the matrix we change, we intensify locations where the water fall.
            3x3x4 7500 2500 raindrop*/
        for ( int j=0; j<picHeight; j+=3,m_a+=picWidth*3 ) {
                for ( int i=0; i<picWidth; i+=2) {
                        if ( m_a[i] != maxValue ) {
                                LocX = i;
                                LocY = j;

                                m_b = ( imgType* ) inputcopy.data;
                                m_b += picWidth*LocY + LocX;
                                index = 0;
                                index += picWidth*LocY + LocX;
                                m_c = ( imgType* ) outputImg.data;
                                m_c += picWidth*LocY + LocX;
                                m_d = Label_P;
                                m_d += picWidth*LocY + LocX;

                                for ( int k=0; k<25; ++k ) {
                                        if ( LocX>picWidth-1 || LocX<2 || LocY>picHeight-1 || LocY<2 ) {
                                                break;
                                        }

                                        memset ( Label_P, 0, picWidth * picHeight ); // matrixd to 0
                                        isNext = false;


                                        minP = *m_b; // is originally at locx locy and looks for regions on the image smaller than current val
                                        waterWay = 9;

                                        if ( minP >= * ( m_b-picWidth-1 ) ) {
                                                waterWay = 1;
                                                minP =  * ( m_b-picWidth-1 );
                                        }
                                        if ( minP >= * ( m_b-picWidth ) ) {
                                                waterWay = 2;
                                                minP = * ( m_b-picWidth );
                                        }
                                        if ( minP >= * ( m_b-picWidth+1 ) ) {
                                                waterWay = 3;
                                                minP = * ( m_b-picWidth+1 );
                                        }
                                        if ( minP >= * ( m_b+1 ) ) {
                                                waterWay = 4;
                                                minP = * ( m_b+1 );
                                        }
                                        if ( minP >= * ( m_b+picWidth+1 ) ) {
                                                waterWay = 5;
                                                minP = * ( m_b+picWidth+1 );
                                        }
                                        if ( minP >= * ( m_b+picWidth ) ) {
                                                waterWay = 6;
                                                minP = * ( m_b+picWidth );
                                        }
                                        if ( minP >= * ( m_b+picWidth-1 ) ) {
                                                waterWay = 7;
                                                minP = * ( m_b+picWidth-1 );
                                        }
                                        if ( minP >= * ( m_b-1 ) ) {
                                                waterWay = 8;
                                                minP =  * ( m_b-1 );
                                        }

                                        while ( !isNext ) {
                                                switch ( waterWay ) {
                                                case 1:
                                                        LocX -= 1;
                                                        LocY -= 1;

                                                        if ( LocX<2 || LocY <2 ) {
                                                                isNext = true;
                                                                break;
                                                        }

                                                        m_b = m_b-picWidth-1;
                                                        index = index-picWidth-1;
                                                        m_c = m_c-picWidth-1;
                                                        m_d = m_d-picWidth-1;
                                                        *m_d = 1;

                                                        minP = *m_b;
                                                        waterWay = 9;

                                                        if ( minP >= * ( m_b+picWidth-1 ) && ! ( * ( m_d+picWidth-1 ) ) ) {
                                                                waterWay = 7;
                                                                minP = * ( m_b+picWidth-1 );
                                                        }
                                                        if ( minP >= * ( m_b-picWidth+1 ) && ! ( * ( m_d-picWidth+1 ) ) ) {
                                                                waterWay = 3;
                                                                minP = * ( m_b-picWidth+1 );
                                                        }
                                                        if ( minP >= * ( m_b-1 ) && ! ( * ( m_d-1 ) ) ) {
                                                                waterWay = 8;
                                                                minP = * ( m_b-1 );
                                                        }
                                                        if ( minP >= * ( m_b-picWidth ) && ! ( * ( m_d-picWidth ) ) ) {
                                                                waterWay = 2;
                                                                minP = * ( m_b-picWidth );
                                                        }
                                                        if ( minP >= * ( m_b-picWidth-1 ) && ! ( * ( m_d-picWidth-1 ) ) ) {
                                                                minP = * ( m_b-picWidth-1 );
                                                                waterWay = 1;
                                                        }
                                                        break;
                                                case 2:
                                                        LocY -= 1;
                                                        if ( LocY < 2 ) {
                                                                isNext = true;
                                                                break;
                                                        }
                                                        m_b -= picWidth;
                                                        index -= picWidth;
                                                        m_c -= picWidth;
                                                        m_d -= picWidth;
                                                        *m_d = 1;

                                                        minP = *m_b;
                                                        waterWay = 9;

                                                        if ( minP >= * ( m_b-picWidth-1 ) && ! ( * ( m_d-picWidth-1 ) ) ) {
                                                                waterWay = 1;
                                                                minP = * ( m_b-picWidth-1 );
                                                        }
                                                        if ( minP >= * ( m_b-picWidth+1 ) && ! ( * ( m_d-picWidth+1 ) ) ) {
                                                                waterWay = 3;
                                                                minP = * ( m_b-picWidth+1 );
                                                        }
                                                        if ( minP >= * ( m_b-picWidth ) && ! ( * ( m_d-picWidth ) ) ) {
                                                                minP = * ( m_b-picWidth );
                                                                waterWay = 2;
                                                        }
                                                        break;
                                                case 3:
                                                        LocY -= 1;
                                                        LocX += 1;
                                                        if ( LocY<2 || LocX>picWidth-1 ) {
                                                                isNext = true;
                                                                break;
                                                        }
                                                        m_b = m_b - picWidth + 1;
                                                        index = index -picWidth+1;
                                                        m_c = m_c-picWidth+1;
                                                        m_d = m_d-picWidth+1;
                                                        *m_d = 1;


                                                        minP = *m_b;
                                                        waterWay = 9;

                                                        if ( minP >= * ( m_b-picWidth-1 ) && ! ( * ( m_d-picWidth-1 ) ) ) {
                                                                waterWay = 1;
                                                                minP = * ( m_b-picWidth-1 );
                                                        }
                                                        if ( minP >= * ( m_b+picWidth+1 ) && ! ( * ( m_d+picWidth+1 ) ) ) {
                                                                waterWay = 5;
                                                                minP = * ( m_b+picWidth+1 );
                                                        }
                                                        if ( minP >= * ( m_b-picWidth ) && ! ( * ( m_d-picWidth ) ) ) {
                                                                waterWay = 2;
                                                                minP = * ( m_b-picWidth );
                                                        }
                                                        if ( minP >= * ( m_b+1 ) && ! ( * ( m_d+1 ) ) ) {
                                                                waterWay = 4;
                                                                minP = * ( m_b+1 );
                                                        }
                                                        if ( minP >= * ( m_b-picWidth+1 ) && ! ( * ( m_d-picWidth+1 ) ) ) {
                                                                minP = * ( m_b-picWidth+1 );
                                                                waterWay = 3;
                                                        }
                                                        break;
                                                case 4:
                                                        LocX += 1;
                                                        if ( LocX >picWidth-1 ) {
                                                                isNext = true;
                                                                break;
                                                        }
                                                        m_b += 1;
                                                        index +=1;
                                                        m_c += 1;
                                                        m_d += 1;
                                                        *m_d = 1;

                                                        minP = *m_b;
                                                        waterWay = 9;

                                                        if ( minP >= * ( m_b-picWidth+1 ) && ! ( * ( m_d-picWidth ) ) ) {
                                                                waterWay = 3;
                                                                minP = * ( m_b-picWidth+1 );
                                                        }
                                                        if ( minP >= * ( m_b+picWidth+1 ) && ! ( * ( m_d+picWidth+1 ) ) ) {
                                                                waterWay = 5;
                                                                minP = * ( m_b+picWidth+1 );
                                                        }
                                                        if ( minP >= * ( m_b+1 ) && ! ( * ( m_d+1 ) ) ) {
                                                                minP = * ( m_b + 1 );
                                                                waterWay = 4;
                                                        }
                                                        break;
                                                case 5:
                                                        LocX += 1;
                                                        LocY += 1;
                                                        if ( LocX >picWidth-1 || LocY > picHeight-1 ) {
                                                                isNext = true;
                                                                break;
                                                        }
                                                        m_b += picWidth+1;
                                                        index += picWidth+1;
                                                        m_c += picWidth+1;
                                                        m_d += picWidth+1;
                                                        *m_d = 1;
                                                        minP = *m_b;
                                                        waterWay = 9;

                                                        if ( minP >= * ( m_b-picWidth+1 ) && ! ( * ( m_d-picWidth+1 ) ) ) {
                                                                waterWay = 3;
                                                                minP = * ( m_b-picWidth+1 );
                                                        }
                                                        if ( minP >= * ( m_b+picWidth-1 ) && ! ( * ( m_d+picWidth-1 ) ) ) {
                                                                waterWay = 7;
                                                                minP = * ( m_b+picWidth-1 );
                                                        }
                                                        if ( minP >= * ( m_b+1 ) && ! ( * ( m_d+1 ) ) ) {
                                                                waterWay = 4;
                                                                minP =  * ( m_b+1 );
                                                        }
                                                        if ( minP >= * ( m_b+picWidth ) && ! ( * ( m_d+picWidth ) ) ) {
                                                                waterWay = 6;
                                                                minP = * ( m_b+picWidth );
                                                        }
                                                        if ( minP >= * ( m_b+picWidth+1 ) && ! ( * ( m_d+picWidth+1 ) ) ) {
                                                                minP = * ( m_b+picWidth+1 );
                                                                waterWay = 5;
                                                        }
                                                        break;
                                                case 6:
                                                        LocY += 1;
                                                        if ( LocY > picHeight-1 ) {
                                                                isNext = true;
                                                                break;
                                                        }
                                                        m_b += picWidth;
                                                        index += picWidth;
                                                        m_c += picWidth;
                                                        m_d += picWidth;
                                                        *m_d = 1;

                                                        minP = *m_b;
                                                        waterWay = 9;

                                                        if ( minP >= * ( m_b+picWidth+1 ) && ! ( * ( m_d+picWidth+1 ) ) ) {
                                                                waterWay = 5;
                                                                minP = * ( m_b+picWidth+1 );
                                                        }
                                                        if ( minP >= * ( m_b+picWidth-1 ) && ! ( * ( m_d+picWidth-1 ) ) ) {
                                                                waterWay = 7;
                                                                minP = * ( m_b+picWidth-1 );
                                                        }
                                                        if ( minP >= * ( m_b+picWidth ) && ! ( * ( m_d+picWidth ) ) ) {
                                                                minP = * ( m_b + picWidth );
                                                                waterWay = 6;
                                                        }
                                                        break;
                                                case 7:
                                                        LocY += 1;
                                                        LocX -= 1;
                                                        if ( LocX <2 || LocY >picHeight-1 ) {
                                                                isNext = true;
                                                                break;
                                                        }
                                                        m_b += picWidth-1;
                                                        index += picWidth-1;
                                                        m_c += picWidth-1;
                                                        m_d += picWidth-1;
                                                        *m_d = 1;
                                                        minP = *m_b;
                                                        waterWay = 9;

                                                        if ( minP > * ( m_b+picWidth+1 ) && ! ( * ( m_d+picWidth+1 ) ) ) {
                                                                waterWay = 5;
                                                                minP = * ( m_b+picWidth+1 );
                                                        }
                                                        if ( minP > * ( m_b-picWidth-1 ) && ! ( * ( m_d-picWidth-1 ) ) ) {
                                                                waterWay = 1;
                                                                minP = * ( m_b-picWidth-1 );
                                                        }
                                                        if ( minP > * ( m_b+picWidth ) && ! ( * ( m_d+picWidth ) ) ) {
                                                                waterWay = 6;
                                                                minP = * ( m_b+picWidth );
                                                        }
                                                        if ( minP > * ( m_b-1 ) && ! ( * ( m_d-1 ) ) ) {
                                                                waterWay = 8;
                                                                minP = * ( m_b-1 );
                                                        }
                                                        if ( minP >= * ( m_b+picWidth-1 ) && ! ( * ( m_d+picWidth-1 ) ) ) {
                                                                minP = * ( m_b+picWidth-1 );
                                                                waterWay = 7;
                                                        }
                                                        break;
                                                case 8:
                                                        LocX -= 1;
                                                        if ( LocX < 2 ) {
                                                                isNext = true;
                                                                break;
                                                        }

                                                        m_b -= 1;// actual input value
                                                        index -=1;
                                                        m_c -= 1;// return matrix
                                                        m_d -= 1;// matrix to ensure we dont have a rain drop follow the same path
                                                        *m_d = 1;

                                                        minP = *m_b;
                                                        waterWay = 9;

                                                        if ( minP >= * ( m_b+picWidth-1 ) && ! ( * ( m_d+picWidth-1 ) ) ) {
                                                                waterWay = 7;
                                                                minP = * ( m_b+picWidth-1 );
                                                        }
                                                        if ( minP >= * ( m_b-picWidth-1 ) && ! ( * ( m_d-picWidth-1 ) ) ) {
                                                                waterWay = 1;
                                                                minP = * ( m_b-picWidth-1 );
                                                        }
                                                        if ( minP >= * ( m_b-1 ) && ! ( * ( m_d-1 ) ) ) {
                                                                minP = * ( m_b-1 );
                                                                waterWay = 8;
                                                        }
                                                        break;
                                                case 9:
                                                        *m_b += num_of_drop;// increases the value of our input copy. Keeps it from traveling it again?
                                                        *m_c += 2500; // output image
                                                        isNext = true;
                                                        break;
                                                }
                                        }


                                }
                        }
                }
        }
        
//}

        delete []Label_P;

        return outputImg;

}

Mat CWaterFill::WaterFilter ( Mat inputImg, int mimNum ) // make everything black other then the minNum
{
        typedef ushort imgType;
        imgType maxValue = 65535;

        Mat outputImg ( inputImg.rows, inputImg.cols, inputImg.type() );

        imgType* pSrc = ( imgType* ) inputImg.data;
        imgType* pDest = ( imgType* ) outputImg.data;

        int piexlNum = inputImg.rows * inputImg.cols;

        for ( int i=0; i<piexlNum; ++i,++pSrc,++pDest ) {
                if ( *pSrc <= mimNum ) {
                        *pDest =0;
                } else {
                        *pDest = maxValue;
                }
        }
        return outputImg;
}


Mat CWaterFill::HalfSizeImg ( Mat SrcImg )
{

        int hightImg = SrcImg.rows /2;
        int widthImg = SrcImg.cols /2;
        Mat DestImg ( hightImg, widthImg, SrcImg.type() );
        ushort* pSrc = ( ushort* ) SrcImg.data;
        ushort* pDest = ( ushort* ) DestImg.data;

        for ( int i=0; i<hightImg; ++i, pSrc+=widthImg*2 ) {
                for ( int j=0; j<widthImg; ++j, pSrc +=2, ++pDest ) {
                        *pDest = *pSrc;
                }
        }
        return DestImg;
}

Mat CWaterFill::Sixth2Eight ( Mat SrcImg, int ratio ) // convert 16 bit image to 8 bit image
{
        int hightImg = SrcImg.rows;
        int widthImg = SrcImg.cols;
        Mat DestImg ( hightImg, widthImg, CV_8UC1 );
        ushort* pSrc = ( ushort* ) SrcImg.data;
        uchar* pDest = DestImg.data;
        int piexlNum = hightImg * widthImg;

        /*
        If the scaling factor is greater than 1./256 (note that 65536 / 256 = 256, in the above example we set the factor to 1./40),
        pixels with larger values will all be mapped to 255 (no difference any more for larger values).

        https://en.wikipedia.org/wiki/Scale_factor_(computer_science)
        */

        double temp = 0;
        if ( ratio > 1 ) {
                for ( int i=0; i<piexlNum; ++i, ++pSrc, ++pDest ) {
                        if ( *pSrc!=0 ) {
                                temp = *pSrc/ratio; //ratio (40) is the scaling factor to map 16bit to 8bit
                                if ( temp > 255 ) {
                                        *pDest = 255;
                                } else if ( temp < 0 ) {
                                        *pDest = 0;
                                } else {
                                        *pDest = ( uchar ) temp;
                                }
                        } else {
                                *pDest = 0;
                        }
                }
        }
        //
        else if ( ratio == 0 ) {
                for ( int i=0; i<piexlNum; ++i, ++pSrc, ++pDest ) {
                        if ( *pSrc == 65535 ) { // ushort max value to rgb max value.
                                *pDest = 255;
                        } else {
                                *pDest = 0;
                        }
                }
        }

        return DestImg;
}

// whenever the value of the matrix being passed in is blank the destination value gets updated to 1.
void CWaterFill::AndOpera ( Mat SrcImg, Mat* DestImg )
{
        typedef ushort imgType;
        uchar* pSrc = SrcImg.data;
        imgType* pDest = ( imgType* ) DestImg->data;
        int pixNum = SrcImg.cols * SrcImg.rows;
        //showImage(SrcImg);
        for ( int i=0; i<pixNum; ++i, ++pSrc, ++pDest ) {
                if ( *pSrc == 0 ) {
                        *pDest = 1;
                }

        }
}

void CWaterFill::showImage ( Mat map,string name )
{
        double min;
        double max;
        cv::minMaxIdx ( map, &min, &max );
        cv::Mat adjMap;
        cv::convertScaleAbs ( map, adjMap, 255 / max );
        cv::imshow ( name, adjMap );
}

void CWaterFill::Water ( int nFrame, int threshold, vector<RECT>* detectBox, int stype )
{

 
       const int minSize = 600;//900   This is the minsize countour box size/ entity size
       const int minHeadSize = 350;//filter for the raindrop image



        minImg = HalfSizeImg ( depthMap );
	
	// bucketing:
	// the largest value we tend to see for distance is 2500mm.  40 * 256 = 10,240 
	// so we can map a value between 0-10,240 correctly. 8 bit imgs are 0-256 values.
	// We also cancel out noise (originally a problem) by bucketing/ mapping 40 values to one.
	// (0-39)/40 = 0 (not including remainder)..
        img_8bit = Sixth2Eight ( minImg, 40 ); //40
	
	imshow ( "Bucketed Img",img_8bit );
  
	 // background subtractor
        GMM2 ( img_8bit, 0.0001 );
	
	imshow ( "Foreground ", m_fgmask);


        BgImg = ContourFilter ( m_fgmask, minSize ); //take a look at old code
 
	
        // whenever the value of the matrix being passed in is blank the destination value gets updated to 1.
        // make all values from the background one and leave the foreground alone
        AndOpera ( BgImg, &minImg );
       

        // merge the blank points on the head with the average value of the row.
        // The last 8 columns are blank might consider changing this function to exlcude them
        MergeBlack ( &minImg );


	//imshow ( "Subtracted + Merged Img ", minImg);
	
        img_16bit = WaterDrop ( minImg, threshold );

	

        waterImg = WaterFilter ( img_16bit, 9500 ); //8000 basically make any pixel less then or equal to this black. "what color grey matters"
      
        BgImg = Sixth2Eight ( waterImg, 0 );
	
	imshow ( "Water Drop + Height Filtered Img",BgImg);
	
        waterImg_8bit = ContourFilter ( BgImg, minHeadSize );
        ///showImage(waterImg);
        GetHead ( waterImg_8bit, detectBox );
	
	imshow ( "Outcome",img_8bit);
        //showImage(img_8bit);
        


}


void CWaterFill::TrainBgSub ( Mat train )
{
        GMM2( train, .1 );
}

