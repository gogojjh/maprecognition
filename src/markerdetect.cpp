#include "maprecognition/base.h"
#include "maprecognition/markerdetect.h"
#include <iostream>

void markerdetect(const Mat src,
                Point2f &minG,
                float &minD,
                bool &minF,
                Mat &result,
                Mat &mask)
{
    src.copyTo(result);
    decectMarkerContour(src, minG, minD, minF, result);
}

void decectMarkerContour(const Mat src, Point2f &minG, float &minD, bool &minF, Mat &result)
{
    Mat img_h, img_threshold, morph_img;
    Mat img_canny(src.rows, src.cols,CV_8UC3,Scalar(0,0,0));

    // extract the red
    img_h = colorConversion(src, BGR_R);
    threshold(img_h, img_threshold,50,255,1);

    Mat element = getStructuringElement(MORPH_RECT,Size(3,3));
    morphologyEx(img_threshold,img_threshold,MORPH_ERODE,element);
    //imshow("2", img_threshold);

    //Canny(img_threshold,img_canny,30,90);
    //imshow("2",img_canny);

    vector<vector<Point> > contours;
    vector<Point> approxCurve;
    vector<Vec4i> hierarchy;
    bool longEdge = 1;
    float markerArea = 0;
    bool sign_bigger = false;
    bool sign_small = false;
    Point2f pointsIn[4]={Point2f(0,0)};
    Point2f approxCenter(0,0);
    Point2f goal(0,0);
    Point2f center(src.cols/2, src.rows/2);
    vector<Point2f>corners;

    //vector<Rect> box(contours.size());
    findContours(img_threshold, contours, hierarchy, RETR_TREE, CHAIN_APPROX_NONE);
    //cout << contours.size() << endl;
    Mat imgContour(src.rows, src.cols,CV_8UC1,Scalar(0));;
    approxCurve.resize(contours.size());

    for(unsigned int i=0; i<contours.size();i++)
    {
        float contour_area=contourArea(contours[i]);
        //cout << contour_area << endl;
        //drawContours(imgContour, contours, i, Scalar(255));
        //cout << hierarchy[i] << endl;
        if (contour_area > 3500)
        {
            markerArea = contour_area;
            //cout <<  contour_area << endl;
            // duo bian xing bi jin
            approxPolyDP(contours[i], approxCurve, double(contours[i].size()) * 0.05, true);
            // the input contour is convex or not
            if (isContourConvex(Mat(approxCurve)))
            {
                sign_bigger = true;
                int min_x = INT_MAX;
                int min_index = 0;
                // zuo shang jiao
                for(int j=0; j<4; j++)
                {
                    if(sqrt(approxCurve[j].x*approxCurve[j].x+approxCurve[j].y*approxCurve[j].y) < min_x)
                    {
                        min_x = sqrt(approxCurve[j].x*approxCurve[j].x+approxCurve[j].y*approxCurve[j].y);
                        min_index = j;
                    }
                }
                approxCenter.x = (approxCurve[1].x + approxCurve[3].x) / 2.0;
                approxCenter.y = (approxCurve[1].y + approxCurve[3].y) / 2.0;

                if (sqrt(pow(approxCenter.x - src.cols/2, 2.0) + pow(approxCenter.y - src.rows/2, 2.0)) < minD)
                {
                    getPosition(center, approxCenter, minG);
                    minD = sqrt(pow(approxCenter.x - src.cols/2, 2.0) + pow(approxCenter.y - src.rows/2, 2.0));
                    minF = true;
                    goal = approxCenter;
                }

                // four corners
                if(min_index == 0 )
                {
                    for(int j=0;j<4 ;j++)
                    {
                        pointsIn[j] = approxCurve[j];
                        circle(result,Point(pointsIn[j].x,pointsIn[j].y),3,Scalar(0,0,255));
                    }
                }
                else
                {
                    // xun zhao si ge jiao dui ying de zuo biao
                    int array[] = {1, 2, 3, 0, 1, 2, 3};
                    size_t count=sizeof(array)/sizeof(int);
                    vector<int> tmp(array, array+count);
                    vector<int>::iterator tmp_itr = tmp.begin();
                    int k=0;
                    tmp_itr = find(tmp.begin(),tmp.end(),min_index);

                    for(;min_index!=0 && tmp_itr!=tmp.end() && k!=4;tmp_itr++)
                    {
                        pointsIn[k++] = approxCurve[*tmp_itr];
                        circle(result,Point(pointsIn[k-1].x,pointsIn[k-1].y),3,Scalar(0,0,255));
                    }
                }
                //rectangle(src_frame, boundRect[i], Scalar(0,0,255), 2,8,0);
            }
        }

        //mini rectangle
        /*
        else if(sign_bigger = true && markerArea != 0 && 0.01*markerArea < contour_area && contour_area < 0.03*markerArea)
        {
            sign_small = true;
        }*/

        if(sign_bigger == true)
        {
            //pose = markerPosition(img_h, pointsIn);
            sign_bigger = false;
        }
    }
    //imshow("imgContour", imgContour);
    //waitKey(0);
    circle(result, goal, 3, Scalar(0,0,255), 2);
    /*
    for (int i=0; i<corners.size(); i++)
    {
        cout << corners[i].x << "  " << corners[i].y << endl;
    }*/
}
