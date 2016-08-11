#include "maprecognition/griddetect.h"
#include "maprecognition/findrect.h"

void griddetect(const Mat src,
                Point2f &goal,
                float &distance,
                bool &find_object,
                Mat &result,
                Mat &mask)
{
    Mat imgResize;
    src.copyTo(imgResize);

    /*rgb2hsv */
    Mat imgHSV = Mat::zeros(imgResize.rows,imgResize.cols,imgResize.type());
    vector<Mat> channels;
    cvtColor( imgResize, imgHSV, CV_BGR2HSV );

    split(imgHSV,channels);
    Mat imgH = channels.at(0);
    Mat imgS = channels.at(1);
    Mat imgV = channels.at(2);

    /* extract the lighting area */
    Mat imgVthreshold;
    threshold(imgV,imgVthreshold,205,255,cv::THRESH_BINARY);

    Mat imgHMasked = Mat::zeros(imgResize.rows,imgResize.cols,imgResize.type());
    imgH.copyTo(imgHMasked, imgVthreshold);
    Mat imgRed = (((imgHMasked > 165)&(imgHMasked < 180))|((imgHMasked > 0)&(imgHMasked < 10)));
    Mat imgBlue = ((imgHMasked > 90) & (imgHMasked < 120));
    //Mat imgYellow = ((imgHMasked > 90) & (imgHMasked < 120));

    Point2f minG(0,0);
    bool minF = false;
    float minD = imgResize.cols * imgResize.rows;

    //findrect(imgResize, imgRed, minG, minD, minF, result, mask);
    findrect(imgResize, imgBlue, minG, minD, minF, result, mask);
    if (minG.x != 0)
        circle(result, Size((minG.x+src.cols/2), ((-1)*minG.y+src.rows/2)), 2,Scalar(0,0,255),2);

    goal.x = minG.x; goal.y = minG.y;
    distance = minD;
    find_object = minF;
    //imshow("mask", mask);
    //findrect(imgResize, imgYellow,  angle, goal, distance, result, find_object);
}
