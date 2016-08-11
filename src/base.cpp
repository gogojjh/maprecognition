#include "maprecognition/base.h"

Mat colorConversion(Mat img, HsvType imgtype)
{
     //GaussianBlur(img, img, Size(5,5),0,0);
     if(imgtype==HSV||imgtype==HSV_H||imgtype==HSV_S||imgtype==HSV_V)
    {
        Mat img_hsv, img_h, img_s, img_v;
        cvtColor(img, img_hsv, CV_BGR2HSV);
        vector<Mat> hsv_channels;
        split(img_hsv, hsv_channels);
        img_h = hsv_channels[0];
        img_s = hsv_channels[1];
        img_v = hsv_channels[2];
        switch (imgtype)
        {
            case 0: return img_hsv;
            case 1: return img_h;
            case 2: return img_s;
            case 3: return img_v;
            default: break;
        }
    }
    if(imgtype==BGR||imgtype==BGR_B||imgtype==BGR_G||imgtype==BGR_R)
    {
        Mat b_img, g_img, r_img;
        vector<Mat> bgr_channels;
        split(img,bgr_channels);
        b_img=bgr_channels[0];
        g_img=bgr_channels[1];
        r_img=bgr_channels[2];
        switch(imgtype)
        {
            case 4: return img;
            case 5: return b_img;imshow("r",r_img);
            case 6: return g_img;
            case 7: return r_img;
        }
    }
    return img;
}

// get the position of destination
void getPosition(const Point2f p1, const Point2f p2, Point2f &ans)
{
    ans.y=p2.x - p1.x;
    ans.x=(-1) * (p2.y - p1.y);
    //cout<< ans <<endl;
}

