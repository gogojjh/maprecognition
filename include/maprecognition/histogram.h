#ifndef HISTOGRAM_H
#define HISTOGRAM_H

#include "maprecognition/base.h"

class Histogram
{
private:
    int histSize[3];
    float hranges[2];
    const float *ranges[3];
    int channels[3];
public:
    vector<Mat>v;
    Histogram()
    {
        histSize[0] = histSize[1] = histSize[2] = 256;
        hranges[0] = 0.0;
        hranges[1] = 255.0;
        //每个通道像素的范围
        ranges[0] = hranges;
        ranges[1] = hranges;
        ranges[2] = hranges;
        channels[0] = 0;
        channels[1] = 1;
        channels[2] = 2;
    }
    MatND getHueHistogram(const Mat &image, int minSaturation = 0)
    {
        MatND hist; //直方图类型
        Mat hsv;
        cvtColor(image, hsv, CV_BGR2HSV);
        Mat mask;
        split(hsv, v);

        histSize[0] = 180; histSize[1] = 180; histSize[2] = 180;
        hranges[0] = 0.0;
        hranges[1] = 180.0;
        channels[0] = 0;
        //计算直方图
        calcHist(&hsv, 1, channels, mask, hist, 1, histSize, ranges);
        return hist;
    }
    //遍历直方图
    void getHistogramStat(const MatND &colorhist)
    {
        float k = 0;
        for (int i=0; i<180; i++)
        {
            k += colorhist.at<float>(i);
            cout << "Value" << i << "=" << colorhist.at<float>(i) << "  " << k << endl;
        }
    }
    Mat getHistogramImage(const MatND &colorhist)
    {
        double maxVal = 0;
        double minVal = 0;
        //取矩阵的最大最小值
        minMaxLoc(colorhist, &minVal, &maxVal, 0, 0);
        //定义显示直方图的图像, height,width
        Mat histImg(histSize[0], histSize[0]+100, CV_8U, Scalar(255));
        int hpt = static_cast<int>(0.9*histSize[0]);
        for (int h = 0; h<histSize[0]; h++)
        {
            float binVal = colorhist.at<float>(h);
            int intensity = static_cast<int>(binVal*hpt/maxVal);
            line(histImg, Point(h+50, histSize[0]), Point(h+50, histSize[0]-intensity), Scalar::all(0));
        }
        return histImg;
    }
};


#endif // HISTOGRAM_H
