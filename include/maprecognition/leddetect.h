#ifndef LEDDETECT_H
#define LEDDETECT_H

#include "maprecognition/base.h"

void leddetect(const Mat src,
                Point2f &minG,
                float &minD,
                bool &minF,
                Mat &result,
                Mat &mask);
void eraseGround(Mat &src);
void lowExposureGrid(const Mat src, Mat &result, Point2f &goal, float &minD,bool &find_object,Mat &mask);
void lowExposureWAWA(const Mat src,Mat &result,Point2f &goal, float &minD,bool &find_object,bool &WAWA);


#endif // LEDDETECT_H
