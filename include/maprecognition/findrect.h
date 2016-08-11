#ifndef FINDRECT_H
#define FINDRECT_H

#include "maprecognition/base.h"

#define THRESHOLD_RECT 0.4
#define THRESHOLD_CANNYL 10
#define THRESHOLD_CANNYH 180

void dilateROI(const Mat src, Mat &dst, Mat mask);
void findrect(const Mat src, const Mat imgBinary, Point2f &minG, float &minD, bool &minF, Mat &result, Mat &mask);
bool rect_finalcheck(const Mat src,Mat mask);

#endif // FINDRECT_H
