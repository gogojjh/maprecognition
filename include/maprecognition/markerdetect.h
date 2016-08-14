#ifndef MARKERDETECT_H
#define MARKERDETECT_H

#include "maprecognition/base.h"

void markerdetect(const Mat src,
                Point2f &minG,
                float &minD,
                bool &minF,
                Mat &result,
                Mat &mask);

void decectMarkerContour(const Mat src, Point2f &minG, float &minD, bool &minF, Mat &result);

#endif // MARKERDETECT_H
