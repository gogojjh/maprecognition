#ifndef GRIDDETECT_H
#define GRIDDETECT_H

#include "maprecognition/base.h"

void griddetect(const Mat src, Point2f &goal,
                float &distance,
                bool &find_object,
                Mat &result,
                Mat &mask);

#endif // GRIDDETECT_H
