#ifndef GRIDFOURDETECT_H
#define GRIDFOURDETECT_H

#include "maprecognition/base.h"

void gridfourdetect(const Mat src,
                Point2f &minG,
                float &minD,
                bool &minF,
                Mat &result,
                Mat &mask);
RotatedRect squareDet(const Mat src, Mat &result, bool &minF);
void dollDetect(Mat img, RotatedRect box, Point2f &goal);
int orbMatching(Mat img_mode,Mat img);
//int gridDetection_3( Mat img_src,float& angle ); // three area

#endif // GRIDFOURDETECT_H
