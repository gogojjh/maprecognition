#ifndef BASE_H
#define BASE_H

#include <iostream>
#include <string>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#define UP 1
#define DOWN -1
#define LEFT -2
#define RIGHT 2
#define PI 3.14159265358979323846
#define IMAGE_SIZE_COL 2
#define IMAGE_SIZE_ROW 2
#define NEAR_OBJECT 50

#define IMAGE_SIZE_SCALE 5
#define HsvType int
#define HSV   0
#define HSV_H 1
#define HSV_S 2
#define HSV_V 3
#define BGR   4
#define BGR_B 5
#define BGR_G 6
#define BGR_R 7

using namespace std;
using namespace cv;

Mat colorConversion(Mat img, HsvType imgtype);

// get the position of destination
void getPosition(const Point2f p1, const Point2f p2, Point2f &ans); //center, goal, minG

#endif // BASE_H
