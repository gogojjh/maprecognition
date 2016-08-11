#ifndef TRANSFORMER_H
#define TRANSFORMER_H

#include "maprecognition/base.h"
#include "maprecognition/griddetect.h"

#define THRESH_RED 0.4
#define COLORL_GROUND_RED 20
#define COLORH_GROUND_RED 80

//Canny边缘检测两个阈值（0~255）
#define CANNY_THL 50
#define CANNY_THH 255

//红色区域FLAG
#define FLAG_TRAVELING 0
#define FLAG_ARRIVE 1
#define FLAG_LOCATE 2
#define FLAG_TARGET 3
#define FLAG_CALIBRATE 4
#define FLAG_RETURN 5

//获取成员变量
#define MEMBER_IMG 0
#define MEMBER_CANNY 1
#define MEMBER_IMG_HSV 0
#define MEMBER_OPEN_HSV 1
#define MEMBER_CLOSE_HSV 2

class Transformer{
public:

    Transformer();   /* 构造函数，输入图片src，初始化得到图像size，图像中心坐标 */
    Point gCenter();
    Mat gImage(int key);
    void gTransformer(Mat &dst, int channel, int key);
    void sTransformer(const Mat image);

    /* areaTwo */
    void detectTwo(Point2f &goal, float &distance, bool &find_object);

    /* apriltag */
    void detectApriltag(Point2f &goal, float &distance, bool &find_object);

    /* areaThree*/
    void detectThree(Point2f &goal, float &distance, bool &find_object);

    /* areaFour */
    void detectFour(Point2f &goal, float &distance, bool &find_object);

    /* areaRed */
    void detectRed(Point2f &goal, float &distance, bool &find_object);

    static int number;
    static int gridfourdetect_threshold;

private:
    Point center;
    Mat img_Origin, img_Canny, img_Result;
    Mat maskgrid;
    vector<Mat> img_HSV;   /*原图HSV三通道图像img_HSV */
    vector<Mat> open_HSV; /* 开运算HSV三通道图像open_HSV */
    vector<Mat> close_HSV; /* 闭运算HSV三通道图像close_HSV */

    int open_size;
    int open_operator;
    int close_size;
    int close_operator;
};

#endif // TRANSFORMER_H
