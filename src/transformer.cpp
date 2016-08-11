// different operation
#include "maprecognition/transformer.h"
#include "maprecognition/griddetect.h"
#include "maprecognition/leddetect.h"
#include "maprecognition/gridfourdetect.h"
#include "maprecognition/markerdetect.h"

Transformer::Transformer()
{
    close_size=17;
    close_operator=0;
    open_size=6;
    open_operator=0;
    //number=0;
}

void Transformer::sTransformer(const Mat image)
{
    resize(image,img_Origin,Size(image.cols/IMAGE_SIZE_COL, image.rows/IMAGE_SIZE_ROW));
    img_Origin.copyTo(img_Result);
    center.x=img_Origin.cols/2;
    center.y=img_Origin.rows/2;

    split(img_Origin,img_HSV);

    Mat img_Open;
    Mat element = getStructuringElement( 0, Size( 2*open_size +3, 2*open_size+10 ), Point(open_size, open_size+5 ));
    morphologyEx( img_Origin, img_Open,MORPH_OPEN, element );
    split(img_Open,open_HSV);

    Mat img_Close;
    element = getStructuringElement( 0, Size( 2*close_size-5, 2*close_size-5 ), Point(close_size, close_size ) );
    morphologyEx( img_Origin, img_Close,  MORPH_CLOSE, element );
    split(img_Close,close_HSV);

    Canny(img_Origin, img_Canny, CANNY_THL, CANNY_THH);

    if (number == 1) maskgrid = Mat(img_Origin.size(), 1, Scalar::all(0));
}

Point Transformer::gCenter()
{
    Point p(center.x,center.y);
    return p;
}

Mat Transformer::gImage(int key)
{
    switch(key)
    {
        case 0: return img_Origin; break;
        case 1: return img_Canny; break;
        case 2: return img_Result; break;
    default: break;
    }
}

void Transformer::gTransformer(Mat &dst, int channel, int key)
{
    switch(key)
    {
        case 1: open_HSV[channel].copyTo(dst); break;
        case 2: close_HSV[channel].copyTo(dst); break;
        default: img_HSV[channel].copyTo(dst); break;
    }
}

void Transformer::detectApriltag(Point2f &goal, float &distance, bool &find_object)
{
    markerdetect(img_Origin, goal, distance, find_object, img_Result, maskgrid);
}

void Transformer::detectTwo(Point2f &goal, float &distance, bool &find_object)
{
    griddetect(img_Origin, goal, distance, find_object, img_Result, maskgrid);
    if (find_object == false) maskgrid = Mat(img_Origin.size(), 1, Scalar::all(0));
}

void Transformer::detectThree(Point2f &goal, float &distance, bool &find_object)
{

}

void Transformer::detectFour(Point2f &goal, float &distance, bool &find_object)
{
    //bian kuang
    gridfourdetect(img_Origin, goal, distance, find_object, img_Result, maskgrid);

    //leddetect(img_Origin, goal, distance, find_object, img_Result, maskgrid);
    //if (find_object == false) maskgrid = Mat(img_Origin.size(), 1, Scalar::all(0));
}

void Transformer::detectRed(Point2f &goal, float &distance, bool &find_object)
{
    /*
    int scale1=10;
    int scale2=5;
    int select;
    int x1,y1,x2,y2;

    x1=img.cols/scale1;
    x2=img.cols-x1;
    y1=img.rows/scale2;
    y2=img.rows-y1;
    Point p1,p2;

    select=oriented+key;
    if(select>4)
    {
        select=select-4;
    }
    switch(select){
    case 1:p1.x=x2;p1.y=y1;	p2.x=img.cols;p2.y=y2;	break;
    case 2:p1.x=x1;p1.y=y2;	p2.x=x2;p2.y=img.rows;	break;
    case 3:p1.x=0;p1.y=y1;	p2.x=x1;p2.y=y2;	break;
    case 4:p1.x=x1;p1.y=0;	p2.x=x2;p2.y=y1;	break;
    default:return false;
    }*/

    /*
    Mat temp;
    medianBlur(open_HSV[1], temp, 9);
    //Mat roi=temp(region);
    //imshow("open_HSV",temp);
    //	imshow("roi",roi);
    Mat red_filter_1, red_filter_2;
    threshold(temp, red_filter_1, COLORL_GROUND_RED,255,CV_THRESH_BINARY);
    //	imshow("binary",roi1);
    threshold(temp, red_filter_2, COLORH_GROUND_RED,255,CV_THRESH_BINARY_INV);
    //	imshow("binary2",roi2);
    Mat red_result;
    bitwise_and(red_filter_1, red_filter_2, red_result);
    //	imshow("and",roi);
    Vector<double>area = sum(red_result) / 256/ (red_result.cols * red_result.rows);
    red_result.copyTo(img_Result);
    */

    //imshow("temp",temp);
}
















