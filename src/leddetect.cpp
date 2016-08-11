#include "maprecognition/leddetect.h"
#include "maprecognition/base.h"

void leddetect(const Mat src,
                Point2f &minG,
                float &minD,
                bool &minF,
                Mat &result,
                Mat &mask)
{
/********************************Camera tester*********************************/
//    VideoCapture capture(0);
//    if(capture.isOpened())  cout<<"Oops"<<endl;
//    while(1){
//        capture>>src;
/******************************************************************************/
/********************************Video tester**********************************/
//    VideoCapture capture("/home/sd/document/videocapture/3.avi");
//    if(capture.isOpened())  cout<<"Oops"<<endl;
//    while(1){
//        capture>>src;
/******************************************************************************/
//resize(src,src,Size(src.cols / IMAGE_SIZE_SCALE,src.rows/IMAGE_SIZE_SCALE));
/******************************************************************************/
/*erase Ground*/
//      eraseGround(src);
/******************************************************************************/
/*low exposure mode*/
    bool WAWA;

/******************************************************************************/
    /*Grid*/
    //lowExposureGrid(src,dst,goal,distance,find_object,mask);
/******************************************************************************/

    /*WAWA*/
    lowExposureWAWA(src, result, minG, minD, minF, WAWA);
//    if(find_object){
//        if(WAWA)    cout<<"octopus"<<endl;
//        else        cout<<"hippo"<<endl;
//    }
//    else {
//        if(WAWA)    cout<<"need little more closer"<<endl;
//        else        cout<<"missing WAWA"<<endl;

//    }
/******************************************************************************/
/******************************************************************************/
//        waitKey(100);
//    }
/******************************************************************************/
    waitKey(0);
}

/*
void eraseGround(Mat &src)
{
    Mat img_h, img_threshold, img_canny;
    img_h=colorConversion(src, BGR_R);
    imshow("1_img_h", img_h);
    threshold(img_h, img_threshold,60,255,1);
    imshow("2_img_threshold", img_threshold);
    Mat element = getStructuringElement(MORPH_RECT, Size(3,3));
    morphologyEx(img_threshold, img_threshold, MORPH_ERODE,element);
    imshow("3_img_erode", img_threshold);
    Canny(img_threshold, img_canny,30,90);
    img_canny.copyTo(src);
}
*/
/*
void lowExposureGrid(const Mat src, Mat &result, Point2f &minG, float &minD, bool &minF, Mat &mask)
{
    find_object=false;
    imshow("src",src);
    bool find_boject = false;
    float distance = src.cols * src.rows;
    Point2f goal=Point(0,0);

    Mat temp;
    src.copyTo(temp);
    medianBlur(temp,temp,5);
    Mat element = getStructuringElement(0, Size(15, 15));
    morphologyEx(temp,temp, MORPH_DILATE,  element);

    Mat imgBlue=colorConversion(temp,BGR_B);
    Mat imgRed=colorConversion(temp,BGR_R);
}
*/

void lowExposureWAWA(const Mat src,Mat &result,Point2f &minG, float &minD, bool &minF, bool &WAWA)
{
    /*initialize the flag*/
    /*WAWA:false=hippo;*/
    minF=false;
    WAWA=false;
    result = colorConversion(src, HSV_V);

    /*smooth using medianBlur*/
    medianBlur(result, result,5);

    /*binary*/
    threshold(result,result,50,255,THRESH_BINARY);
    imshow("binary", result);

    /*morphology closing*/
    Mat element = getStructuringElement(MORPH_RECT,Size(15,15));
    morphologyEx(result,result,MORPH_CLOSE,element);
    imshow("CLOSEING", result);

    /*retrieve contours_external*/
    vector<vector<Point> >contours;
    vector<Vec4i> hierarchy;
    findContours(result, contours, hierarchy,CV_RETR_TREE,CHAIN_APPROX_SIMPLE);
    if(contours.size()==0)  return;
    cout<<"size_external"<<contours.size()<<endl;

    /*display source img with every contours*/
    drawContours(src,contours,-1,Scalar(200),3);
    imshow("src",src);
    /*return if there is no WAWA being targeted*/
    Vector<float> area;
    int idx;
    for(int i=0;i<contours.size();i++)
    {
        area.push_back(contourArea(contours[i]));
        if(area[i]==*max_element(area.begin(),area.end())) idx=i;
    }
    float Parea=area[idx]/(src.cols*src.rows);
    cout<<"Parea:"<<Parea<<endl;

    /*retrieve moments and output*/
    Moments mom=moments(contours[idx], false);
    Point2f goal = Point(mom.m10/mom.m00,mom.m01/mom.m00);
    Point2f center = Point(src.cols/2,src.rows/2);
    if(Parea<0.1)
    {
        getPosition(center, goal, minG);
        WAWA=true;
        return;
    }

    /*set ROI with filled contours*/
    int counter=0;
    for(int i=0;i<contours.size();i++)
    {
        if(hierarchy[i][3]==idx)    counter++;
    }
    drawContours(result,contours,-1,Scalar::all(255),2);
    cout<<"contoooooours:"<<counter<<endl;

    /*tell what the WAWA is*/
     minF=true;
     if(counter>0|contours.size()>1)
     {
         WAWA=true;
         return;
     }
}
