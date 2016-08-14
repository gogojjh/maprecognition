#include "maprecognition/gridfourdetect.h"
#include "maprecognition/transformer.h"

void gridfourdetect(const Mat src,
                Point2f &minG,
                float &minD,
                bool &minF,
                Mat &result,
                Mat &mask)
{
    RotatedRect box = squareDet(src, result, minF);
    Point2f vertex[4];
    box.points(vertex);
    for(int i=0;i<4;i++)
    {
        line(result, vertex[i], vertex[(i+1)%4], Scalar(0,0,255), 5);    //draw the rectangle of the box
    }
    Point2f goal((vertex[1].x+vertex[3].x)/2 , (vertex[1].y+vertex[3].y)/2);
    Point2f center(src.cols/2, src.rows/2);
    circle(result, goal, 3, Scalar(0,0,255), 2, 8, 0);

    minD = sqrt(pow(goal.x-center.x, 2.0) + pow(center.y-goal.y, 2.0));
    getPosition(center, goal, minG);
    //rectangle(img_src,box,Scalar(0,255,0),5);
    //Mat imgROI = test.dollDetect(img_src,box);          //detect the doll
}

// detect the box
RotatedRect squareDet(const Mat src, Mat &result, bool &minF)
{
    //split the channel
    Mat imgSingle = colorConversion(src, BGR_R);

    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    float maxContourArea = 0;
    int maxContourArea_index = 0;

    Mat element = getStructuringElement(MORPH_RECT,Size(27,27));
    morphologyEx(imgSingle, imgSingle,MORPH_DILATE,element);
    imshow("2_img_dilate",imgSingle);

    medianBlur(imgSingle,imgSingle,5);
    element = getStructuringElement(MORPH_RECT,Size(22,22));
    morphologyEx(imgSingle,imgSingle,MORPH_ERODE,element);
    imshow("3_img_close",imgSingle);

    //Mat element = getStructuringElement(MORPH_RECT,Size(5,5));
    //morphologyEx(imgSingle, imgSingle, MORPH_CLOSE, element);
    //imshow("img_single_close",imgSingle);

    threshold(imgSingle, imgSingle, Transformer::gridfourdetect_threshold , 255, THRESH_BINARY_INV );
    imshow("img_single_threshold",imgSingle);

    findContours(imgSingle, contours,hierarchy ,CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    drawContours(imgSingle, contours, -1, Scalar(255), 2);
    imshow("contours",imgSingle);

    // Find the max area of contours
    for (int i=0;i<contours.size();i++)
    {
        int contourSize = contourArea(contours[i]);
//        cout<<"area"<<contourSize<<endl;
        if (contourSize < src.cols * src.rows * 0.066) continue;
        if(contourSize > maxContourArea)
        {
            maxContourArea = contourSize;
            maxContourArea_index = i;
        }
    }
//    cout << "Max area: " << maxContourArea << endl;

    RotatedRect box;
    if(maxContourArea != 0)
    {
         box = minAreaRect(contours[maxContourArea_index]);
         minF = true;
    }
    return box;
}

