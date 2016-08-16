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
    if (minF)
    {
        Point2f vertex[4];
        box.points(vertex);
        for(int i=0;i<4;i++)
        {
            line(result, vertex[i], vertex[(i+1)%4], Scalar(0,0,255), 3);    //draw the rectangle of the box
        }

        Point2f goal((vertex[1].x+vertex[3].x)/2 , (vertex[1].y+vertex[3].y)/2);
        Point2f center(src.cols/2, src.rows/2);
        circle(result, goal, 3, Scalar(0,0,255), 2, 8, 0);
        minD = sqrt(pow(goal.x-center.x, 2.0) + pow(center.y-goal.y, 2.0));
        getPosition(center, goal, minG);
        dollDetect(result, box, goal);          //detect the doll
        cout << "X: " << goal.x << " Y: " << goal.y << endl;
    }
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
    //imshow("2_img_dilate",imgSingle);

    medianBlur(imgSingle,imgSingle,5);
    element = getStructuringElement(MORPH_RECT,Size(22,22));
    morphologyEx(imgSingle,imgSingle,MORPH_ERODE,element);
    //imshow("3_img_close",imgSingle);

    //Mat element = getStructuringElement(MORPH_RECT,Size(5,5));
    //morphologyEx(imgSingle, imgSingle, MORPH_CLOSE, element);
    //imshow("img_single_close",imgSingle);

    threshold(imgSingle, imgSingle, Transformer::gridfourdetect_threshold , 255, THRESH_BINARY_INV );
    //imshow("img_single_threshold",imgSingle);

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
    //waitKey(0);
    return box;
}

void swap(Point2f &a, Point2f &b)
{
    Point2f c;
    c.x = a.x; c.y = a.y;
    a.x = b.x; a.y = b.y;
    b.x = c.x; b.y = c.y;
}

void dollDetect(Mat img, RotatedRect box, Point2f &goal)
{
    // Set ROI
    Mat mask(img.rows,img.cols,CV_8UC3,Scalar::all(0));
    Point2f vertex[4];
    box.points(vertex);
    vector<vector<Point>  > maskContours;
    vector<Point> ptr;
    for(int i=0;i<4;i++)
    {
        ptr.push_back(Point(vertex[i].x,vertex[i].y));
    }
    maskContours.push_back(ptr);
    drawContours(mask,maskContours,0,Scalar::all(255),-1);
    Mat element = getStructuringElement(MORPH_RECT,Size(130, 130));
    morphologyEx(mask, mask, MORPH_DILATE,element); //erode
    Mat imgROI;
    img.copyTo(imgROI,mask);
    //imshow("ROI", imgROI);
    //waitKey(0);

    // extract the blue
    Mat img_hsv_v, img_hsv_s, img_bgr_r;
    img_hsv_v = colorConversion(imgROI, HSV_V);
    img_hsv_s = colorConversion(imgROI, HSV_S);
    img_bgr_r = colorConversion(imgROI, BGR_R);

    Mat blue_bgr_low(Scalar(0));
    Mat blue_bgr_high(Scalar(40));
    Mat blue_hsv_s_low(Scalar(210));
    Mat blue_hsv_s_high(Scalar(255));
    Mat blue_hsv_v_low(Scalar(80));
    Mat blue_hsv_v_high(Scalar(255));
    Mat img_blue_v_threshold, img_blue_s_threshold, img_blue_r_threshold;
    Mat img_threshold_blue;
    inRange(img_bgr_r,   blue_bgr_low,   blue_bgr_high,   img_blue_r_threshold);
    inRange(img_hsv_s,   blue_hsv_s_low,   blue_hsv_s_high,   img_blue_s_threshold);
    inRange(img_hsv_v,   blue_hsv_v_low,   blue_hsv_v_high,   img_blue_v_threshold);
    img_blue_r_threshold = ~img_blue_r_threshold;
    img_blue_s_threshold = ~img_blue_s_threshold;
    img_blue_v_threshold = ~img_blue_v_threshold;
    img_threshold_blue = img_blue_r_threshold + img_blue_s_threshold + img_blue_v_threshold;
    //imshow("r",img_blue_r_threshold);
    //imshow("s",img_blue_s_threshold);
    //imshow("b",img_threshold_blue);
    //waitKey(0);

    element = getStructuringElement(MORPH_CROSS,Size(10, 10));
    morphologyEx(img_threshold_blue,img_threshold_blue,MORPH_DILATE,element); //erode
    element = getStructuringElement(MORPH_CROSS,Size(25, 25));
    morphologyEx(img_threshold_blue,img_threshold_blue,MORPH_ERODE,element); //dliate
    img_threshold_blue = ~img_threshold_blue;
    imshow("blue",img_threshold_blue);

    vector<vector<Point> > contours_blue, contours_red, contours_yellow;
    vector<Vec4i> hierarchy_blue, hierarchy_red, hierarchy_yellow;
    findContours(img_threshold_blue, contours_blue, hierarchy_blue, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

    // Find the max area of contours
    RotatedRect box2;
    goal.x = 0;
    goal.y = 0;
    float minp = img.rows * img.cols;
    int cout[4] = {1};
    vector<Point2f> dollcenter;
    for (int i=0;i<contours_blue.size();i++)
    {
        int contourSize = contourArea(contours_blue[i]);
        //cout << contourSize << endl;
        if (contourSize>1000)
        {
            box2 = minAreaRect(contours_blue[i]);
            Point2f vertex[4];
            box2.points(vertex);
            for(int j=0;j<4;j++)
            {
                line(img, vertex[j], vertex[(j+1)%4], Scalar(0,255,0), 3);
            }
            dollcenter.push_back(box.center);
            //zuo shang jiao
            float k = sqrt(pow(box2.center.x, 2.0) + pow(box2.center.y, 2.0));
            /*
            if (((k - minp) < 10) && (k > minp))
            {
                if (box2.center.x < goal.x)
                {
                    minp = k;
                    goal.x = box2.center.x;
                    goal.y = box2.center.y;
                }
            } else*/
            if (k < minp)
            {
                minp = k;
                goal.x = box2.center.x;
                goal.y = box2.center.y;
            }
            //circle(img, box.center, 3, Scalar(0,0,255));
        }
    }
    /*
    for (int i=0; i<dollcenter.size()-1; i++)
    {
        for (int j=i+1; j<dollcenter.size(); j++)
        {
            float k1 = sqrt(pow(dollcenter[i].x, 2.0) + pow(dollcenter[i].y, 2.0));
            float k2 = sqrt(pow(dollcenter[j].x, 2.0) + pow(dollcenter[j].y, 2.0));
            if (k1>k2) swap(dollcenter[i], dollcenter[j]);
        }
    }
    for (int i=0; i<dollcenter.size(); i++)
    {
        if (cout[i] == 0) continue;
        goal.x = dollcenter[i].x;
        goal.y = dollcenter[i].y;
        cout[i]--;
    }*/

    circle(img, goal, 3, Scalar(0,255,0));
    //imshow("dst",img);
    //waitKey(0);
}



