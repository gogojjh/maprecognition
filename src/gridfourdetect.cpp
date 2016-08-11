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

/*
//公仔检测
Mat dollDetect(Mat img, RotatedRect box)
{
    // Set ROI
    Mat mask(img.rows,img.cols,CV_8UC3,Scalar::all(0));
    Point2f vertex[4];
    box.points(vertex);
    vector<vector<Point>> maskContours;
    vector<Point> ptr;
    for(int i=0;i<4;i++){
        ptr.push_back(Point(vertex[i].x,vertex[i].y));
    }
    maskContours.push_back(ptr);
    drawContours(mask,maskContours,0,Scalar::all(255),-1);
    Mat imgROI;
    img.copyTo(imgROI,mask);
    // convert to HSV
    Mat img_hsv_blue, img_hsv_red1,img_hsv_red2,img_hsv_yellow;
    cvtColor(imgROI, img_hsv_blue, CV_BGR2HSV);
    Mat img_threshold_blue, img_threshold_red, img_threshold_red1, img_threshold_red2, img_threshold_yellow;

    Mat element = getStructuringElement(MORPH_RECT,Size(5,5));
    morphologyEx(img_hsv_blue,img_hsv_blue ,MORPH_OPEN,element,Point(-1,-1),3);
    morphologyEx(img_hsv_blue,img_hsv_blue ,MORPH_CLOSE,element,Point(-1,-1),3);
    img_hsv_red1   = img_hsv_blue.clone();
    img_hsv_red2   = img_hsv_blue.clone();
    img_hsv_yellow = img_hsv_blue.clone();

    Mat blue_low(Scalar(60,43,46));
    Mat blue_higher(Scalar(140,255,255));

    Mat red1_low(Scalar(0,43,46));
    Mat red1_higher(Scalar(3,255,255));

    Mat red2_low(Scalar(170,43,46));
    Mat red2_higher(Scalar(180,255,255));

    Mat yellow_low(Scalar(20,43,46));
    Mat yellow_higher(Scalar(34,255,255));

    inRange(img_hsv_blue,   blue_low,   blue_higher,   img_threshold_blue);
    inRange(img_hsv_red1,   red1_low,   red1_higher,   img_threshold_red1);
    inRange(img_hsv_red2,   red2_low,   red2_higher,   img_threshold_red2);
    inRange(img_hsv_yellow, yellow_low, yellow_higher, img_threshold_yellow);
    img_threshold_red = img_threshold_red1 | img_threshold_red2;

    //imshow("123",img_threshold_red1);

    vector<vector<Point>> contours_blue, contours_red, contours_yellow;
    vector<Vec4i> hierarchy_blue, hierarchy_red, hierarchy_yellow;
    findContours(img_threshold_blue,   contours_blue,hierarchy_blue,     CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
    findContours(img_threshold_red,    contours_red,hierarchy_red,       CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
    findContours(img_threshold_yellow, contours_yellow,hierarchy_yellow, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
    // Find the max area of contours
    for (int i=0;i<contours_blue.size();i++){
        int contourSize = contourArea(contours_blue[i]);
        if(contourSize>6000){
            Rect box = boundingRect(contours_blue[i]);
            rectangle(img,box,Scalar(255,0,0),5);
        }
    }

    for (int i=0;i<contours_red.size();i++){
        int contourSize = contourArea(contours_red[i]);
        if(contourSize>10000){
            Rect box = boundingRect(contours_red[i]);
            rectangle(img,box,Scalar(0,0,255),5);
        }
    }

    for (int i=0;i<contours_yellow.size();i++){
        int contourSize = contourArea(contours_yellow[i]);
        if(contourSize>10000){
            Rect box = boundingRect(contours_yellow[i]);
            rectangle(img,box,Scalar(0,255,255),5);
        }
    }

    //imshow("dst",img);
    return img;
}

//ord匹配算法检测LED屏
int orbMatching(Mat img_1, Mat img_2)
{

    int n = 0;
    if (!img_1.data || !img_2.data)
    {
        cout << "error reading images " << endl;
        return -1;
    }

    ORB orb;
    vector<KeyPoint> keyPoints_1, keyPoints_2;
    Mat descriptors_1, descriptors_2;

    orb(img_1, Mat(), keyPoints_1, descriptors_1);
    orb(img_2, Mat(), keyPoints_2, descriptors_2);
    BruteForceMatcher<HammingLUT> matcher;
    vector<DMatch> matches;
    matcher.match(descriptors_1, descriptors_2, matches);

    double max_dist = 0; double min_dist = 70;
    //-- Quick calculation of max and min distances between keypoints
    for( int i = 0; i < descriptors_1.rows; i++ )
    {
        double dist = matches[i].distance;
        if( dist < min_dist ) min_dist = dist;
        if( dist > max_dist ) max_dist = dist;
    }

    //-- Draw only "good" matches (i.e. whose distance is less than 0.6*max_dist )
    //-- PS.- radiusMatch can also be used here.
    std::vector< DMatch > good_matches;
    for( int i = 0; i < descriptors_1.rows; i++ )
    {
        if( matches[i].distance < 0.45*max_dist )
        {
            good_matches.push_back( matches[i]);
            n++;
        }
    }
    cout << n << endl;
    Mat img_matches;
    drawMatches(img_1, keyPoints_1, img_2, keyPoints_2,
        good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
        vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
    imshow("match",img_matches);
    waitKey(0);
    return n;
}
*/

/*
int figureDetect_4 ( Mat img_src )
{
    //Mat img_src = imread("D:/vs/fff/img/1910.jpg");
    //Mat img_mode1 = imread("D:/vs/fff/img/img_mode12.jpg");
    //Mat img_mode2 = imread("D:/vs/fff/img/img_mode22.jpg");
    //resize(img_mode1,img_mode1,Size(img_mode1.cols/3,img_mode1.rows/3),0,0,CV_INTER_LINEAR);
    resize(img_mode2,img_mode2,Size(img_mode2.cols/3,img_mode2.rows/3),0,0,CV_INTER_LINEAR);
    resize(img_src,img_src,Size(img_src.cols/3,img_src.rows/3),0,0,CV_INTER_LINEAR);
    LedDetect test;
    //img_src = img_src - Scalar(100,100,100);
    waitKey(0);
    int n_mode1 = test.orbMatching(img_mode1,img_src );
    int n_mode2 = test.orbMatching(img_mode2,img_src );
    if ( n_mode1 > n_mode2 )
    {
        return 1;
    }
    else
    {
        return 2;
    }
}
*/

/*
int gridDetection_3( Mat img_src,float& angle )
{
    //resize(img_src,img_src,Size(img_src.cols/3,img_src.rows/3),0,0,CV_INTER_LINEAR);
    if (!img_src.data)
        return 0;
    SquareDetect test;
    RotatedRect box = test.squareDet(img_src);      //detect the box
    Point2f vertex[4];
    box.points(vertex);
    for(int i=0;i<4;i++){
        line(img_src,vertex[i],vertex[(i+1)%4],Scalar(255,0,0),5);    //draw the rectangle of the box
    }
    Point matchLoc((vertex[1].x+vertex[3].x)/2,(vertex[1].y+vertex[3].y)/2);
    Point midPoint(img_src.cols/2,img_src.rows/2);
    circle(img_src,matchLoc,10,Scalar(0,0,255),2,8,0);
    //rectangle(img_src,box,Scalar(0,255,0),5);
    int distance = sqrt(float (midPoint.x-matchLoc.x)*(midPoint.x-matchLoc.x)+(midPoint.y-matchLoc.y)*(midPoint.y-matchLoc.y));
    if (distance <= 50)
    {
       return -2;
    }
    //calculate the angle
    else
    {
        if (midPoint.y > matchLoc.y)
       {
           if (matchLoc.x > midPoint.x)    //first quadrant
           {
              angle=180*atan(float ((matchLoc.x-midPoint.x)/(midPoint.y-matchLoc.y)))/PI;
           }
           else            //fourth quadrant
           {
               angle=360 + 180*atan(float ((matchLoc.x-midPoint.x)/(midPoint.y-matchLoc.y)))/PI;
           }
       }
       else if (midPoint.y < matchLoc.y)
       {
            if (matchLoc.x > midPoint.x)            //second quadrant
           {
              angle=180 - 180*atan(float ((matchLoc.x-midPoint.x)/(matchLoc.y-midPoint.y)))/PI;
           }
           else                 //third quadrant
           {
               angle=180 + 180*atan(float ((midPoint.x-matchLoc.x)/(matchLoc.y-midPoint.y)))/PI;
           }
       }
       return -1;
   }
    //Mat imgROI = test.dollDetect(img_src,box);          //detect the doll
}*/
