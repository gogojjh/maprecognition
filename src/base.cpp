#include "maprecognition/base.h"

Mat colorConversion(Mat img, HsvType imgtype)
{
     //GaussianBlur(img, img, Size(5,5),0,0);
     if(imgtype==HSV||imgtype==HSV_H||imgtype==HSV_S||imgtype==HSV_V)
    {
        Mat img_hsv, img_h, img_s, img_v;
        cvtColor(img, img_hsv, CV_BGR2HSV);
        vector<Mat> hsv_channels;
        split(img_hsv, hsv_channels);
        img_h = hsv_channels[0];
        img_s = hsv_channels[1];
        img_v = hsv_channels[2];
        switch (imgtype)
        {
            case 0: return img_hsv;
            case 1: return img_h;
            case 2: return img_s;
            case 3: return img_v;
            default: break;
        }
    }
    if(imgtype==BGR||imgtype==BGR_B||imgtype==BGR_G||imgtype==BGR_R)
    {
        Mat b_img, g_img, r_img;
        vector<Mat> bgr_channels;
        split(img,bgr_channels);
        b_img=bgr_channels[0];
        g_img=bgr_channels[1];
        r_img=bgr_channels[2];
        switch(imgtype)
        {
            case 4: return img;
            case 5: return b_img;imshow("r",r_img);
            case 6: return g_img;
            case 7: return r_img;
        }
    }
    return img;
}

// get the position of destination
void getPosition(const Point2f p1, const Point2f p2, Point2f &ans)
{
    ans.y=p2.x - p1.x;
    ans.x=(-1) * (p2.y - p1.y);
    //cout<< ans <<endl;
}

// get the pose
float markerPosition(Mat &img, Point2f *pointsIn)
{
//    Mat img_out(400,400,CV_8UC1,Scalar(0,0,0));
//    Point2f pointsRes[4];
//    pointsRes[0] = Point2f(0, 0);
//    pointsRes[1] = Point2f(400 - 1, 0);
//    pointsRes[2] = Point2f(400 - 1, 400 - 1);
//    pointsRes[3] = Point2f(0, 400 - 1);

    // Size of the Marker, a standard marker
    vector<Point3f> objectPoints;
    objectPoints.push_back(Point3f(0,0,0));
    objectPoints.push_back(Point3f(393,0,0));
    objectPoints.push_back(Point3f(393,393,0));
    objectPoints.push_back(Point3f(0,393,0));

    vector<Point2f> imagePoints;

    Point2f p(0,0);
    p.x = pointsIn[1].x; p.y = pointsIn[1].y;
    pointsIn[1].x = pointsIn[3].x; pointsIn[1].y = pointsIn[3].y;
    pointsIn[3].x = p.x; pointsIn[3].y = p.y;

    for(int i=0;i<4;i++)
    {
        imagePoints.push_back(Point2f(pointsIn[i].x,pointsIn[i].y));
        cout << "X: " << pointsIn[i].x << "   Y: " << pointsIn[i].y << endl;
    }

    float tmp1[3][3] = {{1074.37169, 0, 596.13023}, {0, 1075.04626, 336.42194}, {0, 0, 1}};
    Mat cameMatrix(3,3,CV_32FC1, tmp1);

    float tmp2[5] = {-0.41506, 0.24184, 0.00209, -0.00069, 0};
    Mat distCoeffs(1,5,CV_32F,tmp2);

    Mat rotat_vec(3,3,CV_32F), trans_vec(3,3,CV_32F);

    solvePnP(objectPoints, imagePoints, cameMatrix, distCoeffs, rotat_vec, trans_vec);

    Mat rotat_matrix(3, 3, CV_32F);
    Rodrigues(rotat_vec, rotat_matrix);

    // get the transformation matrix
    //Mat transformMatrix = getPerspectiveTransform(pointsIn, pointsRes);
    //cout << transformMatrix << endl;

    // Applies a perspective transformation to an image.
    //warpPerspective(img, img_out, transformMatrix, Size(400,400), cv::INTER_NEAREST);
    //cout<<rotat_vec.col(0).row(2)<<endl;
    //cout<<rotat_vec.at<double>(0,2)*57<<endl;
    //  cout<<"rotat_vec:"<<rotat_vec<<endl;
    //  cout<<"trans_vec:"<<trans_vec<<endl;
    //namedWindow("img_out");
    //imshow("img_out",img_out);

    //cout << "R_vector: " << rotat_vec*57 << endl;
    //cout << "T_vector: " << trans_vec << endl;
    //cout << "R_matrix: " << rotat_matrix << endl;
    return rotat_vec.at<double>(0,2)*57;
}

/*
Focal Length:          fc = [ 1760.80462   1760.28449 ] +/- [ 5.65463   5.71223 ]
Principal point:       cc = [ 926.95610   535.82561 ] +/- [ 6.68057   5.33905 ]
Skew:             alpha_c = [ 0.00000 ] +/- [ 0.00000  ]   => angle of pixel axes = 90.00000 +/- 0.00000 degrees
Distortion:            kc = [ -0.41624   0.23446   -0.00058   0.00086  0.00000 ] +/- [ 0.00756   0.03623   0.00048   0.00093  0.00000 ]
Pixel error:          err = [ 0.24291   0.15701 ]
*/
