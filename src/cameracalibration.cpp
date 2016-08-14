#include "maprecognition/cameracalibration.h"

double calDistance(Mat transform_matrix, double x1, double y1, double x2, double y2, double &wd)
{
    double a1, a2, a3, a4, a5, a6, a7, a8, a9;
    cout << transform_matrix << endl;
    a1 = transform_matrix.at<double>(0, 0);
    a2 = transform_matrix.at<double>(0, 1);
    a3 = transform_matrix.at<double>(0, 2);
    a4 = transform_matrix.at<double>(1, 0);
    a5 = transform_matrix.at<double>(1, 1);
    a6 = transform_matrix.at<double>(1, 2);
    a7 = transform_matrix.at<double>(2, 0);
    a8 = transform_matrix.at<double>(2, 1);
    a9 = transform_matrix.at<double>(2, 2);
    double wx1, wy1;
    wx1 = (a1*x1 + a2*y1 + a3) / (a7*x1 + a8*y1 + a9);
    wy1 = (a4*x1 + a5*y1 + a6) / (a7*x1 + a8*y1 + a9);
    double wx2, wy2;
    wx2 = (a1*x2 + a2*y2 + a3) / (a7*x2 + a8*y2 + a9);
    wy2 = (a4*x2 + a5*y2 + a6) / (a7*x2 + a8*y2 + a9);
    wd = sqrt(pow(wx1-wx2, 2.0)+pow(wy1-wy2, 2.0));
}

Mat image_calibration(Mat image)
{
    char imageFileName[30];
    Size image_size;
    //  information of camera
    Mat intrinsic_matrix = Mat(3,3, CV_64FC1, Scalar::all(0));
    Mat distortion_coeffs = Mat(1,5, CV_64FC1, Scalar::all(0));
    double fx = 1074.37169;
    double fy = 1075.04626;
    double cx = 596.130236;
    double cy = 336.421941;
    double dis1 = -0.415066;
    double dis2 = 0.2418456;
    double dis3 = 0.0020909;
    double dis4 = -0.000691;
    double dis5 = 0;
    intrinsic_matrix.at<double>(0,0) = fx;
    intrinsic_matrix.at<double>(1,1) = fy;
    intrinsic_matrix.at<double>(0,2) = cx;
    intrinsic_matrix.at<double>(1,2) = cy;
    intrinsic_matrix.at<double>(2,2) = 1.0;
    distortion_coeffs.at<double>(0,0) = dis1;
    distortion_coeffs.at<double>(0,1) = dis2;
    distortion_coeffs.at<double>(0,2) = dis3;
    distortion_coeffs.at<double>(0,3) = dis4;
    distortion_coeffs.at<double>(0,4) = dis5;

    // calibrate the images
    image_size = image.size();
    Mat mapx = Mat(image_size,CV_64FC1);
    Mat mapy = Mat(image_size,CV_64FC1);
    Mat R = Mat::eye(3,3,CV_32F);
    initUndistortRectifyMap(intrinsic_matrix, distortion_coeffs, R, intrinsic_matrix, image_size, CV_32FC1, mapx, mapy);
    Mat image_undistorted;
    image.copyTo(image_undistorted);
    remap(image, image_undistorted, mapx, mapy, INTER_LINEAR);
    imshow("1", image_undistorted);
    waitKey(0);
    return image_undistorted;

    // you should have a image containing some points for rotation and transition
    /*
    Mat extrinsic_matrix(3, 3, CV_64FC1, Scalar::all(0));
    extrinsic_matrix.at<double>(0,0) = 1.0;
    extrinsic_matrix.at<double>(1,1) = 1.0;
    extrinsic_matrix.at<double>(2,2) = 1.0;
    cout << extrinsic_matrix << endl;
    Mat hu;
    hu = intrinsic_matrix * extrinsic_matrix;
    cout << hu << endl;
    Mat hu2 = hu.inv();
    cout << hu2 << endl;
    double wd;
    double x1 = 544-image.cols, y1 = 4-image.rows, x2 = 1236-image.cols, y2 = 70-image.rows;
    //calDistance(hu2, image.cols(), image.rows(), x, y, wd);
    calDistance(hu2, x1, y1, x2, y2, wd);
    cout << "Distance: " << wd << endl;
    */
}






