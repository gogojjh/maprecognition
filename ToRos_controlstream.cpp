/*
#include "maprecognition/controlstream.h"
#include "maprecognition/base.h"
#include "maprecognition/transformer.h"

CONTROLSTREAM::CONTROLSTREAM(ros::NodeHandle& n)
{
    init_publishers(n);
    init_subscribers(n);
}

void CONTROLSTREAM::ControlStreamCallback(const sensor_msgs::ImageConstPtr& msg)
{
    static const std::string OPENCV_WINDOW = "Image window";
    static const std::string RESULT_WINDOW = "Result window";
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr=cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    this->img_mat=cv_ptr->image;

    Transformer trans;

    int choice = 0;
    float next_direction;
    bool next_state;
    Mat frame;

    this->current_direction = UP;
    next_direction = current_direction * 90;
    next_state = false;

    //undistort the images
    //Mat image_undistorted;
    //imageCalibration(frame, image_undistorted);

    trans.sTransformer(cv_ptr->image);
    Transformer::number ++;

    switch (choice)
    {
        case 0: trans.detectApriltag(current_direction, next_direction, next_state); break;
        case 1: trans.detectTwo(current_direction, next_direction, next_state); break;
        case 2: trans.detectThree(current_direction, next_direction, next_state); break;
        case 3: trans.detectFour(current_direction, next_direction, next_state); break;
        case 4: trans.detectRed(current_direction, next_direction, next_state); break;
        default:break;
    }

    cout << "Angle: " << next_direction << endl;

    cv::imshow(RESULT_WINDOW, trans.gImage(2));

    if ((Transformer::number % 5) == 0)
    {
        char saveFileName[50];
        sprintf(saveFileName, "/home/gogojjh/catkin_ws/src/maprecognition/image/%d.jpg", Transformer::number/5);
        cv::imwrite(saveFileName, trans.gImage((2)));
    }
    cv::waitKey(100);
    if (next_state)  controlstream_publisher.publish(next_direction);
                        else controlstream_publisher.publish(-1);
}

 */


