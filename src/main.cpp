#include "maprecognition/base.h"
#include "maprecognition/transformer.h"
#include "maprecognition/cameracalibration.h"

static const std::string OPENCV_WINDOW = "Image window";
static const std::string RESULT_WINDOW = "Result window";
int Transformer::number = 0;
int Transformer::gridfourdetect_threshold = 66;

void setting(Point2f &goal, float &distance, bool &find_object, Mat src)
{
    goal.x = 0; goal.y = 0;
    distance = src.cols * src.rows;
    find_object = false;
}

int main()
{
    //VideoCapture capture("/home/gogojjh/QT/mapRecognition/data/Origin253.jpg");
    //VideoCapture capture(1);
    /*
    if (!capture.isOpened())
    {
        cout << "failed to open the camera..." << endl;
        return 0;
    }*/

    Transformer trans;

    namedWindow(OPENCV_WINDOW, 2);
    namedWindow(RESULT_WINDOW, 2);

    int choice = 3;
    float distance;        /* the distance of object */
    bool find_object;   /* if find object */
    float pose;
    Mat frame;
    Point2f goal;
    int countF1 = 0;
    int countF2 = 0;
    int flag;

    while (1)
    {
        //capture >> frame;
        frame = imread("/home/gogojjh/QT/mapRecognition/data815/270.jpg", 1);

        //Mat frame_undistorted = image_calibration(frame);
        //imshow("undistorted", frame_undistorted);
        //waitKey(0);
        //char saveFileName[50];
        //sprintf(saveFileName, "/home/gogojjh/QT/mapRecognition/image3/zzzzz/%d.jpg", Transformer::number);
        //frame = imread(saveFileName, 1);

        //undistort the images
        //Mat image_undistorted;
        //imageCalibration(frame, image_undistorted);

        trans.sTransformer(frame);
        Transformer::number ++;
        //if (Transformer::number < 2000) continue;

        switch (choice)
        {
            case 0: setting(goal, distance, find_object, trans.gImage(0)); trans.detectApriltag(goal, distance, find_object); break; // er wei ma
            case 1: setting(goal, distance, find_object, trans.gImage(0)); trans.detectTwo(goal, distance, find_object); break; // jiu gong ge
            case 2: setting(goal, distance, find_object, trans.gImage(0)); trans.detectThree(goal, distance, find_object); break; // he ma bian kuang
            case 3: setting(goal, distance, find_object, trans.gImage(0)); trans.detectFour(goal, distance, find_object); break; // led \ biankuang
            case 4: setting(goal, distance, find_object, trans.gImage(0)); trans.detectRed(goal, distance, find_object); break;
            default:break;
        }
        /*
        cout<<"distance:"<<distance<<endl;
        cout<<"F1111111:::::::"<<countF1<<endl;
        cout<<"F2222222:::::::"<<countF2<<endl;
        int ACK=3;
        int thresholdD=20;

        if (find_object&countF1<3)    countF1++;
        else
        {
            if(!find_object)
            {
                countF1 = 0;
                cout<<"missing target"<<endl;
                Transformer::gridfourdetect_threshold  = 66;
            }
        }
        if (countF1 == ACK)
        {
            cout << "***********object_targeted**********" << endl;
            Transformer::gridfourdetect_threshold  = 80;
            if(distance<thresholdD)
            {
                countF2++;
            }
            else
                countF2=0;
            if(countF2==ACK)
            {
                cout<<"///////////////////I wanna drop WAWA/////////////////"<<endl;
                countF2=0;
                countF1=0;
                Transformer::gridfourdetect_threshold = 66;
            }
            */
            cout << "Goal: " << goal << endl;
            cout << "Distance: " << distance << endl;
            cout << "Find Object: " << find_object << endl;
            /*
            if (Transformer::number % 1 == 0)
            {
                char saveFileName[50];
                sprintf(saveFileName, "/home/gogojjh/QT/mapRecognition/image2/Origin%d.jpg", Transformer::number);
                imwrite(saveFileName, frame);
                sprintf(saveFileName, "/home/gogojjh/QT/mapRecognition/image2/Result%d.jpg", Transformer::number);
                imwrite(saveFileName, trans.gImage((2)));
            }*/
        //}
        imshow(RESULT_WINDOW, trans.gImage(2));
        imshow(OPENCV_WINDOW, trans.gImage(0));
        waitKey(0);
    }

    return 0;
}

















