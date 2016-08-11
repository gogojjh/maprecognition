#include "maprecognition/findrect.h"
#include "maprecognition/base.h"

void dilateROI(const Mat src, Mat &dst, Mat mask)
{
    dst=Mat(src.size(), src.depth(), Scalar(255,255,255));
    Mat element = getStructuringElement( 0, Size( 3,3 ), Point(1,1) );
    morphologyEx( mask, mask,  MORPH_DILATE, element );
    src.copyTo(dst ,mask);
}

bool rect_finalcheck(const Mat src, Mat mask)
{
    Mat img_gray,mask_gray,mask_result;
    cvtColor(src,img_gray,CV_BGR2GRAY);
    img_gray.copyTo(mask_gray,~mask);

    /*get no&area of ROI contours*/
    Mat mask_edges,mask_filled;
    medianBlur(mask_gray,mask_gray,5);
    imshow("medianblur",mask_gray);
    Canny(mask_gray,mask_edges,THRESHOLD_CANNYL ,THRESHOLD_CANNYH);
    imshow("mask_edges", mask_edges);
    //static int i = 0;
    //i++;
    //char saveFileName[50];
    //sprintf(saveFileName, "/home/gogojjh/QT/mapRecognition/data/%d.jpg", i);
    //imwrite(saveFileName, mask_edges);
    Mat element = getStructuringElement(0, Size(6, 6));
    morphologyEx(mask_edges,mask_filled, MORPH_DILATE,  element);
    //imshow("filled",mask_filled);
    vector<vector<Point> >contours;
    vector<Vec4i> hierarchy;
    findContours(mask_filled, contours, hierarchy,CV_RETR_LIST,CHAIN_APPROX_SIMPLE );
    if(contours.size()>5) return true;
    return false;
}

void findrect(const Mat src, const Mat imgBinary, Point2f &minG, float &minD, bool &minF, Mat &result, Mat &mask)
{
    /* dilate */
    Mat imgResult;
    src.copyTo(imgResult);
    Mat element = getStructuringElement(1, Size(5, 5));
    morphologyEx(imgBinary, imgBinary, MORPH_DILATE,  element);;
    imshow("img", imgBinary);

    vector<Vec4i> lines;
    /* operation in Red Areas*/
    HoughLinesP(imgBinary, lines, 1, CV_PI/90, 50, 10, 0 );
    Mat imgLines = Mat::zeros(imgBinary.rows,imgBinary.cols,imgBinary.type());
    for( size_t i = 0; i < lines.size(); i++ )
    {
        Vec4i l = lines[i];
        line( imgLines, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255), 10, CV_AA);
    }
    // draw the line
    // imshow("imgLineR", imgLines);

    /* find contours through the lines*/
    vector<vector<Point> > contoursr;
    vector<Vec4i> hierarchyr;
    Mat imgr2c;
    imgLines.copyTo(imgr2c);

    element = getStructuringElement(1, Size(20, 20));
    morphologyEx(imgr2c, imgr2c, MORPH_DILATE,  element);
    //imshow("img2r",imgr2c);

    int top = 0.005*src.rows; int bottom = 0.005*src.rows;
    int left = 0.005*src.cols; int right = 0.005*src.cols;
    Scalar border(255);
    copyMakeBorder( imgr2c, imgr2c, top, bottom, left, right, BORDER_CONSTANT, border );

    imgr2c = ~imgr2c;

    vector<Vec4i> hierarchy;

    /* find interior contours */
    //findContours( imgr2c, contoursr, hierarchyr,CV_RETR_CCOMP,CHAIN_APPROX_SIMPLE );
    findContours( imgr2c, contoursr, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE );

    Mat imgContour;
    imgResult.copyTo(imgContour);
    Mat imgROI;
    vector<Mat> imgAp;
    Scalar color(180, 180, 180);
    Point2f center(src.cols/2, src.rows/2);
    float near_point = minD;
    float angle;
    Rect goal_next_area(0,0,0,0);
    Point2f goal;
    Point2f vertex_dst[4];

    for(int i = 0;i<contoursr.size();i ++)
    {
            //drawContours(imgContour,contoursr,i,color,3,8,vector<Vec4i>(),0,Point());
            //if ((hierarchyr[i][3]) != -1)
            //{
                float nowarea = contourArea(contoursr[i]);
                //float length = arcLength(contoursr[i], true);
                //float p = nowarea/length * 1.0;
                if ((nowarea > 5000) && (nowarea <20000))
                {
                    //cout << "Area: " << nowarea << " Length: " << length << " Property: " << p << endl;
                    //drawContours(imgResult,contoursr,i,color,3,8,vector<Vec4i>(),0,Point());

                    /* use rectangles */
                    RotatedRect box = minAreaRect(Mat(contoursr[i]));
                    cout << box.size.width*box.size.height << endl;
                    if (nowarea / (box.size.width*box.size.height) * 1.0 < THRESHOLD_RECT) continue;
                    Point2f vertex[4];
                    box.points(vertex);
                    for(int j = 0;j < 4;j ++)
                    {
                        line(imgResult,vertex[j],vertex[(j+1)%4],CV_RGB(255,0,0),2,CV_AA);
                    }

                    Rect rect = box.boundingRect();
                    rect.x = max(0, rect.x); rect.y = max(0, rect.y);
                    rect.width = rect.x+rect.width>=src.cols ? src.cols-rect.x-1 : rect.width;
                    rect.height = rect.y+rect.height>=src.rows ? src.rows-rect.y-1 : rect.height;

                    imgROI = src(rect);
                    //imshow("ROI", imgROI);
                    //waitKey(0);

                    /* get the center */
                    Point2f imgROI_center;
                    imgROI_center.x = (vertex[0].x + vertex[1].x + vertex[2].x + vertex[3].x)/4;
                    imgROI_center.y = (vertex[0].y + vertex[1].y + vertex[2].y + vertex[3].y)/4;
                    //circle(imgResult,imgROI_center,2,Scalar(0,0,255),2);

                    /* find the average length */
                    float length[5] = {0};
                    length[0] = (vertex[0].x - vertex[1].x)*(vertex[0].x - vertex[1].x) + (vertex[0].y - vertex[1].y)*(vertex[0].y - vertex[1].y);
                    length[1] = (vertex[1].x - vertex[2].x)*(vertex[1].x - vertex[2].x) + (vertex[1].y - vertex[2].y)*(vertex[1].y - vertex[2].y);
                    length[2] = (vertex[2].x - vertex[3].x)*(vertex[2].x - vertex[3].x) + (vertex[2].y - vertex[3].y)*(vertex[2].y - vertex[3].y);
                    length[3] = (vertex[0].x - vertex[3].x)*(vertex[0].x - vertex[3].x) + (vertex[0].y - vertex[3].y)*(vertex[0].y - vertex[3].y);
                    for(int k = 0;k < 4;k++)
                    {
                        length[5]+=length[i];
                    }
                    length[5]/=4;

                    /*check No&area of contounrs in box*/
                    /*set find_object=0 and refresh mask in specific cases*/
                    Mat mask_finalcheck(src.size(), src.depth(), Scalar::all(255));
                    drawContours(mask_finalcheck, contoursr, i, Scalar::all(0),CV_FILLED);
                    if(rect_finalcheck(src,mask_finalcheck))  continue;

                    if (sqrt(pow(center.x-imgROI_center.x, 2.0)+pow(center.y-imgROI_center.y, 2.0)) < near_point)
                    {
                        minF = true;
                        near_point = sqrt(pow(center.x-imgROI_center.x, 2.0)+pow(center.y-imgROI_center.y, 2.0));

                        getPosition(Point2f(src.cols/2, src.rows/2), imgROI_center, goal);

                        vertex_dst[0] = vertex[0]; vertex_dst[1] = vertex[1]; vertex_dst[2] = vertex[2]; vertex_dst[3] = vertex[3];

                        // mask
                        goal_next_area = Rect(rect.x - src.cols/5, rect.y - src.rows/5, rect.width + src.cols/5, rect.height + src.rows/5);
                        goal_next_area.x = max(0, goal_next_area.x); rect.y = max(0, goal_next_area.y);
                        goal_next_area.width = goal_next_area.x + goal_next_area.width>=src.cols ? src.cols - goal_next_area.x-1 : goal_next_area.width;
                        goal_next_area.height = goal_next_area.y + goal_next_area.height>=src.rows ? src.rows - goal_next_area.y-1 : goal_next_area.height;
                        mask = Mat(src.size(), src.depth(),Scalar::all(255));
                        rectangle(mask,goal_next_area,Scalar::all(0),CV_FILLED);

                        //imshow("mask",mask);
                        /*
                        if (center.y > imgROI_center.y)
                        {
                            if ( imgROI_center.x>center.x )
                            {
                                angle=180*atan(float ((imgROI_center.x-center.x)/(center.y-imgROI_center.y))) / PI;
                            }
                            else
                            {
                                angle=360 + 180*atan(float ((imgROI_center.x-center.x)/(center.y-imgROI_center.y))) / PI;
                            }
                        }
                        else if (center.y < imgROI_center.y)
                        {
                             if (imgROI_center.x>center.x )
                            {
                               angle=180 - 180*atan(float ((imgROI_center.x-center.x)/(imgROI_center.y-center.y))) / PI;
                            }
                            else
                            {
                                angle=180 + 180*atan(float ((center.x-imgROI_center.x)/(imgROI_center.y-center.y))) / PI;
                            }
                        }*/
                    }
            }
    }

    //circle(imgResult, goal, 2,Scalar(0,0,255),2);
    for(int j = 0;j < 4;j ++)
    {
        line(result,vertex_dst[j],vertex_dst[(j+1)%4],CV_RGB(255,0,0),2,CV_AA);
    }

    if (near_point < minD)
    {
        minD = near_point;
        minG = goal;
        //an = angle;
        //imgResult.copyTo(result);
    }
    //imshow("Result", result);
    //imshow("imgContour", imgContour);
    //waitKey(0);
}

