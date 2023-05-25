#include <iostream>
#include <map>
#include <vector>

#define OPENCV_470
//ROS용
//#include <ros/ros.h>
//#include <sensor_msgs/Image.h>
//#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#ifdef OPENCV_470
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/imgproc/types_c.h>
#endif // OPENCV_470

#ifdef _DEBUG
#pragma comment(lib,"opencv_world470d.lib")
#else	//RELEASE
#pragma comment(lib,"opencv_world470.lib")
#endif

using namespace cv;
using namespace std;

double angle;

int main(int argc, char** argv) {
    Mat image = imread("level.jpg");
    if (image.empty()) {
        cout << "Could not open or find the image" << endl;
        return -1;
    }
    //ROI영역을 설정하기 위한 4개의 포인트 설정
    Point pts[4];
    pts[0] = Point(0.2 * image.cols, 0.4 * image.rows);
    pts[1] = Point(0.7 * image.cols, 0.4 * image.rows);
    pts[2] = Point(0.7 * image.cols, 0.7 * image.rows);
    pts[3] = Point(0.2 * image.cols, 0.7 * image.rows);
    //HSV이미지로 변환 -> 빨간색을 추려내기 위함
    Mat hsv_image;
    cvtColor(image, hsv_image, COLOR_BGR2HSV);
    //마스크를 생성하여 빨간색만 검출 및 ROI영역 이미지를 만들어 마스크에 적용 -> 정면의 차단기가 보이는 부분만 검출
    Mat mask;
    inRange(hsv_image, Scalar(0, 70, 50), Scalar(10, 255, 255), mask);
    Mat mask_roi = Mat::zeros(image.rows, image.cols, CV_8UC1);
    fillConvexPoly(mask_roi, pts, 4, Scalar(255, 0, 0));
    bitwise_and(mask, mask_roi, mask);
    //빨간색이 구성된 사각형을 구하고 그 사각형의 중심을 centers벡터에 저장
    vector<vector<Point>> contours;
    findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    Mat output = image.clone();
    vector<Point> centers;
    for (size_t i = 0; i < contours.size(); i++) {
        Rect rect = boundingRect(contours[i]);

        if (rect.width > 10 && rect.height > 10) {
            drawContours(output, contours, i, Scalar(0, 0, 255), 2);
            Point center(rect.x + rect.width / 2, rect.y + rect.height / 2);
            circle(output, center, 3, Scalar(0, 255, 0), -1);
            centers.push_back(center);
        }
    }

    // 가장 좌측의 점과 가장 우측의 점을 구함
    Point leftmost_center = centers[0];
    Point rightmost_center = centers[0];
    for (size_t i = 1; i < centers.size(); i++) {
        if (centers[i].x < leftmost_center.x) {
            leftmost_center = centers[i];
        }
        if (centers[i].x > rightmost_center.x) {
            rightmost_center = centers[i];
        }
    }

    // 구한 두 점을 하나의 선으로 연결 -> 차단기의 중심선
    line(output, leftmost_center, rightmost_center, Scalar(255, 0, 0), 2);

    // Calculate the angle of the line
    double dx = rightmost_center.x - leftmost_center.x;
    double dy = rightmost_center.y - leftmost_center.y;
    double angle = atan2(dy, dx) * 180 / CV_PI;

    // 중심선의 각도를 통하여 닫힘인지 열림인지 결정
    string shape;
    if (angle >= -10 && angle <= 10) {
        shape = "closed";
    }
    else if (angle >= 80 && angle <= 100) {
        shape = "open";
    }
    else {
        shape = "unknown";
    }

    // 이미지에 문자열로 출력
    Point text_position((leftmost_center.x + rightmost_center.x) / 2, leftmost_center.y - 10);
    putText(output, shape, text_position, FONT_HERSHEY_SIMPLEX, 0.8, Scalar(170, 150, 255), 2);


    namedWindow("Output", WINDOW_NORMAL);
    imshow("Output", output);
    waitKey(0);

    return 0;
}