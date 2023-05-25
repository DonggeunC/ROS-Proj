#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Twist.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

// 주행 방향 구하기
void get_steering_direction(Mat& binary, double& direction)
{
    // Hough Transform을 이용하여 직선 검출
    vector<Vec2f> lines;
    HoughLines(binary, lines, 1, CV_PI / 180, 50);

    // 검출된 직선 중 가장 긴 직선 선택
    double max_length = 0;
    for (size_t i = 0; i < lines.size(); i++)
    {
        double length = lines[i][0];
        if (length > max_length)
        {
            max_length = length;
            direction = lines[i][1];
        }
    }

    // 방향을 [-pi/2, pi/2] 범위로 조정
    if (direction > CV_PI / 2)
    {
        direction -= CV_PI;
    }
    else if (direction < -CV_PI / 2)
    {
        direction += CV_PI;
    }
}
