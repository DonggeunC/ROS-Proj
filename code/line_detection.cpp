//각도 값 라디안으로 변환하는 함수
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

#define ArraySize 100
#define PI 3.141592

using namespace cv;
using namespace std;

Point* aPoint1 = new Point[ArraySize], * aPoint2 = new Point[ArraySize], * bPoint1 = new Point[ArraySize], * bPoint2 = new Point[ArraySize];
int aIndex = 0, bIndex = 0;

float get_radian(float Val) //각도 계산하여 반환
{
    return PI * Val / 180;  //Val 값을 라디안 값으로 변환
}