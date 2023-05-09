#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>

using namespace cv;
using namespace std;

ros::Publisher twist_pub; // 토픽 발행을 위한 publisher 객체
image_transport::Publisher* pImg_pub1 = NULL;
image_transport::Publisher* pImg_pub2 = NULL;
image_transport::Publisher* pImg_pub3 = NULL;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    //ROS 메시지를 OpenCV 이미지로 변환
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // 이미지 크기 조정
    resize(cv_ptr->image, cv_ptr->image, Size(640, 480));

    // 마스크 색 범위 설정
    Scalar red_lower(0, 100, 100);
    Scalar red_upper(10, 255, 255);
    Scalar green_lower(50, 100, 100);
    Scalar green_upper(100, 255, 255);
    Scalar yellow_lower(20, 100, 100);
    Scalar yellow_upper(40, 255, 255);

    // 이미지를 HSV로 변환
    Mat hsv;
    cvtColor(cv_ptr->image, hsv, COLOR_BGR2HSV);

    // 각 색에 해당하는 마스크 생성
    Mat red_mask, green_mask, yellow_mask;
    inRange(hsv, red_lower, red_upper, red_mask);
    inRange(hsv, green_lower, green_upper, green_mask);
    inRange(hsv, yellow_lower, yellow_upper, yellow_mask);

    // 마스크를 사용하여 이미지에서 각 색을 분리
    Mat red_img, green_img, yellow_img;
    bitwise_and(cv_ptr->image, cv_ptr->image, red_img, red_mask);
    bitwise_and(cv_ptr->image, cv_ptr->image, green_img, green_mask);
    bitwise_and(cv_ptr->image, cv_ptr->image, yellow_img, yellow_mask);

    // 메시지 생성
    geometry_msgs::Twist twist_msg;
    if (countNonZero(green_mask) > 0) { // 초록불일 경우
        twist_msg.linear.x = 0.15;
        twist_msg.angular.z = 0.0;
    } else { // 그 외의 경우
        twist_msg.linear.x = 0.0;
        twist_msg.angular.z = 0.0;
    }

    // 메시지 발행
    twist_pub.publish(twist_msg);
		
		sensor_msgs::ImagePtr img_msg;
    sensor_msgs::ImagePtr img_msg2;
    sensor_msgs::ImagePtr img_msg3;
	  std_msgs::Header header;

    header.stamp = ros::Time::now();
    header.frame_id = "traffic_light_node_output";
    img_msg = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, green_mask).toImageMsg();
    img_msg2 = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, red_mask).toImageMsg();
    img_msg3 = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, yellow_mask).toImageMsg();
    pImg_pub1->publish(img_msg);
    pImg_pub2->publish(img_msg2);
    pImg_pub3->publish(img_msg3);
   
}

int main(int argc, char** argv)
{
// ROS 노드 초기화
ros::init(argc, argv, "traffic_light_detector");
ros::NodeHandle nh;
// 토픽 발행 설정
twist_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
image_transport::ImageTransport it_(nh); // 이미지 전송 클래스
image_transport::Subscriber image_sub_; // 이미지 수신자
image_transport::Publisher image_pub1_; // 이미지 발행자
pImg_pub1 = &image_pub1_;
image_transport::Publisher image_pub2_; // 이미지 발행자
pImg_pub2 = &image_pub2_;
image_transport::Publisher image_pub3_; // 이미지 발행자
pImg_pub3 = &image_pub3_;

image_pub1_ = it_.advertise("/traffic_green",1);
image_pub2_ = it_.advertise("/traffic_red",1);
image_pub3_ = it_.advertise("/traffic_yellow",1);

// 이미지 토픽 구독 설정
ros::Subscriber image_sub = nh.subscribe("/jetbot_camera/raw", 1, imageCallback);

// ROS 메시지 수신 대기
while(ros::ok()){
	ros::spinOnce();
}
	return 0;
}
