#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

image_transport::Publisher* pImage_pub = NULL;
image_transport::Publisher* pImage_pub2 = NULL;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    // ROS 이미지 메시지를 OpenCV 이미지로 변환
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat image = cv_ptr->image;

    // 이미지 전처리
    cv::Mat hsv;
    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);

    // 하얀색 선 구하기
    cv::Scalar lower_white = cv::Scalar(0, 0, 200);
    cv::Scalar upper_white = cv::Scalar(180, 30, 255);
    cv::Mat white_mask;
    cv::inRange(hsv, lower_white, upper_white, white_mask);

    // 엣지 검출
    cv::Mat edges;
    cv::Canny(white_mask, edges, 50, 150);

    // 선분 추출
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(edges, lines, 1, CV_PI / 180, 50, 50, 10);

    // 주차 선 인식
    for (size_t i = 0; i < lines.size(); i++)
    {
      cv::Vec4i line = lines[i];
      // 선분의 길이 계산
      float length = cv::norm(cv::Point(line[0], line[1]) - cv::Point(line[2], line[3]));

      // 주차 선으로 판단할 최소 길이 설정 (예시로 30으로 설정)
      float min_length = 100.0;

      // 주차 선으로 판단
      if (length < min_length)
      {
        cv::line(image, cv::Point(line[0], line[1]), cv::Point(line[2], line[3]), cv::Scalar(0, 255, 0), 2);
      }
    }

    sensor_msgs::ImagePtr img_msg;
    sensor_msgs::ImagePtr img_msg2;
    std_msgs::Header header;

    header.stamp = ros::Time::now();
    header.frame_id = "sign_detector_node_output";
    img_msg = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, image).toImageMsg();
    img_msg2 = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, edges).toImageMsg();
    pImage_pub->publish(img_msg);
    pImage_pub2->publish(img_msg2);

  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "parking_line_detection_node");
  ros::NodeHandle nh;

  image_transport::ImageTransport it(nh);
  image_transport::Publisher img_pub;
  image_transport::Publisher img_pub2;
  pImage_pub = &img_pub;
  pImage_pub2 = &img_pub2;
  image_transport::Subscriber sub = it.subscribe("/camera/image", 1, imageCallback);
  img_pub = it.advertise("/park/output", 1);
  img_pub2 = it.advertise("/park/edges", 1);
  ros::spin();
  return 0;
}
