#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

ros::Publisher* in_pub = NULL;
ros::Publisher edge_pub;

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  Mat img_gray, img_edges;
  cvtColor(cv_ptr->image, img_gray, COLOR_BGR2GRAY);
  Canny(img_gray, img_edges, 100, 200); // Canny Edge

  // Publish edge image
  sensor_msgs::ImagePtr edge_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", img_edges).toImageMsg();
  in_pub->publish(edge_msg);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "image_processing_node");
  ros::NodeHandle nh;

  // Publish edge topic
  edge_pub = nh.advertise<sensor_msgs::Image>("/image_edges", 1);
  // Subscribe to camera topic
  ros::Subscriber image_sub = nh.subscribe("/jetbot_camera/raw", 1,imageCallback);
 
  in_pub = &edge_pub;
  while(ros::ok()){
  	ros::spinOnce();
 }
  return 0;
}
