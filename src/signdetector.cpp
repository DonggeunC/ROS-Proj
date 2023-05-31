#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>

image_transport::Publisher* pImage_pub=NULL;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try {
    // Convert ROS image message to OpenCV Mat
    cv::Mat scene = cv_bridge::toCvShare(msg, "bgr8")->image;

    // Load object image
    cv::Mat object = cv::imread("/home/jetson/catkin_ws/src/lanedetection/file/parking2.jpg", cv::IMREAD_GRAYSCALE);

    // ORB object creation
    cv::Ptr<cv::ORB> orb = cv::ORB::create(300, 1.5f, 5, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31, 20);

    // Compute keypoints and descriptors
    std::vector<cv::KeyPoint> keypoints_object, keypoints_scene;
    cv::Mat descriptors_object, descriptors_scene;
    orb->detectAndCompute(object, cv::Mat(), keypoints_object, descriptors_object);
    orb->detectAndCompute(scene, cv::Mat(), keypoints_scene, descriptors_scene);

    // Matching object and scene descriptors
    cv::BFMatcher matcher(cv::NORM_HAMMING);
    std::vector<std::vector<cv::DMatch>> matches;
    matcher.knnMatch(descriptors_object, descriptors_scene, matches, 2);

    // Filter good matches based on distance ratio
    std::vector<cv::DMatch> good_matches;
    for (int i = 0; i < matches.size(); i++) {
      if (matches[i][0].distance < 0.75 * matches[i][1].distance) {
        good_matches.push_back(matches[i][0]);
      }
    }

    // If enough good matches are found, object is detected in the scene
    if (good_matches.size() >= 8) {
      ROS_INFO("Object detected!");
	}	
      // Draw detected object on the scene image
      cv::Mat img_matches;
      cv::drawMatches(object, keypoints_object, scene, keypoints_scene,
          good_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
          std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
sensor_msgs::ImagePtr img_msg;
               std_msgs::Header header;

                header.stamp = ros::Time::now();
                header.frame_id = "sign_detector_node_output";
                img_msg = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, img_matches).toImageMsg();
                pImage_pub->publish(img_msg);
    
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sign_detection_node");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher image_pub;
  pImage_pub = &image_pub;
  image_transport::Subscriber image_sub = it.subscribe("/camera/image", 1,imageCallback);
  image_pub = it.advertise("/sign_detect", 1);

while(ros::ok()){
	ros::spinOnce();
}
return 0;
}
