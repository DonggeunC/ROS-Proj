#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Twist.h>

#include <opencv2/opencv.hpp>

#include "../include/RoadLaneDetector.h"

using namespace cv;

image_transport::Publisher* pImg_pub_1 = NULL;
image_transport::Publisher* pImg_pub_2 = NULL;
image_transport::Publisher* pImg_pub_3 = NULL;

class LaneDetectorNode {
public:
   LaneDetectorNode(ros::NodeHandle& nh, image_transport::ImageTransport& it) : nh_(nh), it_(it) {
        // Subscribe to the input video feed and publish the output Twist message
        image_sub_ = it_.subscribe("/jetbot_camera/raw", 1, &LaneDetectorNode::imageCallback, this);
        image_pub_1 = it_.advertise("/lane_detection/result",3);
        image_pub_2 = it_.advertise("/lane_detection/mask",3);
        image_pub_3 = it_.advertise("/lane_detection/edge",3);
        pImg_pub_1 = &image_pub_1;
        pImg_pub_2 = &image_pub_2;
        pImg_pub_3 = &image_pub_3;
        twist_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
        try {
            // Convert the received ROS image message to OpenCV Mat
            cv::Mat img_frame = cv_bridge::toCvShare(msg, "bgr8")->image;

            // Perform lane detection using the RoadLaneDetector class
            RoadLaneDetector roadLaneDetector;
            cv::Mat img_filter, img_edges, img_mask, img_lines, img_result;
            std::vector<cv::Vec4i> lines;
            std::vector<std::vector<cv::Vec4i> > separated_lines;
            std::vector<cv::Point> lane;
            std::string dir;

            // Apply the lane detection algorithm
            img_filter = roadLaneDetector.filter_colors(img_frame);
            cv::cvtColor(img_filter, img_filter, cv::COLOR_BGR2GRAY);
            cv::Canny(img_filter, img_edges, 50, 150);
            img_mask = roadLaneDetector.limit_region(img_edges);
            lines = roadLaneDetector.houghLines(img_mask);
            if (lines.size() > 0) {
                separated_lines = roadLaneDetector.separateLine(img_mask, lines);
                lane = roadLaneDetector.regression(separated_lines, img_frame);
                dir = roadLaneDetector.predictDir();
                img_result = roadLaneDetector.drawLine(img_frame, lane, dir);
	    
	    // Convert the image to a ROS image message and publish
                sensor_msgs::ImagePtr img_msg;
                sensor_msgs::ImagePtr img_msg2;
                sensor_msgs::ImagePtr img_msg3;
                std_msgs::Header header;

                header.stamp = ros::Time::now();
                header.frame_id = "lane_detector_node_output";
                img_msg = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, img_result).toImageMsg();
                img_msg2 = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, img_mask).toImageMsg();
                img_msg3 = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, img_edges).toImageMsg();
                pImg_pub_1->publish(img_msg);
                pImg_pub_2->publish(img_msg2);
                pImg_pub_3->publish(img_msg3);

                // Publish the Twist message containing the predicted steering direction
                geometry_msgs::Twist twist_msg;
                twist_msg.linear.x = 0.0;
                twist_msg.linear.y = 0.0;
                twist_msg.linear.z = 0.0;
                twist_msg.angular.x = 0.0;
                twist_msg.angular.y = 0.0;
                if (dir == "left") {
                    twist_msg.angular.z = 1.0;
                }
                else if (dir == "right") {
                    twist_msg.angular.z = -1.0;
                }
                else {
                    twist_msg.angular.z = 0.0;
                }
                twist_pub_.publish(twist_msg);
            }
        }
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
    }

private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    ros::Publisher twist_pub_;
    image_transport::Publisher image_pub_1;
    image_transport::Publisher image_pub_2;
    image_transport::Publisher image_pub_3;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "lane_detector_node");

    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

    LaneDetectorNode node(nh, it);

    while (ros::ok()) {
        ros::spinOnce();
    }

    return 0;
}
