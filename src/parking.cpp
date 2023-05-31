#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>

image_transport::Publisher* pImage_pub=NULL;
image_transport::Publisher* pImage_pub2=NULL;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        // ROS 메시지를 OpenCV 이미지로 변환
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat image = cv_ptr->image;
 // HSV로 변환
        cv::Mat hsv_image;
        cv::cvtColor(image, hsv_image, cv::COLOR_BGR2HSV);

        // 주차선의 HSV 범위 설정
        cv::Scalar lower_white = cv::Scalar(0, 0, 200);  // 하얀색 범위의 하한값 (HSV)
        cv::Scalar upper_white = cv::Scalar(180, 30, 255);  // 하얀색 범위의 상한값 (HSV)

        // 주차선의 색상에 대한 마스크 생성
        cv::Mat white_mask;
        cv::inRange(hsv_image, lower_white, upper_white, white_mask);

        // ROI 설정
        int img_width = image.cols;
        int img_height = image.rows;
        cv::Point pt1(img_width * 0.2, img_height);
        cv::Point pt2(img_width * 0.6, img_height);
        cv::Point pt3(img_width * 0.4, img_height * 0.4);
        cv::Point pt4(img_width * 0.8, img_height * 0.4);
        cv::Mat roi_mask = cv::Mat::zeros(image.size(), CV_8UC1);
        cv::Point pts[] = {pt1, pt2, pt4, pt3};
        cv::fillConvexPoly(roi_mask, pts, 4, cv::Scalar(255));

        // 주차선 영역에 대한 ROI 마스크 생성
        cv::Mat roi_white_mask;
        white_mask.copyTo(roi_white_mask, roi_mask);

        // 이미지 처리 및 주차선 인식 코드 작성
        cv::Mat edges;
        cv::Canny(roi_white_mask, edges, 50, 150);

        // 허프 변환을 사용하여 선 감지
        std::vector<cv::Vec4i> lines;
        cv::HoughLinesP(edges, lines, 1, CV_PI / 180, 50, 30, 10);

        // 주차선으로 판단하기 위한 필터링
        cv::Mat result = image.clone();
        for (const cv::Vec4i& line : lines)
        {
            cv::Point p1(line[0], line[1]);
            cv::Point p2(line[2], line[3]);
            // 점선으로 된 부분을 필터링하는 조건 설정
            if (cv::norm(p1 - p2) > 20 && std::abs(p1.y - p2.y) < 5)  // 예시: 선의 길이가 20 이상이고, y좌표 차이가 5 이하인 경우
            {
                // 주차선으로 판단하여 선 그리기 등의 추가 처리 수행
                cv::line(result, p1, p2, cv::Scalar(0, 0, 255), 2);
            }
        }

        // 주차선이 인식되었을 때 텍스트 출력
        cv::putText(image, "Parking", cv::Point(50, 50), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);

        // 결과 이미지 출력

		sensor_msgs::ImagePtr img_msg;
		sensor_msgs::ImagePtr img_msg2;
               std_msgs::Header header;

                header.stamp = ros::Time::now();
                header.frame_id = "parking_node_output";
                img_msg = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, result).toImageMsg();
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
    ros::init(argc, argv, "parking_detection_node");
    ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	image_transport::Publisher image_pub;
	image_transport::Publisher image_pub2;
	pImage_pub = &image_pub;
	pImage_pub2 = &image_pub2;
    // /camera/image 토픽에서 이미지를 받는 Subscriber 생성
    ros::Subscriber sub = nh.subscribe("/camera/image", 1, imageCallback);
    image_pub = it.advertise("/parking/output",1);
	image_pub2 = it.advertise("/parking/edges",1);
	while(ros::ok()){
        ros::spinOnce();
    }
    return 0;
}
