
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

ros::Publisher pub_image;

void imageCallback(const sensor_msgs::Image::ConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    Mat image = cv_ptr->image;
    if (image.empty()) {
        ROS_WARN("Could not open or find the image");
        return;
    }

    // ROI 영역을 설정하기 위한 4개의 포인트 설정
    Point pts[4];
    pts[0] = Point(0.2 * image.cols, 0.4 * image.rows);
    pts[1] = Point(0.7 * image.cols, 0.4 * image.rows);
    pts[2] = Point(0.7 * image.cols, 0.7 * image.rows);
    pts[3] = Point(0.2 * image.cols, 0.7 * image.rows);

    // HSV 이미지로 변환 -> 빨간색을 추려내기 위함
    Mat hsv_image;
    cvtColor(image, hsv_image, COLOR_BGR2HSV);

    // 마스크를 생성하여 빨간색만 검출 및 ROI 영역 이미지를 만들어 마스크에 적용 -> 정면의 차단기가 보이는 부분만 검출
    Mat mask;
    inRange(hsv_image, Scalar(0, 70, 50), Scalar(10, 255, 255), mask);
    Mat mask_roi = Mat::zeros(image.rows, image.cols, CV_8UC1);
    fillConvexPoly(mask_roi, pts, 4, Scalar(255, 255, 255));
    bitwise_and(mask, mask_roi, mask);

    // 빨간색이 구성된 사각형을 구하고 그 사각형의 중심을 centers 벡터에 저장
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

// centers 벡터에 저장된 중심점들을 이용하여 가장 x좌표 값이 낮은 점과 높은 점 두 점을 구하여 선으로 이은 후 그 각도를 통해 차단기의 상태(open/closed)를 판단하여 이미지에 puttext
if (centers.size() >= 2) {
    sort(centers.begin(), centers.end(), [](Point a, Point b) {
        return a.x < b.x;
    });

    Point p1 = centers.front();
    Point p2 = centers.back();
    line(output, p1, p2, Scalar(255, 0, 0), 2);

    double angle = atan2(p2.y - p1.y, p2.x - p1.x) * 180 / CV_PI;
    string status;
    if (angle >= -10 && angle <= 10) {
        status = "closed";
    }
    else if (angle >= 80 && angle <= 100) {
        status = "open";
    }
    else {
        status = "unknown";
    }
    putText(output, status, Point(50, 50), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 255, 255), 2);
}

// 결과 이미지 publish
sensor_msgs::ImagePtr output_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", output).toImageMsg();
pub_image.publish(output_msg);
}

int main(int argc, char **argv) {
ros::init(argc, argv, "blockade_detector");
ros::NodeHandle nh;
// 이미지 토픽 subscribe
ros::Subscriber sub_image = nh.subscribe("/camera/image", 1, imageCallback);

// 결과 이미지 토픽 publish
pub_image = nh.advertise<sensor_msgs::Image>("/level_cross/image", 1);

ros::spin();

return 0;
}
