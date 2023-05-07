#include <ros/ros.h> // ROS 라이브러리를 사용하기 위한 헤더 파일
#include <image_transport/image_transport.h> // 이미지 퍼블리싱 및 구독을 위한 헤더 파일
#include <cv_bridge/cv_bridge.h> // OpenCV와 ROS 이미지 포맷 변환을 위한 헤더 파일
#include <opencv2/highgui/highgui.hpp> // OpenCV GUI를 위한 헤더 파일
#include <opencv2/imgproc/imgproc.hpp> // OpenCV 영상 처리를 위한 헤더 파일

#define ArraySize 100
#define PI 3.141592


using namespace cv;
using namespace std;

static const string OPENCV_WINDOW = "Image window"; // OpenCV 창의 이름
Point* aPoint1 = new Point[ArraySize], * aPoint2 = new Point[ArraySize], * bPoint1 = new Point[ArraySize], * bPoint2 = new Point[ArraySize];
int aIndex = 0, bIndex = 0;

class LaneDetector
{
    ros::NodeHandle nh_; // ROS 노드 핸들러
    image_transport::ImageTransport it_; // 이미지 전송 클래스
    image_transport::Subscriber image_sub_; // 이미지 수신자
    image_transport::Publisher image_pub_; // 이미지 발행자

public:
    LaneDetector()
        : it_(nh_) // 생성자 초기화 목록을 사용하여 노드 핸들러 객체와 이미지 전송 객체 초기화
    {
        // Subscribe to input video feed and publish output video feed
        image_sub_ = it_.subscribe("/jetbot_camera/raw", 1, // "/camera/image_raw" 토픽으로부터 영상 수신
            &LaneDetector::imageCb, this); // 수신 콜백 함수 등록
        image_pub_ = it_.advertise("/lane_detection/output_video", 1); // "/lane_detection/output_video" 토픽으로 이미지 발행

        cv::namedWindow(OPENCV_WINDOW); // OpenCV 창 생성
    }

    ~LaneDetector()

        cv::destroyWindow(OPENCV_WINDOW); // 소멸자에서 OpenCV 창 제거
}

Point IntersectionPoint1(Point p1, Point p2, Point p3, Point p4) { //두 선의 교차점 출력함수
    Point ret;
    //교차되는 점의 x 좌표 계산결과
    ret.x = ((p1.x * p2.y - p1.y * p2.x) * (p3.x - p4.x) - (p1.x - p2.x) * (p3.x * p4.y - p3.y * p4.x)) / ((p1.x - p2.x) * (p3.y - p4.y) - (p1.y - p2.y) * (p3.x - p4.x));

    //교차되는 점의 y 좌표 계산결과
    ret.y = ((p1.x * p2.y - p1.y * p2.x) * (p3.y - p4.y) - (p1.y - p2.y) * (p3.x * p4.y - p3.y * p4.x)) / ((p1.x - p2.x) * (p3.y - p4.y) - (p1.y - p2.y) * (p3.x - p4.x));

    return ret;
}

float get_radian(float Val) //각도 계산하여 반환
{
    return PI * Val / 180;  //Val 값을 라디안 값으로 변환
}

Mat drivingArea(Mat image, Point pt1, Point pt2, Point pt3, Point pt4)
{
    Point pty = IntersectionPoint1(pt2, pt3, pt1, pt4) + Point(0, 20); //평균 직선의 중앙점에 약간 아래의 좌표를 추출
    Point a1 = pty + Point(200, 0); //x축과 평행한 직선을 만들기 위해 값을 넣음
    Point a2 = pty + Point(-200, 0);
    Point Pt23Fix = IntersectionPoint1(a1, a2, pt1, pt4);  //이전 평균 직선과 사다리꼴 형태를 만들기 위해 구한 직선의 교차점을 넣음
    Point Pt14Fix = IntersectionPoint1(pt2, pt3, a1, a2);
    Point points[4] = { pt3,pt4,Pt23Fix ,Pt14Fix }; // 각 점의 좌표를 함수에 전달하기위해 저장
    const Point* ppt[1] = { points };

    int npt[] = { 4 };
    Mat result = image;
    Mat img_mask(image.size(), CV_8UC3, cv::Scalar(255, 255, 255)); //하얀색바탕 생성
    fillPoly(img_mask, ppt, npt, 1, Scalar(0, 255, 0), LINE_8); //초록색으로 사다리꼴 색칠
    addWeighted(result, 0.8, img_mask, 0.2, 0, result); //  원래 그림과 사다리꼴 영역을 이용해 가중치를 두어 그림을 합침
    line(result, Pt14Fix, pt3, Scalar(255, 0, 0), 2, LINE_8); // 사다리꼴의 옆부분에 빨간 선을 그어줌
    line(result, Pt23Fix, pt4, Scalar(0, 0, 255), 2, LINE_8);

    return result; //나온 결과 그림 반환
}

void imageCb(const sensor_msgs::ImageConstPtr& msg) // 이미지 수신 콜백 함수
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); // ROS 메시지를 OpenCV 형식으로 변환
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what()); // 예외 처리
        return;
    }

    cv::Mat src_img = cv_ptr->image; // OpenCV 형식 이미지로부터 원본 이미지 획득

    //ROI영역 생성용 포인트
    Point pts[4];
    pts[0] = Point(0.48 * src_img.cols, 0.6 * src_img.rows);
    pts[1] = Point(0.6 * src_img.cols, 0.6 * src_img.rows);
    pts[2] = Point(0.85 * src_img.cols, 0.9 * src_img.rows);
    pts[3] = Point(0.2 * src_img.cols, 0.9 * src_img.rows);

    // Convert to grayscale
    cv::Mat gray_img;
    cv::cvtColor(src_img, gray_img, cv::COLOR_BGR2GRAY); // 원본 이미지를 그레이스케일 영상으로 변환

    // Apply Gaussian blur to smooth the image and reduce noise
    cv::Mat blurred_img, hsv;
    cvtColor(src_img, hsv, COLOR_BGR2HSV);
    cv::GaussianBlur(gray_img, blurred_img, cv::Size(5, 5), 0); // 가우시안 블러를 적용하여 이미지를 부드럽게 만듦

    Mat mask = Mat::zeros(src_img.rows, src_img.cols, CV_8UC1);
    fillConvexPoly(mask, pts, 4, Scalar(255, 0, 0));

    Mat binary, binary_y, binary_w;
    inRange(blurred_img, Scalar(20, 100, 100), Scalar(30, 255, 255), binary_y); // 노란색 마스크
    bitwise_and(binary_y, mask, binary_y); // ROI 마스크 적용
    inRange(blurred_img, Scalar(0, 0, 200), Scalar(255, 30, 255), binary_w); // 흰색 마스크
    bitwise_and(binary_w, mask, binary_w); // ROI 마스크 적용	
    // 마스킹된 이미지 생성
    bitwise_or(binary_w, binary_y, binary);
    Mat img_masked;
    bitwise_and(gray_img, binary, img_masked);

    // Detect lines using Hough transform
    vector<Vec2f> lines;
    HoughLines(img_masked, lines, 1, CV_PI / 180, 30, 30, 100);
    Mat HoughLineDetected_img(img_masked.rows, img_masked.cols, CV_8U, cv::Scalar(255));
    vector<Vec2f>::const_iterator it = lines.begin();

    while (it != lines.end()) {
        float rho = (*it)[0];   // 첫 번째 요소는 rho 거리
        float theta = (*it)[1]; // 두 번째 요소는 델타 각도
        if (theta < get_radian(90) && theta >get_radian(35)) { // 각도 우측 범위 35 ~ 85 각도 선에서 인지
            Point pt1(rho / cos(theta), 0); // 첫 우측각에서 해당 선의 교차점   
            aPoint1[(aIndex) % ArraySize] = pt1; //마치 Queue가 돌아가듯 사이즈 안에서만 값을 덧씌우며 채움
            Point pt2((rho - HoughLineDetected_img.rows * sin(theta)) / cos(theta), HoughLineDetected_img.rows);
            // 마지막 우측각에서 해당 선의 교차점
            aPoint2[(aIndex++) % ArraySize] = pt2; // 다음 인덱스 값으로 넘어가며 값을 넣음
            //line(img, pt1, pt2, cv::Scalar(255), 2); // 모든 직선 그리기
        }
        else if (theta<get_radian(145) && theta>get_radian(90)) { // 각도 좌측 범위 95 ~ 145 각도 선에서 인지
            cv::Point pt1(0, rho / sin(theta)); // 첫 좌측각에서 해당 선의 교차점  
            bPoint1[(bIndex) % ArraySize] = pt1; //마치 Queue가 돌아가듯 사이즈 안에서만 값을 덧씌우며 채움
            Point pt2(HoughLineDetected_img.cols, (rho - HoughLineDetected_img.cols * cos(theta)) / sin(theta));
            // 마지막 좌측각에서 해당 선의 교차점
            bPoint2[(bIndex++) % ArraySize] = pt2; // 다음 인덱스 값으로 넘어가며 값을 넣음
            //line(img, pt1, pt2, cv::Scalar(255), 2); // 모든 직선 그리기
        }
        ++it; //반복자 증가
    }
    //--    평균 직선 추정 ---//
    Point aP1S(0, 0), aP2S(0, 0), bP1S(0, 0), bP2S(0, 0); //각 끝점의 평균값을 담기 위한 점
    int a1cnt = 0, a2cnt = 0, b1cnt = 0, b2cnt = 0;
    for (int i = 0; i < ArraySize; i++) {//배열에 저장된 모든 점의 좌표 합을 구함
        aP1S += aPoint1[i];
        aP2S += aPoint2[i];
        bP1S += bPoint1[i];
        bP2S += bPoint2[i];
        if (aPoint1[i] != Point(0, 0)) {
            a1cnt++;
        }
        if (aPoint2[i] != Point(0, 0)) {
            a2cnt++;
        }
        if (bPoint1[i] != Point(0, 0)) {
            b1cnt++;
        }
        if (bPoint2[i] != Point(0, 0)) {
            b2cnt++;
        }
    }
    aP1S /= a1cnt;//배열의 저장된 모든 점의 평균값을 구함
    aP2S /= a2cnt;
    bP1S /= b1cnt;
    bP2S /= b2cnt;

    Mat result = drivingArea(img, bP1S, aP1S, aP2S, bP2S);

    sensor_msgs::ImagePtr output_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", result).toImageMsg(); // output_img를 ROS Image 메시지로 변환
    image_pub_.publish(output_msg); // 변환된 이미지를 topic으로 publish

    cv::imshow(OPENCV_WINDOW, result); // 이미지 출력
    cv::waitKey(1); // imshow에서의 입력을 기다리기 위한 대기 함수 (1ms)
}
};

int main(int argc, char** argv)
{
    // "lane_detector"는 이 노드의 이름으로 사용된다.
    ros::init(argc, argv, "lane_detector");
    // LaneDetector 클래스의 인스턴스를 만든다.
    LaneDetector ld;
    // ROS 시스템에서 메시지를 받고, 발행하며, 콜백을 호출한다.
    ros::spin();
    // 프로그램이 정상적으로 종료됨을 나타내는 0을 반환한다.
    return 0;
}