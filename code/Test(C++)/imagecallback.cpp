//이미지 콜백 함수 - 이미지 메시지를 받을 때마다 실행
void imageCallback(const sensor_msgs::ImageConstPtr& msg, ros::Publisher& pub)
{
	try
	{
		// ROS 메시지를 OpenCV 이미지로 변환
		Mat img = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;

		// 이미지 전처리 (색상공간 변환, 가우시안 블러 적용)
		Mat hsv, blurred;
		cvtColor(img, hsv, COLOR_BGR2HSV);
		GaussianBlur(hsv, blurred, Size(5, 5), 0);

		// ROI 영역 설정
		Mat mask = Mat::zeros(img.size(), CV_8UC1);
		Point roi_points[4] = { Point(0, img.rows), Point(img.cols, img.rows), Point(img.cols * 3 / 4, img.rows / 2),Point(img.cols / 4, img.rows / 2) };
		fillConvexPoly(mask, roi_points, 4, Scalar(255, 255, 255));

		// ROI 영역을 제외한 부분 제거
		Mat masked;
		bitwise_and(blurred, mask, masked);

		// 이진화
		Mat binary;
		inRange(masked, Scalar(0, 0, 200), Scalar(255, 100, 255), binary);

		// 주행 방향 구하기
		double direction;
		get_steering_direction(binary, direction);

		// 주행 방향에 따른 터틀봇 제어값 설정
		geometry_msgs::Twist twist;
		twist.linear.x = 0.15;   // 선속도 = 로봇 속도
		twist.angular.z = direction * 0.01;  // 각속도 = 로봇 회전 각도

		// 터틀봇 제어값 전송
		pub.publish(twist);

		// 디버그용 이미지 표시
		imshow("binary", binary);
		waitKey(1);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
}