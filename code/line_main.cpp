//메인함수
int main(void) {

    Mat img = imread("image.png", IMREAD_COLOR);

    //ROI영역 생성용 포인트
    Point pts[4];
    pts[0] = Point(0.48 * img.cols, 0.6 * img.rows);
    pts[1] = Point(0.6 * img.cols, 0.6 * img.rows);
    pts[2] = Point(0.85 * img.cols, 0.9 * img.rows);
    pts[3] = Point(0.2 * img.cols, 0.9 * img.rows);

    Mat img_gray = img.clone();
    cvtColor(img_gray, img_gray, COLOR_BGR2GRAY);
    // 이미지 전처리 (색상공간 변환, 가우시안 블러 적용)
    Mat hsv, blurred;
    cvtColor(img, hsv, COLOR_BGR2HSV);
    GaussianBlur(hsv, blurred, Size(5, 5), 0);

    // 노란색 차선과 흰색 차선을 합쳐서 마스킹 결과 생성
    Mat mask = Mat::zeros(img.rows, img.cols, CV_8UC1);
    fillConvexPoly(mask, pts, 4, Scalar(255, 0, 0));

    Mat binary, binary_y, binary_w;
    inRange(blurred, Scalar(20, 100, 100), Scalar(30, 255, 255), binary_y); // 노란색 마스크
    bitwise_and(binary_y, mask, binary_y); // ROI 마스크 적용
    inRange(blurred, Scalar(0, 0, 200), Scalar(255, 30, 255), binary_w); // 흰색 마스크
    bitwise_and(binary_w, mask, binary_w); // ROI 마스크 적용
    // 마스킹된 이미지 생성
    bitwise_or(binary_w, binary_y, binary);
    Mat img_masked;
    bitwise_and(img_gray, binary, img_masked);

    vector<Vec2f> lines;
    HoughLines(img_masked, lines, 1, CV_PI / 180, 30, 30, 100);
    Mat HoughLineDetected_img(img_masked.rows, img_masked.cols, CV_8U, cv::Scalar(255));
    std::vector<cv::Vec2f>::const_iterator it = lines.begin();

    while (it != lines.end()) { // it(iterator)반복자
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
    //line(img, aP1S, aP2S, cv::Scalar(0,0,255), 2); // 추정된 평균 점으로 직선 그리기
    //line(img, bP1S, bP2S, cv::Scalar(0,0,255), 2); //추정된 평균 점으로 직선 그리기

    Mat result = drivingArea(img, bP1S, aP1S, aP2S, bP2S);

    //ROS용
    //ros::init(argc, argv, "image_publisher");
    //// 이미지를 발행할 ROS 노드 핸들 생성
    //ros::NodeHandle nh;
    //// 발행할 이미지 파일 경로
    //std::string filename = "path/to/image.png";
    //// OpenCV를 사용하여 이미지 파일 로드
    //cv::Mat image = cv::imread(filename, cv::IMREAD_COLOR);
    //// cv_bridge를 사용하여 OpenCV 이미지를 ROS 메시지로 변환
    //sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    //// 이미지 발행을 위한 ROS publisher 생성
    //ros::Publisher pub = nh.advertise<sensor_msgs::Image>("image_topic", 1);
    //// 발행할 이미지 루프
    //ros::Rate loop_rate(10);
    //while (ros::ok()){
    //    // 이미지 발행
    //    pub.publish(msg);
    //    // 루프 속도 유지
    //    loop_rate.sleep();
    //}

    return 0;
}