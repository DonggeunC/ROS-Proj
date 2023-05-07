//점 4개를 이용하여 사각형 그리는 함수
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