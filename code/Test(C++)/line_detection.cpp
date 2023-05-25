
#include "RoadLaneDetector.h"


RoadLaneDetector::RoadLaneDetector()
	: img_size(0.)
	, img_center(0.)
	, left_m(0.)
	, right_m(0.)
	, left_b(Point(0, 0))
	, right_b(Point(0, 0))
	, left_detect(false)
	, right_detect(false)
	, poly_bottom_width(0.85)
	, poly_top_width(0.07)
	, poly_height(0.3)
	, predict_x(0.)
{

}

RoadLaneDetector::~RoadLaneDetector()
{
}

Mat RoadLaneDetector::extract_colors(Mat img_frame) {
	/*
		흰색/노란색 색상의 범위를 정해 해당되는 차선을 필터링한다.
	*/
	Mat output;

#ifdef _DEBUG
	Mat img_hsv;
	Mat white_mask, white_image;
	Mat yellow_mask, yellow_image;
	img_frame.copyTo(output);
#else	//RELEASE
	UMat img_hsv;
	UMat white_mask, white_image;
	UMat yellow_mask, yellow_image;
	img_frame.copyTo(output);
#endif

#ifdef _DEBUG
	img_frame.copyTo(drawDebug);
#endif // _DEBUG

	//차선 색깔 범위 
	Scalar lower_white = Scalar(120, 120, 120); //흰색 차선 (RGB)
	Scalar upper_white = Scalar(255, 255, 255);
	Scalar lower_yellow = Scalar(10, 100, 100); //노란색 차선 (HSV)
	Scalar upper_yellow = Scalar(40, 255, 255);

	

	cvtColor(output, img_hsv, COLOR_BGR2HSV);

	//흰색 필터링
	inRange(output, lower_white, upper_white, white_mask);
	bitwise_and(output, output, white_image, white_mask);


	//노란색 필터링
	inRange(img_hsv, lower_yellow, upper_yellow, yellow_mask);
	bitwise_and(output, output, yellow_image, yellow_mask);

	//두 영상을 합친다.
	addWeighted(white_image, 1.0, yellow_image, 1.0, 0.0, output);
	return output;
}


Mat RoadLaneDetector::limit_region(Mat img_edges) {
	/*
		관심 영역의 가장자리만 감지되도록 마스킹한다.
		관심 영역의 가장자리만 표시되는 이진 영상을 반환한다.
	*/
	int width = img_edges.cols;
	int height = img_edges.rows;

	Mat output;
	Mat mask = Mat::zeros(height, width, CV_8UC1);

	//관심 영역 정점 계산
	const int poly_pts = 4;
	Point points[4]{
		Point((width * (1 - poly_bottom_width)) / 2, height),
		Point((width * (1 - poly_top_width)) / 2, height - height * poly_height),
		Point(width - (width * (1 - poly_top_width)) / 2, height - height * poly_height),
		Point(width - (width * (1 - poly_bottom_width)) / 2, height)
	};

	//정점으로 정의된 다각형 내부의 색상을 채워 그린다.
	fillConvexPoly(mask, points, poly_pts, Scalar(255, 0, 0));

	//결과를 얻기 위해 edges 이미지와 mask를 곱한다.
	bitwise_and(img_edges, mask, output);
	return output;
}

vector<Vec4i> RoadLaneDetector::houghLines(Mat img_mask) {
	/*
		관심영역으로 마스킹 된 이미지에서 모든 선을 추출하여 반환
	*/
	vector<Vec4i> line;

	//확률적용 허프변환 직선 검출 함수 
	HoughLinesP(img_mask, line, 1, CV_PI / 180, 20, 10, 20);
	return line;
}

vector<vector<Vec4i>> RoadLaneDetector::separateLine(Mat img_edges, vector<Vec4i> lines) {
	/*
		검출된 모든 허프변환 직선들을 기울기 별로 정렬한다.
		선을 기울기와 대략적인 위치에 따라 좌우로 분류한다.
	*/

	vector<vector<Vec4i>> output(2);
	Point p1, p2;
	vector<double> slopes;
	vector<Vec4i> final_lines, left_lines, right_lines;
	double slope_thresh = 0.3;

	double line_thresh = 10;
	//검출된 직선들의 기울기를 계산
	for (int i = 0; i < lines.size(); i++) {
		Vec4i line = lines[i];
		p1 = Point(line[0], line[1]);
		p2 = Point(line[2], line[3]);
		double dist = sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
		if (dist < line_thresh) continue;
#ifdef _DEBUG
		if (!drawDebug.empty())
			cv::line(drawDebug, p1, p2, Scalar(0, 0, 255), 1, LINE_AA);
#endif // _DEBUG

		double slope;
		if (p2.x - p1.x == 0)  //코너 일 경우
			slope = 999.0;
		else
			slope = (p2.y - p1.y) / (double)(p2.x - p1.x);

		//기울기가 너무 수평인 선은 제외
		if (abs(slope) > slope_thresh) {
			slopes.push_back(slope);
			final_lines.push_back(line);
		}
	}

	//선들을 좌우 선으로 분류
	img_center = (double)((img_edges.cols / 2 - 1));

	for (int i = 0; i < final_lines.size(); i++) {
		p1 = Point(final_lines[i][0], final_lines[i][1]);
		p2 = Point(final_lines[i][2], final_lines[i][3]);

		if (slopes[i] > 0 && p1.x > img_center && p2.x > img_center) {
			right_detect = true;
			right_lines.push_back(final_lines[i]);
		}
		else if (slopes[i] < 0 && p1.x < img_center && p2.x < img_center) {
			left_detect = true;
			left_lines.push_back(final_lines[i]);
		}
	}

	output[0] = right_lines;
	output[1] = left_lines;
	return output;
}

vector<Point> RoadLaneDetector::regression(vector<vector<Vec4i>> separatedLines, Mat img_input) {
	/*
		선형 회귀를 통해 좌우 차선 각각의 가장 적합한 선을 찾는다.
	*/
	vector<Point> output(4);
	Point p1, p2, p3, p4;
	Vec4d left_line, right_line;
	vector<Point> left_points, right_points;

	if (right_detect) {
		for (auto i : separatedLines[0]) {
			p1 = Point(i[0], i[1]);
			p2 = Point(i[2], i[3]);

			right_points.push_back(p1);
			right_points.push_back(p2);
		}

		if (right_points.size() > 0) {
			//주어진 contour에 최적화된 직선 추출
			fitLine(right_points, right_line, DIST_L2, 0, 0.01, 0.01);

			right_m = right_line[1] / right_line[0];  //기울기
			right_b = Point(right_line[2], right_line[3]);
		}
	}

	if (left_detect) {
		for (auto j : separatedLines[1]) {
			p3 = Point(j[0], j[1]);
			p4 = Point(j[2], j[3]);

			left_points.push_back(p3);
			left_points.push_back(p4);
		}

		if (left_points.size() > 0) {
			//주어진 contour에 최적화된 직선 추출
			fitLine(left_points, left_line, DIST_L2, 0, 0.01, 0.01);

			left_m = left_line[1] / left_line[0];  //기울기
			left_b = Point(left_line[2], left_line[3]);
		}
	}

	//좌우 선 각각의 두 점을 계산한다.
	//y = m*x + b  --> x = (y-b) / m
	int y1 = img_input.rows - 1;
	int y2 = img_input.rows / 2 - 1;// 470;

	double right_x1 = ((y1 - right_b.y) / right_m) + right_b.x;
	double right_x2 = ((y2 - right_b.y) / right_m) + right_b.x;

	double left_x1 = ((y1 - left_b.y) / left_m) + left_b.x;
	double left_x2 = ((y2 - left_b.y) / left_m) + left_b.x;

	output[0] = Point(right_x1, y1);
	output[1] = Point(right_x2, y2);
	output[2] = Point(left_x1, y1);
	output[3] = Point(left_x2, y2);

	return output;
}

string RoadLaneDetector::predictDir() {
	/*
		두 차선이 교차하는 지점(사라지는 점)이 중심점으로부터
		왼쪽에 있는지 오른쪽에 있는지로 진행방향을 예측한다.
	*/

	string output = "Straight";
	double x = 0.0, threshold = 10;

	//두 차선이 교차하는 지점 계산
	x = (double)(((right_m * right_b.x) - (left_m * left_b.x) - right_b.y + left_b.y) / (right_m - left_m));

	if (x >= (img_center - threshold) && x <= (img_center + threshold))
		output = "Straight";
	else if (x > img_center + threshold)
		output = "Right Turn";
	else if (x < img_center - threshold)
		output = "Left Turn";

	predict_x = x;
	return output;
}

Mat RoadLaneDetector::drawLine(Mat img_input, vector<Point> lane, string dir) {
	/*
		좌우 차선을 경계로 하는 내부 다각형을 투명하게 색을 채운다.
		예측 진행 방향 텍스트를 영상에 출력한다.
		좌우 차선을 영상에 선으로 그린다.
	*/

	vector<Point> poly_points;
	Mat output;
	img_input.copyTo(output);


	poly_points.push_back(lane[2]);
	poly_points.push_back(lane[0]);
	poly_points.push_back(lane[1]);
	poly_points.push_back(lane[3]);
	fillConvexPoly(output, poly_points, Scalar(0, 230, 30), LINE_AA, 0);  //다각형 색 채우기
	addWeighted(output, 0.3, img_input, 0.7, 0, img_input);  //영상 합하기

	//예측 진행 방향 텍스트를 영상에 출력
	putText(img_input, dir, Point(520, 100), FONT_HERSHEY_PLAIN, 3, Scalar(255, 255, 255), 3, LINE_AA);

	//좌우 차선 선 그리기
	line(img_input, lane[0], lane[1], Scalar(0, 255, 255), 5, LINE_AA);
	line(img_input, lane[2], lane[3], Scalar(0, 255, 255), 5, LINE_AA);

	

	//predict horizontal direction
	{
		cv::Point ptCross(0, 0);
		IntersectPoint(lane[0], lane[1], lane[2], lane[3], &ptCross);
		//cv::circle(img_input, ptCross, 20, Scalar(0, 0, 255), 1, LINE_AA);
		drawMarker(img_input, ptCross, Scalar(0, 0, 255), MARKER_TRIANGLE_DOWN);
		//
		//y = a*x+b
		Point p1, p2;
		Vec4d axis_line;
		vector<Point> axis_points;
		axis_points.push_back(Point(img_input.cols / 2 - 1, img_input.rows - 1));
		axis_points.push_back(ptCross);
		fitLine(axis_points, axis_line, DIST_L2, 0, 0.01, 0.01);
		double m = axis_line[1] / axis_line[0];  //기울기
		Point b = cv::Point(axis_line[2], axis_line[3]);

		int y1 = img_input.rows - 1;
		int y2 = 50;// 470;
		double x1 = ((y1 - b.y) / m) + b.x;
		double x2 = ((y2 - b.y) / m) + b.x;
		cv::arrowedLine(img_input, Point(x1, y1), Point(x2, y2), Scalar(255, 0, 255), 2);
		

		cv::arrowedLine(img_input, Point(img_input.cols / 2 - 1, ptCross.y), ptCross, Scalar(0, 255, 0), 5);

		y1 = 0;
		y2 = img_input.rows - 1; 
		x1 = ((y1 - b.y) / m) + b.x;
		x2 = ((y2 - b.y) / m) + b.x;
		DrawDashedLine(img_input, Point(x1, y1), Point(x2, y2), Scalar(0, 255, 255), 2, "dotted", 10);

		
	}
	

	//reference center cross line
	DrawDashedLine(img_input, Point(img_input.cols / 2 - 1, 0), Point(img_input.cols / 2 - 1, img_input.rows - 1), Scalar(0, 0, 255), 1, "", 5);
	DrawDashedLine(img_input, Point(0, img_input.rows / 2 - 1), Point(img_input.cols - 1, img_input.rows / 2 - 1), Scalar(0, 0, 255), 1, "", 5);

	return img_input;
}
void RoadLaneDetector::DrawDashedLine(cv::Mat& img, cv::Point pt1, cv::Point pt2,
	cv::Scalar color, int thickness, std::string style,
	int gap) {
	float dx = pt1.x - pt2.x;
	float dy = pt1.y - pt2.y;
	float dist = std::hypot(dx, dy);

	std::vector<cv::Point> pts;
	for (int i = 0; i < dist; i += gap) {
		float r = static_cast<float>(i / dist);
		int x = static_cast<int>((pt1.x * (1.0 - r) + pt2.x * r) + .5);
		int y = static_cast<int>((pt1.y * (1.0 - r) + pt2.y * r) + .5);
		pts.emplace_back(x, y);
	}

	int pts_size = pts.size();

	if (style == "dotted") {
		for (int i = 0; i < pts_size; ++i) {
			cv::circle(img, pts[i], thickness, color, FILLED);
		}
	}
	else {
		cv::Point s = pts[0];
		cv::Point e = pts[0];

		for (int i = 0; i < pts_size; ++i) {
			s = e;
			e = pts[i];
			if (i % 2 == 1) {
				cv::line(img, s, e, color, thickness);
			}
		}
	}
}

bool RoadLaneDetector::IntersectPoint(const Point& pt11, const Point& pt12,
	const Point& pt21, const Point& pt22, Point* ptCross)
{
	double t;
	double s;
	double under = (pt22.y - pt21.y) * (pt12.x - pt11.x) - (pt22.x - pt21.x) * (pt12.y - pt11.y);
	if (under == 0) return false;

	double _t = (pt22.x - pt21.x) * (pt11.y - pt21.y) - (pt22.y - pt21.y) * (pt11.x - pt21.x);
	double _s = (pt12.x - pt11.x) * (pt11.y - pt21.y) - (pt12.y - pt11.y) * (pt11.x - pt21.x);

	t = _t / under;
	s = _s / under;

	if (t < 0.0 || t>1.0 || s < 0.0 || s>1.0) return false;
	if (_t == 0 && _s == 0) return false;

	ptCross->x = pt11.x + t * (double)(pt12.x - pt11.x);
	ptCross->y = pt11.y + t * (double)(pt12.y - pt11.y);

	return true;
}
