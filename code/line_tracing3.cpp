//교차점을 구하는 함수
Point IntersectionPoint1(Point p1, Point p2, Point p3, Point p4) { //두 선의 교차점 출력함수
    Point ret;
    //교차되는 점의 x 좌표 계산결과
    ret.x = ((p1.x * p2.y - p1.y * p2.x) * (p3.x - p4.x) - (p1.x - p2.x) * (p3.x * p4.y - p3.y * p4.x)) / ((p1.x - p2.x) * (p3.y - p4.y) - (p1.y - p2.y) * (p3.x - p4.x));

    //교차되는 점의 y 좌표 계산결과
    ret.y = ((p1.x * p2.y - p1.y * p2.x) * (p3.y - p4.y) - (p1.y - p2.y) * (p3.x * p4.y - p3.y * p4.x)) / ((p1.x - p2.x) * (p3.y - p4.y) - (p1.y - p2.y) * (p3.x - p4.x));

    return ret;
}