#include <iostream>
#include <map>
#include <vector>

#define OPENCV_470
//ROS용
//#include <ros/ros.h>
//#include <sensor_msgs/Image.h>
//#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#ifdef OPENCV_470
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/imgproc/types_c.h>
#endif // OPENCV_470

#ifdef _DEBUG
#pragma comment(lib,"opencv_world470d.lib")
#else	//RELEASE
#pragma comment(lib,"opencv_world470.lib")
#endif

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
    // object 이미지 로드
    Mat object = imread("object.jpg", IMREAD_GRAYSCALE);

    // scene 이미지 로드
    Mat scene = imread("scene.jpg", IMREAD_GRAYSCALE);

    // ORB 객체 생성
    Ptr<ORB> orb = ORB::create(400, 1.2f, 8, 31, 2, 2, ORB::HARRIS_SCORE, 31, 20);

    // 키포인트 및 디스크립터 계산
    vector<KeyPoint> keypoints_object, keypoints_scene;
    Mat descriptors_object, descriptors_scene;
    orb->detectAndCompute(object, Mat(), keypoints_object, descriptors_object);
    orb->detectAndCompute(scene, Mat(), keypoints_scene, descriptors_scene);

    // 매칭 객체 생성
    BFMatcher matcher(NORM_HAMMING);
    vector<vector<DMatch>> matches;
    matcher.knnMatch(descriptors_object, descriptors_scene, matches, 2); //최근접 이웃 매칭

    // 이웃 거리 비율을 이용하여 유효한 매칭 포인트만 선택
    vector<DMatch> good_matches;
    for (int i = 0; i < matches.size(); i++) {
        //잘못된 매칭이나 노이즈 제거
        if (matches[i][0].distance < 0.75 * matches[i][1].distance) {
            good_matches.push_back(matches[i][0]);
        }
    }

    // good_matches가 일정 개수 이상일 때 object 이미지가 scene 이미지에서 검출됨을 출력
    if (good_matches.size() >= 8) {
        cout << "object.jpg is detected in scene.jpg" << endl;

        // 검출 이미지 출력
        Mat img_matches;
        drawMatches(object, keypoints_object, scene, keypoints_scene,
            good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
            vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

        imshow("Detected Object", img_matches);
        waitKey(0);
    }

    return 0;
}