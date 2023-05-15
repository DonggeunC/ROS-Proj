#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/xfeatures2d.hpp>

using namespace cv;
using namespace std;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        // OpenCV bridge를 이용하여 ROS Image 메시지를 Mat 객체로 변환
        cv_bridge::CvImageConstPtr cv_ptr;
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
        Mat scene = cv_ptr->image;

        // object 이미지 로드
        Mat object_park = imread("../file/parking.png", IMREAD_GRAYSCALE);
        Mat object_cons = imread("../file/construction.png", IMREAD_GRAYSCALE);
        Mat object_left = imread("../file/left.png", IMREAD_GRAYSCALE);
        
        // ORB 객체 생성
        Ptr<SIFT> sift = SIFT::create(400, 1.2f, 8, 31, 2, 2, ORB::HARRIS_SCORE, 31, 20);

        // object 이미지를 저장할 vector
        vector<Mat> object_vec = {object_park, object_cons, object_left};
        // 매칭 결과를 저장할 vector
        vector<int> good_matches_vec = {0, 0, 0};

        // 각각의 object 이미지에 대해 매칭 수행
        for (int i = 0; i < object_vec.size(); i++) {
            Mat object = object_vec[i];
            // 키포인트 및 디스크립터 계산
            vector<KeyPoint> keypoints_object, keypoints_scene;
            Mat descriptors_object, descriptors_scene;
            sift->detectAndCompute(object, Mat(), keypoints_object, descriptors_object);
            sift->detectAndCompute(scene, Mat(), keypoints_scene, descriptors_scene);

            // 매칭 객체 생성
            BFMatcher matcher(NORM_HAMMING);
            vector<vector<DMatch>> matches;
            matcher.knnMatch(descriptors_object, descriptors_scene, matches, 2); //최근접 이웃 매칭

            // 이웃 거리 비율을 이용하여 유효한 매칭 포인트만 선택
            vector<DMatch> good_matches;
            for (int j = 0; j < matches.size(); j++) {
                //잘못된 매칭이나 노이즈 제거
                if (matches[j][0].distance < 0.75 * matches[j][1].distance) {
                    good_matches.push_back(matches[j][0]);
                }
            }

            // 매칭 결과 저장
            good_matches_vec[i] = good_matches.size();
        }

        // good_matches가 일정 개수 이상일 때 object 이미지가 scene 이미지에서 검출됨을 출력
        int max_idx = distance(good_matches_vec.begin(), max_element(good_matches_vec.begin(), good_matches_vec.end()));
        if (good_matches_vec[max_idx] >= 8) {
            cout << "object.jpg is detected in scene image." << endl;

            // 매칭 수행된 object 이미지 선택
            Mat object = object_vec[max_idx];

            // 키포인트 및 디스크립터 계산
            vector<KeyPoint> keypoints_object, keypoints_scene;
            Mat descriptors_object, descriptors_scene;
            orb->detectAndCompute(object, Mat(), keypoints_object, descriptors_object);
            orb->detectAndCompute(scene, Mat(), keypoints_scene, descriptors_scene);

            //매칭 객체 생성
        BFMatcher matcher(NORM_HAMMING);
        vector<DMatch> matches;
        matcher.match(descriptors_object, descriptors_scene, matches);

        //매칭 결과 시각화
        Mat img_matches;
        drawMatches(object, keypoints_object, scene, keypoints_scene,
                    matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                    vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

        sensor_msgs::ImagePtr output_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_matches).toImageMsg(); // output_img를 ROS Image 메시지로 변환
		        img_pub.publish(output_msg); // 변환된 이미지를 topic으로 publish
    }
    else {
        cout << "Object not found." << endl;
    }
}
catch (cv_bridge::Exception& e)
{
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
}
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_subscriber");
    ros::NodeHandle nh;
		image_transport::ImageTransport it(nh);
		image_transport::Publisher img_pub;
    ros::Subscriber sub = nh.subscribe("/camera/image", 1, imageCallback);
		img_pub = it.advertise("/sign_detector",1);

		while(ros::ok()){
	    ros::spinOnce();
		}
    return 0;
}
