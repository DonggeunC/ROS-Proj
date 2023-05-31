#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

ros::Publisher twist_pub;
float desired_distance = 0.15; // 장애물과의 거리
float linear_velocity = 0.2; // 직진 속도
float angular_velocity = 0.5; // 회전 속도

void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
    // 스캔값 가져오기
    std::vector<float> ranges = scan_msg->ranges;
    int num_ranges = ranges.size();

    // 최소 거리 및 해당 인덱스 초기화
    float min_distance = std::numeric_limits<float>::infinity();
    int min_distance_index = -1;

    // 최소 거리 및 해당 인덱스 찾기
    for (int i = 0; i < num_ranges; ++i)
    {
        if (std::isfinite(ranges[i]) && ranges[i] < min_distance)
        {
            min_distance = ranges[i];
            min_distance_index = i;
        }
    }

    // twist 메시지 생성
    geometry_msgs::Twist twist_msg;

    if (min_distance < desired_distance)
    {
        // 장애물이 감지된 경우
        twist_msg.linear.x = linear_velocity * (min_distance / desired_distance);
        twist_msg.angular.z = angular_velocity;
    }
    else
    {
        // 장애물이 감지되지 않은 경우
        twist_msg.linear.x = linear_velocity;
        twist_msg.angular.z = 0.0;
    }

    twist_pub.publish(twist_msg);
	ROS_INFO_STREAM("Dist : " << min_distance);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "avoid_obstacles_node");
    ros::NodeHandle nh;

    twist_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    ros::Subscriber lidar_sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 10, lidarCallback);

    ros::spin();

    return 0;
}
