int main(int argc, char** argv)
{
	ros::init(argc, argv, "lane_detection");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	image_transport::Subscriber sub = it.subscribe("/camera/image_raw", 1, boost::bind(&imageCallback, _1, pub));
	ros::spin();
	return 0;
}