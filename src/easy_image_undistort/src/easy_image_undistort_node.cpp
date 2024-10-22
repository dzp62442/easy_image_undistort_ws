#include <stdio.h>
#include <Eigen/Dense>

#include <image_transport/image_transport.h>
#include <nodelet/nodelet.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include "easy_image_undistort/undistorter.h"


int main(int argc, char* argv[]) {
	// 初始化 ROS
	ros::init(argc, argv, "easy_image_undistort_node");

	ros::spin();

	return 0;
}