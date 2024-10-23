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
#include "easy_image_undistort/ImageUndistort.h"

using namespace easy_image_undistort;

class EasyImageUndistortNode
{
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    ros::Timer timer_;
	ros::ServiceServer undistort_server_;
    image_transport::Publisher render_result_pub_;
    std::shared_ptr<CameraParametersPair> camera_parameters_pair_ptr_;
    std::shared_ptr<Undistorter> undistorter_ptr_;

public:
    EasyImageUndistortNode(): nh_("~"), it_(nh_) 
    {
        camera_parameters_pair_ptr_ = std::make_shared<CameraParametersPair>(DistortionProcessing::UNDISTORT);

        // 加载输入相机参数
        std::string input_camera_namespace;
        nh_.param("input_camera_namespace", input_camera_namespace, std::string("input_camera"));
        if (!camera_parameters_pair_ptr_->setCameraParameters(nh_, input_camera_namespace, CameraIO::INPUT)) {
            ROS_FATAL("Loading of input camera parameters failed, exiting");
            ros::shutdown();
            exit(EXIT_FAILURE);
        }

        // 加载输出相机参数
        double scale_;
        nh_.param("scale", scale_, 1.0);
        std::string output_camera_info_source;
        nh_.param("output_camera_info_source", output_camera_info_source, std::string("input"));
        if (output_camera_info_source == "ros_params") {
            std::string output_camera_namespace;
            nh_.param("output_camera_namespace", output_camera_namespace, std::string("output_camera"));
            if (!camera_parameters_pair_ptr_->setCameraParameters(nh_, output_camera_namespace, CameraIO::OUTPUT)) {
                ROS_FATAL("Loading of output camera parameters failed, exiting");
                ros::shutdown();
                exit(EXIT_FAILURE);
            }
        } 
        else if (output_camera_info_source == "match_input") {
            camera_parameters_pair_ptr_->setOutputFromInput(scale_);
        } 
        else if (output_camera_info_source == "auto_generated") {
            camera_parameters_pair_ptr_->setOptimalOutputCameraParameters(scale_);
        }
        
        undistorter_ptr_ = std::make_shared<Undistorter>(*camera_parameters_pair_ptr_);
        undistort_server_ = nh_.advertiseService("/undistort", &EasyImageUndistortNode::processUndistortRequest, this);  
        ROS_WARN("Easy image undistort node started .");
    }

private:
    // 服务器回调函数，处理请求图像并生成响应图像
    bool processUndistortRequest(easy_image_undistort::ImageUndistort::Request &req, easy_image_undistort::ImageUndistort::Response &res)
    {
        std::string encoding = sensor_msgs::image_encodings::BGR8;
        sensor_msgs::ImageConstPtr image_msg_ptr = boost::make_shared<sensor_msgs::Image>(req.request_image);
        cv_bridge::CvImageConstPtr image_in_ptr = cv_bridge::toCvShare(image_msg_ptr, encoding);
        cv_bridge::CvImagePtr image_out_ptr(new cv_bridge::CvImage(image_in_ptr->header, encoding));

        // Undistort the image
        undistorter_ptr_->undistortImage(image_in_ptr->image, &(image_out_ptr->image));

        res.response_image = *(image_out_ptr->toImageMsg());

        ROS_WARN("Response one image undistort request .");
        return true;
    }

};




int main(int argc, char* argv[]) {

	ros::init(argc, argv, "easy_image_undistort_node");

    EasyImageUndistortNode easy_image_undistort_node;

	ros::spin();

	return 0;
}