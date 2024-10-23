#!/usr/bin/env python

import os
import rospy
from sensor_msgs.msg import Image
from easy_image_undistort.srv import ImageUndistort, ImageUndistortRequest, ImageUndistortResponse
import cv2
from cv_bridge import CvBridge

ORIGIN_DATA_PATH = '/home/dzp62442/Projects/white_car_demo/origin/0'
UNDISTORT_DATA_PATH = '/home/dzp62442/Projects/white_car_demo/undistort_ros/0'

def image_service_client():
    rospy.init_node('image_service_client', anonymous=True)
    rospy.wait_for_service('/undistort')  # 等待服务可用
    os.makedirs(UNDISTORT_DATA_PATH, exist_ok=True)
    print("Start undistort images...")

    imgs_list = os.listdir(ORIGIN_DATA_PATH)
    imgs_list.sort()
    
    for img_name in imgs_list:
        img_path = os.path.join(ORIGIN_DATA_PATH, img_name)
        cv_image = cv2.imread(img_path)
        
        try:
            image_service = rospy.ServiceProxy('/undistort', ImageUndistort)  # 创建服务客户端
            bridge = CvBridge()  # 使用cv_bridge将OpenCV图像转换为ROS图像消息
            request_image = bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")

            # 创建服务请求
            request = ImageUndistortRequest()
            request.request_image = request_image

            # 调用服务
            response = image_service(request)

            # 从响应中获取图像
            response_image = bridge.imgmsg_to_cv2(response.response_image, desired_encoding="bgr8")
            print(response_image.shape)
            cv2.imwrite(os.path.join(UNDISTORT_DATA_PATH, img_name), response_image)

        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)


if __name__ == "__main__":
    image_service_client()
