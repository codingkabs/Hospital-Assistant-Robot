#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from resit_coursework.srv import YOLOLastFrame, YOLOLastFrameResponse
from yolov4 import Detector
import cv2
from cv_bridge import CvBridge, CvBridgeError

class YOLOv4ROSITR: #from the lecture
    def __init__(self):
        self.cv_image = None
        self.bridge = CvBridge()
        self.cam_subs = rospy.Subscriber("/camera/image", Image, self.img_callback)
        self.yolo_srv = rospy.Service('/detect_frame', YOLOLastFrame, self.yolo_service)
        self.detector = Detector(gpu_id=0, config_path='/opt/darknet/cfg/yolov4.cfg',
                                 weights_path='/opt/darknet/yolov4.weights',
                                 lib_darknet_path='/opt/darknet/libdarknet.so',
                                 meta_path='/home/k21082509/ros_ws/src/resit_coursework/config/coco.data') 

    def img_callback(self, msg):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            #rospy.loginfo("Image received and converted.")
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")
            self.cv_image = None

    def yolo_service(self, request):
        res = YOLOLastFrameResponse()
        if self.cv_image is None:
            rospy.logwarn("No image received yet.")
            return False

        img_arr = cv2.resize(self.cv_image, (self.detector.network_width(), self.detector.network_height()))
        
        detections = self.detector.perform_detect(image_path_or_buf=img_arr, show_image=False)

        for detection in detections:
            #rospy.loginfo(f"Detected: {detection.class_name}, confidence: {detection.class_confidence}")
            if detection.class_name == request.object_class and detection.class_confidence > 0.2: #getting errors when using a hishger level of confidence
                res.detected = True
                return res

        res.detected = False
        return res

if __name__ == '__main__':
    rospy.init_node('yolo_ros_itr')
    yolo_ros = YOLOv4ROSITR()
    rospy.spin()
