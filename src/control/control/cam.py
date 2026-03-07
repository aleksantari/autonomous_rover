#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
import random
import time
from cv_bridge import CvBridge
from ultralytics import YOLO

import cv2


class Cam(Node):
    def __init__(self):
        super().__init__('cam')
        self.sub = self.create_subscription(Image, '/image_raw', self.cam_callback, 2)
        self.yolo_pub = self.create_publisher(Image, '/image_annotated', 10)
        self.bridge = CvBridge()
        print('kms')
        self.yolo = YOLO('yolo26n.pt')
    def cam_callback(self, msg):
        self.get_logger().info("Received an image at time %d" % msg.header.stamp.sec)
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        results = self.yolo.track(frame, persist=True, stream=False)
        annotated_frame = frame
        for r in results:
            boxes = r.boxes.xyxy.cpu().numpy()
            clss = r.boxes.cls.cpu().numpy()

            annotated_frame = r.plot()
        
        cv2.imshow("YOLO Rover Feed", annotated_frame)
        cv2.waitKey(1)
        print("hello?")
        rosframe = self.bridge.cv2_to_imgmsg(annotated_frame, encoding = "bgr8")
        self.yolo_pub.publish(rosframe)
        # result_img_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
        # self.greyscale_publisher.publish(result_img_msg)
    def getColors(cls):
        # random rgb
        random.seed(cls)
        return tuple(random.randint(0, 255) for _ in range(3))
    


rclpy.init()
cam = Cam()
rclpy.spin(cam)