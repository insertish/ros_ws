#!/usr/bin/env python3
# coding=utf-8
import rospy
from cv_bridge import CvBridge
import cv2
import numpy as np
from yolov4 import Detector
import random

from yolo.srv import YOLOLastFrame, YOLOLastFrameResponse
from yolo.msg import YOLODetection
from sensor_msgs.msg import Image

from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient


class YOLOv4ROSITR:
    def __init__(self):
        self.soundhandle = SoundClient()
        self.bridge = CvBridge()
        self.cv_image = None
        self.colors = {}

        # TODO ADD HERE YOUR CODE TO INITIALIZE THE DETECTOR, SUBSCRIBERS AND SERVICES
        self.detector = Detector(gpu_id=0, config_path='/opt/darknet/cfg/yolov4.cfg',
                                 weights_path='/opt/darknet/yolov4.weights',
                                 lib_darknet_path='/opt/darknet/libdarknet.so',
                                 meta_path='src/yolo/cfg/coco.data')
        self.sub = rospy.Subscriber(
            "/usb_cam/image_raw", Image, self.img_callback)
        self.pub = rospy.Publisher("/detected_frame", Image, queue_size=10)
        self.srv = rospy.Service(
            'detection_service', YOLOLastFrame, self.yolo_service)

    def img_callback(self, msg):
        self.cv_image = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding='passthrough')

    def yolo_service(self, request):
        res = YOLOLastFrameResponse()
        if self.cv_image is not None:
            cv_copy = self.cv_image.copy()
            # reize all images to network size
            img_arr = cv2.resize(cv_copy, (
                self.detector.network_width(),
                self.detector.network_height()
            ))

            # run YOLO
            detections = self.detector.perform_detect(
                image_path_or_buf=img_arr, show_image=True)

            cv_height, cv_width, _ = self.cv_image.shape
            for detection in detections:
                box = detection.left_x, detection.top_y, detection.width, detection.height
                print(
                    f'{detection.class_name.ljust(10)} | {detection.class_confidence * 100:.1f} % | {box}')
                d = YOLODetection(detection.class_name, detection.class_confidence, detection.left_x, detection.top_y,
                                  detection.width, detection.height)
                # convert bbox to image space
                d.bbox_x = int(
                    (d.bbox_x/self.detector.network_width())*cv_width)
                d.bbox_y = int(
                    (d.bbox_y/self.detector.network_height())*cv_height)
                d.width = int((d.width/self.detector.network_width())*cv_width)
                d.height = int(
                    (d.height/self.detector.network_height())*cv_height)
                res.detections.append(d)

                # This paints the bounding boxes in the images with the name in it.
                if d.name in self.colors:
                    color = self.colors[d.name]
                else:
                    color = (random.randint(0, 255), random.randint(
                        0, 255), random.randint(0, 255))
                    self.colors[d.name] = color
                cv2.rectangle(cv_copy, (d.bbox_x, d.bbox_y),
                              (d.bbox_x+d.width, d.bbox_y+d.height), color, 2)
                cv2.rectangle(cv_copy, (d.bbox_x, d.bbox_y),
                              (d.bbox_x+5+(23*len(d.name)), d.bbox_y+30), color, -1)
                cv2.putText(cv_copy, d.name, (d.bbox_x+2, d.bbox_y+25),
                            cv2.FONT_HERSHEY_PLAIN, 2.5, (0, 0, 0))
                # TODO: publish the inpainted image
                # Helper: to convert from opencv image to a image message you can do :
                message = self.bridge.cv2_to_imgmsg(cv_copy, encoding="bgr8")
                self.pub.publish(message)

                self.soundhandle.say(
                    # voice_kal_diphone
                    f"I found a {{d.name}}", "kal_diphone", 1.0)
            else:
                print("!shit fuck1!!")
                pass
        return res


if __name__ == '__main__':
    rospy.init_node('yolo_ros_itr')
    yolo_ros = YOLOv4ROSITR()
    rospy.spin()
