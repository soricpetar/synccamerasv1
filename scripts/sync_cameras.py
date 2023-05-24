#!/home/s_petar/miniconda3/envs/clean_seminar/bin/python
# -*- coding: utf-8 -*-

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from message_filters import ApproximateTimeSynchronizer, Subscriber
from sensor_msgs.msg import Image
import numpy as np
import signal
import torch
from torchvision import transforms
import sys
print("Python version")
print (sys.version)
print("Version info.")
print (sys.version_info)


def process_images(img1, img2):
    combined_image = cv2.hconcat([img1, img2])
    print("ovdje sam")
    cv2.imshow('Synchronized Images', combined_image)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        cv2.destroyAllWindows()
        rospy.signal_shutdown('User requested shutdown')


class SyncCameras:

    def __init__(self):
        self.bridge = CvBridge()

        cam1_sub = Subscriber('/usb_cam1/image_raw', Image)
        cam2_sub = Subscriber('/usb_cam2/image_raw', Image)

        self.ats = ApproximateTimeSynchronizer([cam1_sub, cam2_sub],
                                               queue_size=10, slop=0.015)
        self.ats.registerCallback(self.callback)
        print("init")

    def callback(self, img1, img2):
        print("callback")
        try:
            cv_img1 = self.bridge.imgmsg_to_cv2(img1, 'bgr8')
            cv_img2 = self.bridge.imgmsg_to_cv2(img2, 'bgr8')
        except CvBridgeError as e:
            rospy.logerr(e)
            return
        image_rgb1 = cv2.cvtColor(cv_img1, cv2.COLOR_BGR2RGB)
        image_rgb2 = cv2.cvtColor(cv_img2, cv2.COLOR_BGR2RGB)
        image_resized1 = cv2.resize(image_rgb1, (256, 256))
        image_resized2 = cv2.resize(image_rgb2, (256, 256))
        transform = transforms.Compose([
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])]) # means and std taken from ImageNet dataset statistics.
        transformed_img = transform(image_resized1)
        transformed_img = transform(image_resized2)
        time_difference = img1.header.stamp - img2.header.stamp
        rospy.loginfo('Time difference between images: %s seconds',
                      time_difference.to_sec())
        process_images(cv_img1, cv_img2)


if __name__ == '__main__':

    rospy.init_node('sync_cameras', anonymous=True)
    sc = SyncCameras()
    print('running')
    rospy.spin()
