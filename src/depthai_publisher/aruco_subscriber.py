#!/usr/bin/env python3

import cv2
import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from std_msgs.msg import Int32
import threading

class ArucoDetector():
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_100)
    aruco_params = cv2.aruco.DetectorParameters_create()

    frame_sub_topic = '/depthai_node/image/compressed'

    def __init__(self):
        self.image_pub = rospy.Publisher(
            '/processed_aruco/image/compressed', CompressedImage, queue_size=10)  # Publisher for processed images

        self.aruco_pub = rospy.Publisher('/aruco_id', Int32, queue_size=10)  # Publisher for ArUco ID

        self.br = CvBridge()
        self.last_msg_time = rospy.Time(0)
        self.lock = threading.Lock()
        self.image_sub = rospy.Subscriber('/camera/image/compressed', CompressedImage, self.img_callback)

        if not rospy.is_shutdown():
            self.frame_sub = rospy.Subscriber(
                self.frame_sub_topic, CompressedImage, self.img_callback)

    def img_callback(self, msg_in):
        with self.lock:
            if msg_in.header.stamp <= self.last_msg_time:
                return
            self.last_msg_time = msg_in.header.stamp

        try:
            frame = self.br.compressed_imgmsg_to_cv2(msg_in)
        except CvBridgeError as e:
            rospy.logger(e)
            return
  

        aruco = self.find_aruco(frame)
        self.publish_to_ros(aruco)

    def find_aruco(self, frame):
        (corners, ids, _) = cv2.aruco.detectMarkers(
            frame, self.aruco_dict, parameters=self.aruco_params)

        if len(corners) > 0:
            ids = ids.flatten()

            for (marker_corner, marker_ID) in zip(corners, ids):
                corners = marker_corner.reshape((4, 2))
                (top_left, top_right, bottom_right, bottom_left) = corners

                top_right = (int(top_right[0]), int(top_right[1]))
                bottom_right = (int(bottom_right[0]), int(bottom_right[1]))
                bottom_left = (int(bottom_left[0]), int(bottom_left[1]))
                top_left = (int(top_left[0]), int(top_left[1]))

                cv2.line(frame, top_left, top_right, (0, 255, 0), 2)
                cv2.line(frame, top_right, bottom_right, (0, 255, 0), 2)
                cv2.line(frame, bottom_right, bottom_left, (0, 255, 0), 2)
                cv2.line(frame, bottom_left, top_left, (0, 255, 0), 2)

                rospy.loginfo("Aruco detected, ID: {}".format(marker_ID))

                # Create an Int32 message and publish it
                marker_msg = Int32()
                marker_msg.data = int(marker_ID)
                self.aruco_pub.publish(marker_msg)  # Publish the ArUco ID
                rospy.loginfo("Published Aruco ID: {}".format(marker_ID))

                cv2.putText(frame, str(marker_ID), (top_left[0], top_right[1] - 15),
                            cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 255, 0), 2)

        return frame

    def publish_to_ros(self, frame):
        msg_out = CompressedImage()
        msg_out.header.stamp = rospy.Time.now()
        msg_out.format = "jpeg"
        msg_out.data = np.array(cv2.imencode('.jpg', frame)[1]).tostring()

        self.image_pub.publish(msg_out)  # Publish the processed image

def main():
    rospy.init_node('EGB349_vision', anonymous=True)
    rospy.loginfo("Processing images...")

    aruco_detect = ArucoDetector()

    rospy.spin()

if __name__ == '__main__':
    main()
