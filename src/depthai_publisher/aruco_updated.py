#!/usr/bin/env python3

import cv2
import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from std_msgs.msg import Float32MultiArray
import threading

class ArucoDetector():
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_100)
    aruco_params = cv2.aruco.DetectorParameters_create()

    frame_sub_topic = '/depthai_node/image/compressed'

    def __init__(self):
        self.image_pub = rospy.Publisher(
            '/processed_aruco/image/compressed', CompressedImage, queue_size=10)  # Publisher for processed images

        self.aruco_detection_pub = rospy.Publisher('/aruco_detection', Float32MultiArray, queue_size=10)  # Combined publisher

        self.br = CvBridge()
        self.last_msg_time = rospy.Time(0)
        self.lock = threading.Lock()
        self.detected_ids = set()  # Set to store detected ArUco marker IDs
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
            rospy.logerr(e)
            return

        aruco = self.find_aruco(frame)
        self.publish_to_ros(aruco)

    def find_aruco(self, frame):
        (corners, ids, _) = cv2.aruco.detectMarkers(
            frame, self.aruco_dict, parameters=self.aruco_params)

        if len(corners) > 0:
            ids = ids.flatten()

            for (marker_corner, marker_ID) in zip(corners, ids):
                # Draw the bounding box around the detected ArUco marker
                corners = marker_corner.reshape((4, 2))
                (top_left, top_right, bottom_right, bottom_left) = corners

                # Draw marker edges regardless of detection state
                cv2.line(frame, tuple(map(int, top_left)), tuple(map(int, top_right)), (0, 255, 0), 2)
                cv2.line(frame, tuple(map(int, top_right)), tuple(map(int, bottom_right)), (0, 255, 0), 2)
                cv2.line(frame, tuple(map(int, bottom_right)), tuple(map(int, bottom_left)), (0, 255, 0), 2)
                cv2.line(frame, tuple(map(int, bottom_left)), tuple(map(int, top_left)), (0, 255, 0), 2)

                # Annotate the frame with the detected ID
                cv2.putText(frame, str(marker_ID), (int(top_left[0]), int(top_left[1]) - 15),
                            cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 255, 0), 2)

                # Publish ID and corners only if the marker hasn't been detected before
                if marker_ID not in self.detected_ids:
                    rospy.loginfo("Aruco detected, ID: {}".format(marker_ID))
                    
                    # Mark ID as detected
                    self.detected_ids.add(marker_ID)

                    # Create a Float32MultiArray message for the combined ID and corners
                    detection_msg = Float32MultiArray()
                    detection_msg.data = [float(marker_ID)] + [coord for point in corners for coord in point]

                    # Publish the combined message
                    self.aruco_detection_pub.publish(detection_msg)
                    rospy.loginfo("Published Aruco ID and corners: {}".format(detection_msg.data))

        return frame

    def publish_to_ros(self, frame):
        msg_out = CompressedImage()
        msg_out.header.stamp = rospy.Time.now()
        msg_out.format = "jpeg"
        msg_out.data = np.array(cv2.imencode('.jpg', frame)[1]).tobytes()

        self.image_pub.publish(msg_out)  # Publish the processed image

def main():
    rospy.init_node('EGB349_vision', anonymous=True)
    rospy.loginfo("Processing images...")

    aruco_detect = ArucoDetector()

    rospy.spin()

if __name__ == '__main__':
    main()
