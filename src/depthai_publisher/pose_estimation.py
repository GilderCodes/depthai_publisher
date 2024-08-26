#!/usr/bin/env python3

import sys
import rospy
import cv2
import numpy as np
import tf2_ros
from geometry_msgs.msg import TransformStamped

from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge, CvBridgeError

class PoseEstimator():
    def __init__(self):
        # Initialize other members...
        marker_size = 1.0  # Assuming a marker of size 1 unit
        self.model_object = np.array([
         (-marker_size / 2, marker_size / 2, 0),  # Top-left corner
         (marker_size / 2, marker_size / 2, 0),   # Top-right corner
         (marker_size / 2, -marker_size / 2, 0),  # Bottom-right corner
         (-marker_size / 2, -marker_size / 2, 0)  # Bottom-left corner
        ], dtype=np.float32)

        self.model_image = np.zeros((4, 2), dtype=np.float32)
        self.marker_id = None

          # Setup the TF2 broadcaster
        self.tfbr = tf2_ros.TransformBroadcaster()      

    def corners_callback(self, msg):
        global processed_markers

        marker_id = msg.data[0]
        if marker_id in processed_markers:
            corners_2d = processed_markers[marker_id]

            # Perform pose estimation
            (success, rvec, tvec) = cv2.solvePnP(self.model_object, corners_2d, self.camera_matrix, self.dist_coeffs)

            if success:
               # Create a TransformStamped message
                
                # rvec: rotation vector
                # tvec: translation vector
                rospy.loginfo(f"Rotation Vector: {rvec}")
                rospy.loginfo(f"Translation Vector: {tvec}")
                
                t = TransformStamped()
                t.header.stamp = rospy.Time.now()
                t.header.frame_id = "camera"  # Camera frame
                t.child_frame_id = "marker_{}".format(marker_id) #"target"  # Target frame

                t.transform.translation.x = tvec[0]
                t.transform.translation.y = tvec[1]
                t.transform.translation.z = tvec[2]

                # Convert rotation vector to quaternion
                (rotation_matrix, _) = cv2.Rodrigues(rvec)
                quaternion = tf.transformations.quaternion_from_matrix(np.hstack((rotation_matrix, np.array([[0], [0], [0]]))))
                t.transform.rotation.x = quaternion[0]
                t.transform.rotation.y = quaternion[1]
                t.transform.rotation.z = quaternion[2]
                t.transform.rotation.w = quaternion[3]

                # Broadcast the transform
                self.tfbr.sendTransform(t)

                # Optionally publish timestamp for external use
                pub_found.publish(rospy.Time.now())
            else:
                rospy.logwarn("Pose estimation failed for marker ID {}.".format(marker_id))

    def camera_info_callback(self, msg):
        self.camera_matrix = np.array([
            [(1479.458984, 0.000000, 950.694458), 
             (0.000000, 1477.587158, 530.697632), 
             (0.000000, 0.000000, 1.000000)]
        ], dtype=np.float32)
        self.dist_coeffs = np.array([-1.872860,   16.683033,    0.001053,   -0.002063,   61.878521,   -2.158907,   18.424637,
        57.682858,    0.000000,    0.000000,    0.000000,    0.000000,    0.000000,    0.000000], dtype=np.float32)

    def main(self):
        rospy.init_node('pose_estimator', anonymous=True)
        rospy.Subscriber('/aruco_corners', Float32MultiArray, self.corners_callback)
        # Not sure if i need the line below as the params have been manually inserted
        rospy.Subscriber('/camera_info', CameraInfo, self.camera_info_callback)
        rospy.spin()

