#!/usr/bin/env python3

import cv2
import rospy
import numpy as np
import tf2_ros
import tf_conversions
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import TransformStamped

class PoseEstimator:
    def __init__(self, tf_broadcaster):
        rospy.init_node('pose_estimator', anonymous=True)
        
        # Subscribers for ArUco and ROI detections
        rospy.Subscriber('/aruco_detection', Float32MultiArray, self.aruco_callback)
        rospy.Subscriber('/roi_detection', Float32MultiArray, self.roi_callback)

        # Publishers
        self.aruco_stored_pub = rospy.Publisher('/aruco_stored', Float32MultiArray, queue_size=10)

        # Initialize ArUco marker parameters
        marker_size = 1.0
        self.model_object = np.array([
            (-marker_size / 2, marker_size / 2, 0),  # Top-left corner
            (marker_size / 2, marker_size / 2, 0),   # Top-right corner
            (marker_size / 2, -marker_size / 2, 0),  # Bottom-right corner
            (-marker_size / 2, -marker_size / 2, 0)  # Bottom-left corner
        ], dtype=np.float32)

        # Camera parameters
        self.dist_coeffs = np.array([[-0.10818, 0.12793, 0.00000, 0.00000, -0.04204]], dtype=np.float32)
        self.camera_matrix = np.array([(615.381, 0.0, 320.0), 
                                       (0.0, 615.381, 240.0), 
                                       (0.0, 0.0, 1.0)], dtype=np.float32)
        
        # Store the tf2 broadcaster
        self.tf_broadcaster = tf_broadcaster
        
        rospy.loginfo("PoseEstimator initialized successfully")

    def aruco_callback(self, msg):
        # The first element of `msg.data` is the ArUco ID, and the rest are corners
        aruco_id = int(msg.data[0])
        corners = msg.data[1:]

        # Pull out the ID and the corners into the desired 2D format
        corners_2d = np.array([
            [corners[0], corners[1]],  # Top-left corner
            [corners[2], corners[3]],  # Top-right corner
            [corners[4], corners[5]],  # Bottom-right corner
            [corners[6], corners[7]]   # Bottom-left corner
        ], dtype=np.float32)

        # Print the details of the ArUco ID and corners
        rospy.loginfo(f"Detected ArUco ID: {aruco_id}")
        rospy.loginfo(f"Corners:\n{corners_2d}")

        # Perform pose estimation
        (success, rvec, tvec) = cv2.solvePnP(self.model_object, corners_2d, self.camera_matrix, self.dist_coeffs)

        if success:
            # Create a TransformStamped message
            t = TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "camera"
            t.child_frame_id = "marker_{}".format(aruco_id)

            t.transform.translation.x = tvec[0]
            t.transform.translation.y = tvec[1]
            t.transform.translation.z = 0.0

            # Convert rotation vector to quaternion (dummy values used here)
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0

            # Publish the transform
            self.tf_broadcaster.sendTransform(t)

        else:
            rospy.logwarn(f"Pose estimation failed for marker ID {aruco_id}.")

    def roi_callback(self, msg):
        # The data consists of [object_id, xmin, ymin, xmax, ymax]
        object_id = int(msg.data[0])
        xmin, ymin, xmax, ymax = msg.data[1:5]
        # Determine the label based on the ID
        if object_id == 0.1:
            label = "Backpack"
        elif object_id == 0.2:
            label = "Human"
        elif object_id == 0.3:
            label = "Drone"
        elif object_id == 0.4:
            label = "Phone"
        else:
            label = "Unknown"

        # Convert ROI to a 4-corner format for pose estimation
        corners_2d = np.array([
            [xmin, ymin],  # Top-left corner
            [xmax, ymin],  # Top-right corner
            [xmax, ymax],  # Bottom-right corner
            [xmin, ymax]   # Bottom-left corner
        ], dtype=np.float32)

        # Print the details of the ROI detection
        rospy.loginfo(f"Detected ROI Object ID: {object_id}")
        rospy.loginfo(f"Bounding Box: ({xmin}, {ymin}), ({xmax}, {ymax})")
        rospy.loginfo(f"Converted Corners:\n{corners_2d}")

        # Perform pose estimation
        (success, rvec, tvec) = cv2.solvePnP(self.model_object, corners_2d, self.camera_matrix, self.dist_coeffs)

        if success:
            # Create a TransformStamped message
            t = TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "camera"
            t.child_frame_id = f"roi_{label}" 

            t.transform.translation.x = tvec[0]
            t.transform.translation.y = tvec[1]
            t.transform.translation.z = 0.0

            # Convert rotation vector to quaternion (dummy values used here)
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0

            # Publish the transform
            self.tf_broadcaster.sendTransform(t)

        else:
            rospy.logwarn(f"Pose estimation failed for ROI Object ID {object_id}.")

    def run(self):
        rospy.spin()

def main():
    tf_broadcaster = tf2_ros.TransformBroadcaster()
    estimator = PoseEstimator(tf_broadcaster)
    estimator.run()

if __name__ == '__main__':
    main()
