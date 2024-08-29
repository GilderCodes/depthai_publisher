#!/usr/bin/env python3
import cv2
import rospy
import numpy as np
import tf2_ros
import tf_conversions
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import TransformStamped
#from cv_bridge import CvBridge

class PoseEstimator:
    def __init__(self, tf_broadcaster):
        # Initialize the CvBridge for image processing (if needed later)
        #self.br = CvBridge()
        rospy.init_node('pose_estimator', anonymous=True)
        rospy.Subscriber('/aruco_detection', Float32MultiArray, self.corners_callback)

        self.aruco_stored_pub = rospy.Publisher('/aruco_stored', Float32MultiArray, queue_size=10)

        # Initialize object (ArUco) as a square
        marker_size = 1.0  # Assuming a marker of size 1 unit
        self.model_object = np.array([
            (-marker_size / 2, marker_size / 2, 0),  # Top-left corner
            (marker_size / 2, marker_size / 2, 0),   # Top-right corner
            (marker_size / 2, -marker_size / 2, 0),  # Bottom-right corner
            (-marker_size / 2, -marker_size / 2, 0)  # Bottom-left corner
        ], dtype=np.float32)

        # Camera Params!!!
        #info from D (0,1,2,3,4)
        self.dist_coeffs = np.array([[-0.10818, 0.12793, 0.00000, 0.00000, -0.04204]], dtype=np.float32)
        #info from P ([0,1,2],[4,5,6],[8,9,10])
        self.camera_matrix = np.array([(615.381, 0.0, 320.0), 
                                       (0.0, 615.381, 240.0), 
                                       (0.0, 0.0, 1.0)], dtype=np.float32)
        
        # Store the tf2 broadcaster
        self.tf_broadcaster = tf_broadcaster
        
        rospy.loginfo("PoseEstimator initialized successfully")

    def corners_callback(self, msg):
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
            t.header.frame_id = "camera"  # Camera frame
            t.child_frame_id = "marker_{}".format(aruco_id)  # Target frame

            t.transform.translation.x = tvec[0]
            t.transform.translation.y = tvec[1]
            t.transform.translation.z = 0.0#tvec[2]

            # Convert rotation vector to quaternion
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0

            # Publish the transform
            self.tf_broadcaster.sendTransform(t)

        else:
            rospy.logwarn(f"Pose estimation failed for marker ID {aruco_id}.")

    def run(self):
        rospy.spin()

# Define the callable function that will be used in the main script
def main():
    tf_broadcaster = tf2_ros.TransformBroadcaster()
    estimator = PoseEstimator(tf_broadcaster)
    estimator.run()

if __name__ == '__main__':
    main()
