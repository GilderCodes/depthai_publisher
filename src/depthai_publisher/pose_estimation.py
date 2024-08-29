#!/usr/bin/env python3
import cv2
import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import TransformStamped
#from cv_bridge import CvBridge


class PoseEstimator:
    def __init__(self):
        # Initialize the CvBridge for image processing (if needed later)
        #self.br = CvBridge()
        rospy.init_node('pose_estimator', anonymous=True)
        rospy.Subscriber('/aruco_detection', Float32MultiArray, self.corners_callback)

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
        
        rospy.loginfo(f"Succuess int ")

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
                
                # rvec: rotation vector
                # tvec: translation vector
            rospy.loginfo(f"Rotation Vector: {rvec}")
            rospy.loginfo(f"Translation Vector: {tvec}")
                 
            t = TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "camera"  # Camera frame
            t.child_frame_id = "marker_{}".format(aruco_id) #"target"  # Target frame

            t.transform.translation.x = tvec[0]
            t.transform.translation.y = tvec[1]
            t.transform.translation.z = tvec[2]

                # Convert rotation vector to quaternion
                #(rotation_matrix, _) = cv2.Rodrigues(rvec)
                #quaternion = tf.transformations.quaternion_from_matrix(np.hstack((rotation_matrix, np.array([[0], [0], [0]]))))
                #t.transform.rotation.x = quaternion[0]
                #t.transform.rotation.y = quaternion[1]
                #t.transform.rotation.z = quaternion[2]
                #t.transform.rotation.w = quaternion[3]


        else:
            rospy.logwarn("Pose estimation failed for marker ID {}.".format(aruco_id))




    def run(self):
        
        rospy.spin()


# Define the callable function that will be used in the main script
def main():
    estimator = PoseEstimator()
    estimator.run()