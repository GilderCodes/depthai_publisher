#!/usr/bin/env python3

import cv2
import rospy
import numpy as np
import tf2_ros
import tf_conversions
from tf2_ros import StaticTransformBroadcaster, Buffer, TransformListener
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Time
from geometry_msgs.msg import TransformStamped, PoseStamped, PointStamped
import subprocess
import math 

class PoseEstimator:
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_100)
    aruco_params = cv2.aruco.DetectorParameters_create()

    def __init__(self, tf_broadcaster, static_tf_broadcaster):
        rospy.init_node('pose_estimator', anonymous=True)
        
        # Subscribers for ArUco and ROI detections
        rospy.Subscriber('/aruco_detection', Float32MultiArray, self.aruco_callback)
        rospy.Subscriber('/roi_detection', Float32MultiArray, self.roi_callback)
        self.pub_found = rospy.Publisher('/emulated_uav/target_found', Time, queue_size=10)
        # self.sub_uav_pose = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.callback_uav_pose)
        self.sub_uav_pose = rospy.Subscriber('/uavasr/pose', PoseStamped, self.callback_uav_pose)

        # Publishers
        self.aruco_stored_pub = rospy.Publisher('/aruco_stored', Float32MultiArray, queue_size=10)

        # Initialize ArUco marker parameters
        marker_size = 0.2 # Updated the marker size to what they will be
        self.model_object = np.array([
            (0, 0, 0),
            (-marker_size / 2, marker_size / 2, 0),  # Top-left corner
            (marker_size / 2, marker_size / 2, 0),   # Top-right corner
            (marker_size / 2, -marker_size / 2, 0),  # Bottom-right corner
            (-marker_size / 2, -marker_size / 2, 0)  # Bottom-left corner
        ], dtype=np.float32)

        # Camera parameters
        self.dist_coeffs = np.array([[-0.10818, 0.12793, 0.00000, 0.00000, -0.04204]], dtype=np.float32) #msg in D
        self.camera_matrix = np.array([(615.381, 0.0, 320.0), 
                                       (0.0, 615.381, 240.0), 
                                       (0.0, 0.0, 1.0)], dtype=np.float32) # msg in P 
        
        # Store the tf2 broadcasters
        self.tf_broadcaster = tf_broadcaster
        self.static_tf_broadcaster = static_tf_broadcaster
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)
        
        rospy.loginfo("PoseEstimator initialized successfully")

    def callback_uav_pose(self, msg_in):
        self.current_location = msg_in.pose.position
        self.uav_pose = [self.current_location.x, self.current_location.y, self.current_location.z, 0.0]
        self.x_p = self.uav_pose[0]
        self.y_p = self.uav_pose[1]
        self.z_p = self.uav_pose[2]
        
        # Log the current UAV position to the screen
        #rospy.loginfo(f"UAV Position: x = {self.x_p:.2f}, y = {self.y_p:.2f}, z = {self.z_p:.2f}")

    def aruco_callback(self, msg):
        # The first element of `msg.data` is the ArUco ID, and the rest are corners
        aruco_id = int(msg.data[0])
        corners = msg.data[1:]

        # Calculate the center as the average of the x and y coordinates of all corners
        center_xc = np.mean([corners[0], corners[2], corners[4], corners[6]])
        center_yc = np.mean([corners[1], corners[3], corners[5], corners[7]])

        # Pull out the ID and the corners into the desired 2D format
        corners_2d = np.array([
            #[center_xc, center_yc],
            [corners[0], corners[1]],  # Top-left corner
            [corners[2], corners[3]],  # Top-right corner
            [corners[4], corners[5]],  # Bottom-right corner
            [corners[6], corners[7]]   # Bottom-left corner
        ], dtype=np.float32)

        # Print the details of the ArUco ID and corners
        rospy.loginfo(f"Detected ArUco ID: {aruco_id}")
        rospy.loginfo(f"Corners:\n{corners_2d}")

        # Perform pose estimation
        #(success, rvec, tvec) = cv2.solvePnP(self.model_object, corners_2d, self.camera_matrix, self.dist_coeffs)
        aruco_corners = corners_2d.reshape((1, 4, 2))
        rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(aruco_corners, 0.2, self.camera_matrix, self.dist_coeffs)

        x, y, z = tvec[0][0]
        rospy.loginfo(f"Marker position from camera: x={x:.3f}, y={y:.3f}, z={z:.3f} meters")
        
        if rvec is not None and tvec is not None:
        # Obtain the transform from the camera frame to the map frame
            try:
                trans = self.tf_buffer.lookup_transform('map', 'camera', rospy.Time(0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logwarn("Failed to lookup transform from camera to map frame")
                return

            # Extract translation and rotation from the transform
            map_translation = trans.transform.translation
            map_rotation = trans.transform.rotation

            # Convert map rotation quaternion to a rotation matrix
            rotation_matrix = tf_conversions.transformations.quaternion_matrix(
                [map_rotation.x, map_rotation.y, map_rotation.z, map_rotation.w]
            )[:3, :3]  # Extract the 3x3 rotation part of the matrix

            # Convert tvec to a 3D numpy array
            tvec_cam = np.array(tvec[0][0])  # Assuming a single marker, extract the first translation vector

            # Apply the rotation and translation to transform the marker position to the map frame
            tvec_map = np.dot(rotation_matrix, tvec_cam) + np.array([map_translation.x, map_translation.y, map_translation.z])

            # Create a TransformStamped message for publishing
            t = TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "map"
            t.child_frame_id = f"ArUco_{aruco_id}"

            t.transform.translation.x = tvec_map[0]
            t.transform.translation.y = tvec_map[1]
            t.transform.translation.z = tvec_map[2]

            rospy.loginfo("Translation x: %f", t.transform.translation.x)
            rospy.loginfo("Translation y: %f", t.transform.translation.y)
            rospy.loginfo("Translation z: %f", t.transform.translation.z)

            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0

            # Publish the transformed pose
            self.static_tf_broadcaster.sendTransform(t)
            rospy.loginfo(f"Static marker published for ArUco ID: {aruco_id}")
        else:
            rospy.logwarn(f"Pose estimation failed for marker ID {aruco_id}.")

    # def roi_callback(self, msg):
    #    # subprocess.Popen(['rosrun', 'spar_node', 'tf2_broadcaster_target'])
    #     # The data consists of [object_id, xmin, ymin, xmax, ymax]
    #     object_id = int(10*msg.data[0])
    #     xmin, ymin, xmax, ymax = msg.data[1:5]
    #     marker_size_x, marker_size_y = msg.data[5:7] 

    #     # Determine the label based on the ID
    #     if object_id == 1:
    #         label = "Backpack"
    #     elif object_id == 2:
    #         label = "Human"
    #     elif object_id == 3:
    #         label = "Drone"
    #     elif object_id == 4:
    #         label = "Phone"
    #     else:
    #         label = "Unknown"


    #     # Create the object model based on the bounding box size
    #     self.model_obj = np.array([
    #         (0, 0, 0),
    #         (-marker_size_x / 2, marker_size_y / 2, 0),  # Top-left corner
    #         (marker_size_x / 2, marker_size_y / 2, 0),   # Top-right corner
    #         (marker_size_x / 2, -marker_size_y / 2, 0),  # Bottom-right corner
    #         (-marker_size_x / 2, -marker_size_y / 2, 0)  # Bottom-left corner
    #     ], dtype=np.float32)
        

    #     # Calculate the center as the average of the x and y coordinates of all corners
    #     center_x = np.mean([xmin, xmax])
    #     center_y = np.mean([ymin, ymax])

    #     # Convert ROI to a 4-corner format for pose estimation
    #     corners_t2d = np.array([
    #         [int(center_x), int(center_y)],
    #         [xmin, ymin],  # Top-left corner
    #         [xmax, ymin],  # Top-right corner
    #         [xmax, ymax],  # Bottom-right corner
    #         [xmin, ymax]   # Bottom-left corner
    #     ], dtype=np.float32)

    #     # Print the details of the ROI detection
    #     rospy.loginfo(f"Detected ROI Object ID: {object_id}")
    #     rospy.loginfo(f"Bounding Box: ({xmin}, {ymin}), ({xmax}, {ymax})")
    #     rospy.loginfo(f"Converted Corners:\n{corners_t2d}")

    #     # Perform pose estimation
    #     (success, rvec, tvec) = cv2.solvePnP(self.model_obj, corners_t2d, self.camera_matrix, self.dist_coeffs)

    #     if success and label != "Unknown":
    #         # Create a TransformStamped message
    #         # Send the target transform
    #         time_found = rospy.Time.now()
    #         t = TransformStamped()
    #         t.header.stamp = rospy.Time.now()
    #         t.header.frame_id = "camera"
    #         t.child_frame_id = "target" #"{}".format(object_id)

    #         t.transform.translation.x = tvec[0]
    #         t.transform.translation.y = tvec[1]
    #         t.transform.translation.z = tvec[2]

    #         rospy.loginfo("Translation x: %f", t.transform.translation.x)
    #         rospy.loginfo("Translation y: %f", t.transform.translation.y)
    #         rospy.loginfo("Translation z: %f", t.transform.translation.z)

    #         # Convert rotation vector to quaternion (dummy values used here)
    #         t.transform.rotation.x = 0.0
    #         t.transform.rotation.y = 0.0
    #         t.transform.rotation.z = 0.0
    #         t.transform.rotation.w = 1.0

    #         # Publish the transform
    #         self.tf_broadcaster.sendTransform(t)
    #         self.pub_found.publish(time_found)
    #         rospy.loginfo(f"Target Sent")
            
    #     else:
    #         rospy.logwarn(f"Pose estimation failed for ROI Object ID {object_id}.")
    def roi_callback(self, msg):
        # The data consists of [object_id, xmin, ymin, xmax, ymax, marker_size_x, marker_size_y]
        object_id = int(10*msg.data[0])
        xmin, ymin, xmax, ymax = msg.data[1:5]
        marker_size_x, marker_size_y = msg.data[5:7]

        # Determine the label based on the ID
        if object_id == 1:
            label = "Backpack"
        elif object_id == 2:
            label = "Human"
        elif object_id == 3:
            label = "Drone"
        elif object_id == 4:
            label = "Phone"
        else:
            label = "Unknown"

        # Calculate object size in pixels
        object_width_pixels = xmax - xmin
        object_height_pixels = ymax - ymin

        # Estimate real-world size
        # Note: You need to provide the actual drone height here
        drone_height = self.z_p  # Assuming self.z_p contains the current drone height
        camera_fov_vertical = 54  # Adjust this based on your camera specs
        image_size = (416, 416)  # As per your specification

        # Calculate total height captured by camera at ground level
        fov_rad = math.radians(camera_fov_vertical)
        total_height_captured = 2 * drone_height * math.tan(fov_rad/2)
        
        # Calculate ratio of real-world distance to pixels
        ratio = total_height_captured / image_size[1]  # Using height of the image
        
        # Estimate object size
        estimated_width = object_width_pixels * ratio
        estimated_height = object_height_pixels * ratio

        rospy.loginfo(f"Estimated object size: {estimated_width:.2f}m x {estimated_height:.2f}m")

        # Create the object model based on the estimated size
        self.model_obj = np.array([
            (-estimated_width/2,  estimated_height/2, 0),  # Top-left corner
            ( estimated_width/2,  estimated_height/2, 0),  # Top-right corner
            ( estimated_width/2, -estimated_height/2, 0),  # Bottom-right corner
            (-estimated_width/2, -estimated_height/2, 0)   # Bottom-left corner
        ], dtype=np.float32)

        # Convert ROI to a 4-corner format for pose estimation
        corners_t2d = np.array([
            [xmin, ymin],  # Top-left corner
            [xmax, ymin],  # Top-right corner
            [xmax, ymax],  # Bottom-right corner
            [xmin, ymax]   # Bottom-left corner
        ], dtype=np.float32)

        # Print the details of the ROI detection
        rospy.loginfo(f"Detected ROI Object ID: {object_id}")
        rospy.loginfo(f"Bounding Box: ({xmin}, {ymin}), ({xmax}, {ymax})")
        rospy.loginfo(f"Converted Corners:\n{corners_t2d}")

        # Perform pose estimation
        (success, rvec, tvec) = cv2.solvePnP(self.model_obj, corners_t2d, self.camera_matrix, self.dist_coeffs)

        if success and label != "Unknown":
            # Create a TransformStamped message
            # Send the target transform
            time_found = rospy.Time.now()
            t = TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "camera"
            t.child_frame_id = "target" #"{}".format(object_id)

            t.transform.translation.x = tvec[0]
            t.transform.translation.y = tvec[1]
            t.transform.translation.z = tvec[2]

            rospy.loginfo("Translation x: %f", t.transform.translation.x)
            rospy.loginfo("Translation y: %f", t.transform.translation.y)
            rospy.loginfo("Translation z: %f", t.transform.translation.z)

            # Convert rotation vector to quaternion (dummy values used here)
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0

            # Publish the transform
            self.tf_broadcaster.sendTransform(t)
            self.pub_found.publish(time_found)
            rospy.loginfo(f"Target Sent")
            
        else:
            rospy.logwarn(f"Pose estimation failed for ROI Object ID {object_id}.")

    def run(self):
        rospy.spin()

def main():
    tf_broadcaster = tf2_ros.TransformBroadcaster()
    static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster()
    estimator = PoseEstimator(tf_broadcaster, static_tf_broadcaster)
    
    estimator.run()

if __name__ == '__main__':
    main()
