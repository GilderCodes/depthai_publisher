#!/usr/bin/env python3

import math

import rospy
import tf2_ros
from std_msgs.msg import Time
from geometry_msgs.msg import TransformStamped

# Global Variables
tfbr = None
tfsbr = None

target_name = "target"
camera_name = "camera"

def send_tf_camera():
	# Create a static transform that is slightly
	# below the UAV and pointing downwards
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = camera_name
    t.child_frame_id = target_name

    t.transform.translation.x = msg.transform.translation.x
    t.transform.translation.y = msg.transform.translation.y
    t.transform.translation.z = msg.transform.translation.z
    t.transform.rotation.x = msg.transform.rotation.x
    t.transform.rotation.y = msg.transform.rotation.y
    t.transform.rotation.z = msg.transform.rotation.z
    t.transform.rotation.w = msg.transform.rotation.w
    
    # Send the dynamic transformation
    tfsbr.sendTransform(t) 
    pub_found.publish(rospy.Time.now())
    

"""
This is step typically done by the same program that outputs the pose

def callback_pose( msg_in ):
	# Create a transform at the time
	# from the pose message for where
	# the UAV is in the map
	t = TransformStamped()
	t.header = msg_in.header
	t.child_frame_id = uav_name

	t.transform.translation = msg_in.pose.position
	t.transform.rotation = msg_in.pose.orientation

	# Send the transformation
	tfbr.sendTransform(t)
"""

if __name__ == '__main__':
	rospy.init_node('tf2_broadcaster_target')
	rospy.loginfo("tf2_broadcaster_target sending target found...")

	# Setup pose subscriber
	# This functionality is provided by the emulator
	#rospy.Subscriber('/emulated_uav/pose', PoseStamped, callback_pose)
    # Setup tf2 broadcaster and timestamp publisher
	tfbr = tf2_ros.TransformBroadcaster() 
	pub_found = rospy.Publisher('/emulated_uav/target_found', Time, queue_size=10)

	# Give the nodes a few seconds to configure
	rospy.sleep(rospy.Duration(2))

	# Send out our target messages
	send_tf_camera()
    # Subscribe to the pose estimator topic
	rospy.Subscriber('/pose_estimator/target_transform', TransformStamped, target_tf_callback)
      
	rospy.spin()


