#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray

def corners_callback(msg):
    # Extract the data from the Float32MultiArray message
    corners = msg.data
    
    # Print the corners to the terminal
    rospy.loginfo("Received ArUco corners:")
    for i in range(0, len(corners), 2):
        x, y = corners[i], corners[i+1]
        rospy.loginfo("Corner {}: x = {:.2f}, y = {:.2f}".format(i // 2, x, y))

def main():
    rospy.init_node('aruco_corners_subscriber', anonymous=True)
    
    rospy.loginfo("Starting ArUco corners subscriber...")

    # Create a subscriber to the /aruco_corners topic
    rospy.Subscriber('/aruco_corners', Float32MultiArray, corners_callback)

    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    main()
