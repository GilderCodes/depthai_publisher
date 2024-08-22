#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray, Int32

# Global dictionary to keep track of processed marker IDs and their corners
processed_markers = {}

def corners_callback(msg):
    global processed_markers

    # Extract the data from the Float32MultiArray message
    corners = msg.data

    # Assuming the format is [marker_id, x1, y1, x2, y2, ..., xn, yn]
    num_corners = len(corners) - 1
    marker_id = int(corners[0])  # Extract marker ID

    # Check if this marker ID has already been processed
    if marker_id in processed_markers:
        return

    # If not processed, add to the dictionary
    processed_markers[marker_id] = corners[1:]

    # Print the corners to the terminal
    rospy.loginfo("Received ArUco corners for marker {}:".format(marker_id))
    for i in range(0, len(processed_markers[marker_id]), 2):
        x, y = processed_markers[marker_id][i], processed_markers[marker_id][i+1]
        rospy.loginfo("Corner {}: x = {:.2f}, y = {:.2f}".format(i // 2, x, y))

def marker_id_callback(msg):
    global processed_markers

    marker_id = msg.data

    # If marker_id is in the processed_markers, print its corners
    if marker_id in processed_markers:
        rospy.loginfo("Marker ID {} found. Corners:".format(marker_id))
        corners = processed_markers[marker_id]
        for i in range(0, len(corners), 2):
            x, y = corners[i], corners[i+1]
            rospy.loginfo("Corner {}: x = {:.2f}, y = {:.2f}".format(i // 2, x, y))
    else:
        rospy.loginfo("Marker ID {} not found in processed markers.".format(marker_id))

def main():
    rospy.init_node('aruco_corners_subscriber', anonymous=True)
    
    rospy.loginfo("Starting ArUco corners subscriber...")

    # Create subscribers for both the /aruco_corners and /aruco_id topics
    rospy.Subscriber('/aruco_corners', Float32MultiArray, corners_callback)
    rospy.Subscriber('/aruco_id', Int32, marker_id_callback)

    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    main()
