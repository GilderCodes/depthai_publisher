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

#!/usr/bin/env python3

# import rospy
# from std_msgs.msg import Int32
# import subprocess
# import os

# class ArucoSubscriber:
#     def __init__(self):
#         self.last_detected_id = None
#         rospy.Subscriber('/aruco_id', Int32, self.aruco_callback)
#         rospy.loginfo("Aruco Subscriber Node Started")

#     def aruco_callback(self, data):
#         current_id = data.data

#         # Check if the detected ID is different from the last detected ID
#         if current_id != self.last_detected_id:
#             rospy.loginfo(f"New Aruco ID detected: {current_id}")
            
#             # Call the rosrun command
#             subprocess.Popen(['rosrun', 'spar_node', 'tf2_broadcaster_target'])
            
#             self.speak_aruco_id(current_id)
#             self.last_detected_id = current_id

#     def speak_aruco_id(self, aruco_id):
#         # Use espeak to announce the ArUco number
#         os.system(f'espeak "ArUco Number {aruco_id} Identified"')
#         rospy.loginfo(f"Announced Aruco ID: {aruco_id}")

# def main():
#     rospy.init_node('aruco_subscriber', anonymous=True)
#     aruco_subscriber = ArucoSubscriber()
#     rospy.spin()

# if __name__ == '__main__':
#     main()
