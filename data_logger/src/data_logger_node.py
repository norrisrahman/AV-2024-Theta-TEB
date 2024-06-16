#!/usr/bin/env python

import rospy
from nav_msgs.msg import Path
import csv
import os

# File path for the CSV file
file_path = '/home/norris/global_path1.csv'

# Write headers to the CSV file
with open(file_path, 'w', newline='') as csvfile:
    csv_writer = csv.writer(csvfile)
    csv_writer.writerow(['x', 'y'])

def path_callback(msg):
    # Append x and y coordinates to the CSV file
    with open(file_path, 'a', newline='') as csvfile:
        csv_writer = csv.writer(csvfile)
        for pose in msg.poses:
            x = pose.pose.position.x
            y = pose.pose.position.y
            csv_writer.writerow([x, y])
            rospy.loginfo("x: %f, y: %f", x, y)

def listener():
    rospy.init_node('path_listener', anonymous=True)
    rospy.Subscriber('/move_base/GraphPlanner/plan', Path, path_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
