#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
import json

pose_data = []

def pose_callback(msg):
    global pose_data
    pose = {
        'position': {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'z': msg.pose.pose.position.z
        },
        'orientation': {
            'x': msg.pose.pose.orientation.x,
            'y': msg.pose.pose.orientation.y,
            'z': msg.pose.pose.orientation.z,
            'w': msg.pose.pose.orientation.w
        }
    }
    pose_data.append(pose)

def save_pose_data():
    with open('slam_poses.json', 'w') as outfile:
        json.dump(pose_data, outfile)

def pose_subscriber():
    rospy.init_node('pose_subscriber_node', anonymous=True)
    pose_topic = rospy.get_param('~pose_topic', '/slam_pose')
    rospy.Subscriber(pose_topic, Odometry, pose_callback)
    rospy.on_shutdown(save_pose_data)
    rospy.spin()

if __name__ == '__main__':
    pose_subscriber()
