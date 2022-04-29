#!/usr/bin/env python

import roslib
import rospy

import tf
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import numpy as np
import tf_conversions
import tf2_ros
import geometry_msgs.msg


pub = None
br = tf2_ros.TransformBroadcaster()
new_msg = Odometry()
t = geometry_msgs.msg.TransformStamped()
def handle_pose(msg, name):
    global new_msg
    new_msg.header = msg.header
    new_msg.header.frame_id = "map"
    new_msg.header.stamp = rospy.get_rostime()
    new_msg.child_frame_id = name+"/base_footprint"
    new_msg.pose.pose = msg.pose

def handle_odom(msg, name):
    global t, br

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = name+"/odom"
    t.child_frame_id = name+"/base_footprint"
    t.transform.translation.x = msg.pose.pose.position.x
    t.transform.translation.y = msg.pose.pose.position.y
    t.transform.translation.z = msg.pose.pose.position.z
    t.transform.rotation = msg.pose.pose.orientation


if __name__ == '__main__':
    global t, br, new_msg, pub
    try:
        rospy.init_node('tf_broadcaster')
        name = rospy.get_param('~name')
        t.header.frame_id = name+"/odom"
        t.child_frame_id = name+"/base_footprint"
        t.header.stamp = rospy.Time.now()
        t.transform.translation.x = 0
        t.transform.translation.y = 0
        t.transform.translation.z = 0
        t.transform.rotation.w = 1
        rospy.Subscriber('/%s/pose' % name, PoseStamped, handle_pose, name)
        pub = rospy.Publisher('/%s/base_pose_ground_truth' % name, Odometry, queue_size=1)
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            new_msg.header.stamp = rospy.get_rostime()
            pub.publish(new_msg)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
