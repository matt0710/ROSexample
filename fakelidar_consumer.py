#!/usr/bin/python

import numpy as np
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point32
from sensor_msgs.msg import PointCloud


def callback(data):

    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.points)


def main():

    rospy.init_node("fakelidar_consumer")
    rospy.Subscriber("fakelidar_pc_rot", PointCloud, callback)

    rospy.spin()

if __name__ == "__main__":
    main()
