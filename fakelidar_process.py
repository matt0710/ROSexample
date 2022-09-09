#!/usr/bin/python

import numpy as np
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point32
from sensor_msgs.msg import PointCloud

points = []

def callback(data):
        global points
        points = data.points
        rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.points)


def main():

    rospy.init_node("fakelidar_process")
    rospy.Subscriber("fakelidar_pc", PointCloud, callback)

    pub = rospy.Publisher("fakelidar_pc_rot", PointCloud, queue_size=10)
    rate = rospy.Rate(1)


    rot_angle = rospy.get_param("/rot_angle")


    while not rospy.is_shutdown():
        pointcloud = PointCloud()


        for i in range(len(points)):
                pointcloud.points.append(Point32(points[i].x*np.cos(rot_angle) + points[i].y*np.sin(rot_angle),
                                                -points[i].x*np.sin(rot_angle) + points[i].y*np.cos(rot_angle),
                                                0
                                                )
                                        )

        hello_str = "points: %s" % pointcloud.points
        rospy.loginfo(hello_str)
        pub.publish(pointcloud)
        rate.sleep()

if __name__ == "__main__":
        main()

