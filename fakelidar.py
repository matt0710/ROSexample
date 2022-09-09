#!/usr/bin/env python
import numpy as np
import rospy
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Point32
from sensor_msgs.msg import PointCloud

def publisher():
    pub = rospy.Publisher('fakelidar_pc', PointCloud, queue_size=10)
    rospy.init_node('fakelidar', anonymous=True)
    rate = rospy.Rate(1) # 10hz
    R = 10

    while not rospy.is_shutdown():
        pointCloud = PointCloud()
        N = int(np.random.randint(2, 100, 1))

        theta = np.random.uniform(0, 2*np.pi, [1,N])

        for t in theta[0]:
            pointCloud.points.append(Point32(R*np.cos(t), R*np.sin(t), 0))

        hello_str = "points: {}".format(pointCloud.points)  # % theta % N
        rospy.loginfo(hello_str)
        pub.publish(pointCloud)
        rate.sleep()




if __name__ == '__main__':
    publisher()