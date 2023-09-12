#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from uav_info import UAVInfo
from geometry_msgs.msg import Point32
from sensor_msgs.msg import LaserScan, PointCloud
from laser_geometry import LaserProjection

uav_info = UAVInfo(1)

class FSPP_RPLidar:
    def __init__(self) -> None:
        self.laser_scan = LaserScan()
        
        self.pub = rospy.Publisher('/fspp_classical/rplidar', PointCloud, queue_size=10)
        rospy.Subscriber(f'/uav1/rplidar/scan', LaserScan, self.callback_laser_scan)

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            uav_position = uav_info.get_uav_position()
            points = PointCloud()
            points.header.stamp = rospy.Time.now()
            for obstacle in self.obstacles_rplidar():
                for z in np.linspace(uav_position.z - 1.0, uav_position.z + 2.0, num=3):
                    point = Point32()
                    point.x, point.y = obstacle
                    point.z = z
                    points.points.append(point)
            self.pub.publish(points)
            rate.sleep()

    def callback_laser_scan(self, msg):
        self.laser_scan = msg

    def obstacles_rplidar(self):
        pc2_msg = LaserProjection().projectLaser(self.laser_scan)
        for p in pc2.read_points(pc2_msg, field_names=('x', 'y'), skip_nans=True):
            yield p

if __name__ == '__main__':
    rospy.init_node('fspp_classical_rplidar')
    rospy.sleep(5)
    rospy.loginfo('Iniciando o rplidar...')
    while rospy.get_time() <= 15.0 or uav_info.get_active_tracker() == 'NullTracker':
        rospy.sleep(0.1)
    FSPP_RPLidar()
