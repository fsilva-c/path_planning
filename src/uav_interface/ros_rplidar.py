#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from uav_info import UAVInfo
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PointStamped
from laser_geometry import LaserProjection

uav_info = UAVInfo(1)

class FSPP_RPLidar:
    def __init__(self) -> None:
        self.laser_scan = LaserScan()
        
        self.pub = rospy.Publisher('/fspp_classical/rplidar', PointStamped, queue_size=10)
        rospy.Subscriber(f'/uav1/rplidar/scan', LaserScan, self.callback_laser_scan)

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            uav_position = uav_info.get_uav_position()
            for obstacle in self.obstacles_rplidar():
                point = PointStamped()
                point.header.stamp = rospy.Time.now()
                point.point.x, point.point.y = obstacle
                point.point.z = uav_position.z
                self.pub.publish(point)
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
    while rospy.get_time() <= 25.0 or uav_info.get_active_tracker() == 'NullTracker':
        rospy.sleep(0.1)
    FSPP_RPLidar()
