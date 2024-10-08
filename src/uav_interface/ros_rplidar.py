#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import sensor_msgs.point_cloud2 as pc2
from uav_interface.uav import UAV
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2

uav = UAV(uav_id=1)

class FSPP_RPLidar:
    def __init__(self) -> None:        
        self.pub = rospy.Publisher('/fspp_classical/rplidar_3D', PointCloud2, queue_size=10)

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            obstacles = uav.map_environment.get_obstacles_3D()
            header = Header()
            header.frame_id = 'uav1/fcu'
            header.stamp = rospy.Time.now()
            pc2obs = pc2.create_cloud_xyz32(header, obstacles)
            self.pub.publish(pc2obs)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('fspp_classical_obstacles_3D')
    rospy.sleep(5)
    rospy.loginfo('Iniciando o rplidar...')
    while rospy.get_time() <= 15.0 or uav.uav_info.get_active_tracker() == 'NullTracker':
        rospy.sleep(0.1)
    FSPP_RPLidar()
