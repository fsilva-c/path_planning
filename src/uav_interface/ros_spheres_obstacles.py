#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from uav import UAV
from fs_path_planning.msg import SphereCloud
from fs_path_planning.msg import Sphere

uav = UAV(uav_id=1)

class  FSPP_SphereCloud:
     def __init__(self) -> None:
        self.pub = rospy.Publisher('/fspp_classical/spheres_cloud', SphereCloud, queue_size=10)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            sphere_cloud = SphereCloud()
            for sc in uav.map_environment.get_sphere_cloud():
                s1 = Sphere()
                s1.center.x = sc[0]
                s1.center.y = sc[1]
                s1.center.z = sc[2]
                s1.radius = sc[3]
                sphere_cloud.spheres.append(s1)
            self.pub.publish(sphere_cloud)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('fspp_classical_sphere_cloud')
    rospy.sleep(5)
    rospy.loginfo('Iniciando o rplidar...')
    while rospy.get_time() <= 15.0 or uav.uav_info.get_active_tracker() == 'NullTracker':
        rospy.sleep(0.1)
    FSPP_SphereCloud()
