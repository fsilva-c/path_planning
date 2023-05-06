#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Point
from uav import UAV

def start():
    rospy.init_node('test_env', anonymous=True)
    rospy.loginfo('Iniciando os testes...')
    rospy.sleep(5)

    uav = UAV(uav_id=1)

    target = Point(10.0, 10.0, 3.0)
    uav.movements.goto(target)

start()
