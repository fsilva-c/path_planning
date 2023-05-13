#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from uav import UAV
from hector import Hector
from astar import Astar
from path_planner import PathPlanner

uav = UAV(uav_id=1)
hector = Hector()
pp = PathPlanner(goal=(0, 0))

def start_astar():
    rospy.init_node('test_octomap', anonymous=True)

    rospy.loginfo('Iniciando os testes...')
    rospy.sleep(5.0)

    rospy.loginfo('Aguardando carregamento dos módulos do controlador de voo...')
    while rospy.get_time() <= 40.0 or uav.uav_info.get_active_tracker() == 'NullTracker':
        rospy.sleep(0.1)

    rospy.loginfo('Iniciando os testes...')
    pp.run()

start_astar()
