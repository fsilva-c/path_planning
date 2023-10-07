#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from uav_interface.uav import UAV
from uav_interface.position_collector import PositionCollector
from classic_path_planner.path_planner import PathPlanner

uav = UAV(uav_id=1)
pp = PathPlanner(goal=[-2.5, 15, 2.0])
collector = PositionCollector()

def start():
    rospy.init_node('astar_mission', anonymous=True)

    rospy.loginfo('Iniciando os testes...')
    rospy.sleep(5.0)

    rospy.loginfo('Aguardando carregamento dos módulos do controlador de voo...')
    while rospy.get_time() <= 25.0 or uav.uav_info.get_active_tracker() == 'NullTracker':
        rospy.sleep(0.1)

    # collector.start_collecting()
    # collector.stop_collecting()
    # pp.run()
    from time import perf_counter
    from fs_path_planning.srv import Astar
    from geometry_msgs.msg import Point
    rospy.wait_for_service('/path_finder')
    try:
        astar_service = rospy.ServiceProxy('/path_finder', Astar, persistent=True)
        req = Astar._request_class()
        start = perf_counter()
        req.start = Point(0, 0, 1.22)
        req.goal = Point(-12.5, -10, 1.8)
        resp = astar_service(req)
        uav.movements.goto_trajectory(resp.path.points, fly_now=False)
        print(perf_counter() - start)
    except rospy.ServiceException as e:
        print("Erro ao chamar o serviço 'astar':", e)

start()
