#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from trajectory import load
from uav import UAV
from gazebo import Gazebo
from hector import Hector
from planning import OctoMap
import networkx as nx

uav = UAV(uav_id=1)
gazebo = Gazebo()
hector = Hector()
octomap_mrs = OctoMap()

def start():
    rospy.init_node('test_env', anonymous=True)

    rospy.loginfo('Iniciando os testes...')
    rospy.sleep(5.0)

    uav.movements.goto([0.0, 0.0, 3.0])

    path = load('/home/fs/user_ros_workspace/src/fs_path_planning/trajectories/trajectory3.txt')
    uav.movements.goto_trajectory(path)

    rospy.loginfo('Finalizando os testes...')

def start_test_gazebo():
    rospy.init_node('test_gazebo', anonymous=True)

    rospy.loginfo('Iniciando os testes...')
    rospy.sleep(5.0)

    rospy.loginfo('Aguardando carregamento dos módulos do controlador de voo...')
    while rospy.get_time() <= 40.0 or uav.uav_info.get_active_tracker() == 'NullTracker':
        rospy.sleep(0.1)

    rospy.loginfo('Iniciando os testes...')
    trees = gazebo.get_all_trees()
    size = 101
    matrix = np.zeros((size, size))

    for obstacle in trees:
        x, y = obstacle
        i = 50 - round(x)
        j = 50 - round(y)
        matrix[j][i] = 1

    print(matrix)

def start_slam():
    rospy.init_node('test_octomap', anonymous=True)

    rospy.loginfo('Iniciando os testes...')
    rospy.sleep(5.0)

    rospy.loginfo('Aguardando carregamento dos módulos do controlador de voo...')
    while rospy.get_time() <= 40.0 or uav.uav_info.get_active_tracker() == 'NullTracker':
        rospy.sleep(0.1)

    rospy.loginfo('Iniciando os testes...')

    uav_pos = uav.uav_info.get_uav_position()
    point_cloud = hector.get_point_cloud()
    goal = (20, 0)
    start = (int(uav_pos.x), int(uav_pos.y))

    obstacles = np.array([[point.x, point.y] for point in point_cloud.points])
    # # agrupando os valores próximos...
    rounded_data = np.around(obstacles, decimals=0)
    obstacles = np.unique(rounded_data, axis=0)

    grid = np.zeros((28 + 1, 28 + 1))
    mid_x = grid.shape[0] // 2
    mid_y = grid.shape[1] // 2

    for obstacle in obstacles:
        x = int(obstacle[0] + mid_x)
        y = int(obstacle[1] + mid_y)
        grid[x, y] = 1

    graph = nx.Graph()

    for i in range(grid.shape[0]):
        for j in range(grid.shape[1]):
            if grid[i, j] == 0:
                graph.add_node((i, j))

    for node in graph.nodes():
        i, j = node
        neighbors = [(i-1, j), (i+1, j), (i, j-1), (i, j+1)]
        for neighbor in neighbors:
            if neighbor in graph.nodes():
                graph.add_edge(node, neighbor)

    graph.add_node(goal)
    graph.add_node(start)
    path = nx.astar_path(graph, start, goal)
    
    print(f'Caminho gerado: {path}')
    print(f'Start: {start}, Goal: {goal}')

    uav.movements.goto_trajectory([[point[0], point[1], uav_pos.z] for point in path])

    # for point in path:
    #     uav.movements.goto([point[0], point[1]])
    
# start()
# start_test_gazebo()
# start_octomap()
start_slam()
