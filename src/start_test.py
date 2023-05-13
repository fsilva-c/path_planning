#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from trajectory import load
from uav import UAV
from hector import Hector
from astar import Astar

uav = UAV(uav_id=1)
hector = Hector()

def start():
    rospy.init_node('test_env', anonymous=True)

    rospy.loginfo('Iniciando os testes...')
    rospy.sleep(5.0)

    uav.movements.goto([0.0, 0.0, 3.0])

    path = load('/home/fs/user_ros_workspace/src/fs_path_planning/trajectories/trajectory3.txt')
    uav.movements.goto_trajectory(path)

    rospy.loginfo('Finalizando os testes...')

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
    '''
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
    '''

def start_astar():
    rospy.init_node('test_octomap', anonymous=True)

    rospy.loginfo('Iniciando os testes...')
    rospy.sleep(5.0)

    rospy.loginfo('Aguardando carregamento dos módulos do controlador de voo...')
    while rospy.get_time() <= 40.0 or uav.uav_info.get_active_tracker() == 'NullTracker':
        rospy.sleep(0.1)

    rospy.loginfo('Iniciando os testes...')

    # while True:
    uav_pos = uav.uav_info.get_uav_position()
    point_cloud = hector.get_point_cloud()
    
    # make grid...
    goal = (-12, -10)
    resolution = 1.0 # tamanho de cada célula
    grid_size = 100
    grid = np.zeros((grid_size, grid_size), dtype=np.int8)

    # agrupando os obstáculos próximos...
    obstacles = np.array([[point.x, point.y] for point in point_cloud.points])
    rounded_data = np.around(obstacles)
    obstacles = np.unique(rounded_data, axis=0)

    # conversão para o sistemas de coordendas do grid...
    # usando as coordenadas reais drone...
    rel = grid_size // 2
    drone_pos = (int(round(uav_pos.x / resolution + rel)), int(round(uav_pos.y / resolution + rel)))
    goal_pos = (int(round(goal[0] + rel)), int(round(goal[1] + rel)))

    # adiciona os obstáculos no grid...
    for obstacle in obstacles:
        pos = np.array([uav_pos.x, uav_pos.y])
        pos = np.round(pos, decimals=0)

        x = int(round(obstacle[0] + pos[0] / resolution + grid_size / 2))
        y = int(round(obstacle[1] + pos[1] / resolution + grid_size / 2))

        grid[x, y] = 1

    path = Astar(grid, drone_pos, goal_pos).a_star()
    converted_path = [((p[0] - rel) * resolution, (p[1] - rel) * resolution, uav_pos.z) for p in path]
    print(converted_path)
    print(f'UAV pos: {uav_pos}')

    uav.movements.goto_trajectory(converted_path)

    # if len(converted_path) > 1:
    #     p = converted_path[1]
    #     uav.movements.goto([p[0], p[1]])

    # if uav.movements.in_target([goal[0], goal[1], uav_pos.z]):
    #     break
        
'''
# sem obstaculos
[(0.0, 0.0), (0.0, 1.0), (1.0, 1.0), (1.0, 2.0), (2.0, 2.0), (2.0, 3.0), (3.0, 3.0), (3.0, 4.0), (4.0, 4.0), (4.0, 5.0), (5.0, 5.0), (5.0, 6.0), (6.0, 6.0), (6.0, 7.0), (7.0, 7.0), (7.0, 8.0), (8.0, 8.0), (8.0, 9.0), (9.0, 9.0), (9.0, 10.0), (10.0, 10.0), (10.0, 11.0), (11.0, 11.0), (11.0, 12.0), (12.0, 12.0), (12.0, 13.0), (13.0, 13.0), (13.0, 14.0), (14.0, 14.0), (14.0, 15.0), (15.0, 15.0), (15.0, 16.0), (16.0, 16.0), (16.0, 17.0), (17.0, 17.0), (17.0, 18.0), (18.0, 18.0), (18.0, 19.0), (19.0, 19.0), (19.0, 20.0), (20.0, 20.0)]

# com obstaculos
[(0.0, 0.0), (0.0, 1.0), (1.0, 1.0), (1.0, 2.0), (2.0, 2.0), (2.0, 3.0), (3.0, 3.0), (3.0, 4.0), (4.0, 4.0), (4.0, 5.0), (5.0, 5.0), (5.0, 6.0), (6.0, 6.0), (6.0, 7.0), (6.0, 8.0), (7.0, 8.0), (8.0, 8.0), (8.0, 9.0), (9.0, 9.0), (9.0, 10.0), (10.0, 10.0), (10.0, 11.0), (11.0, 11.0), (11.0, 12.0), (12.0, 12.0), (12.0, 13.0), (13.0, 13.0), (13.0, 14.0), (14.0, 14.0), (14.0, 15.0), (15.0, 15.0), (15.0, 16.0), (16.0, 16.0), (16.0, 17.0), (17.0, 17.0), (17.0, 18.0), (18.0, 18.0), (18.0, 19.0), (19.0, 19.0), (19.0, 20.0), (20.0, 20.0)]

# código aparentemente funcional, mas deu loop.. ajeitar
while True:
uav_pos = uav.uav_info.get_uav_position()
point_cloud = hector.get_point_cloud()

# make grid...
goal = (10, 10)
resolution = 1.0
grid_size = 200
grid = np.zeros((grid_size, grid_size), dtype=np.int8)

# agrupando os obstáculos próximos...
obstacles = np.array([[point.x, point.y] for point in point_cloud.points])
rounded_data = np.around(obstacles, decimals=0)
obstacles = np.unique(rounded_data, axis=0)
# print(obstacles)

# usando as coordenadas reais drone...
rel = grid_size // 2
drone_pos = (int(round(uav_pos.x / resolution + rel)), int(round(uav_pos.y / resolution + rel)))
goal_pos = (int(round(goal[0] + rel)), int(round(goal[1] + rel)))

# adiciona os obstáculos no grid...
for obstacle in obstacles:
    pos = np.array([uav_pos.x, uav_pos.y])
    pos = np.round(pos, decimals=0)

    x = int(round(obstacle[0] + pos[0] / resolution + grid_size / 2))
    y = int(round(obstacle[1] + pos[1] / resolution + grid_size / 2))

    grid[x, y] = 1

path = Astar(grid, drone_pos, goal_pos).a_star()
converted_path = [((p[0] - rel) * resolution, (p[1] - rel) * resolution) for p in path]
print(converted_path)
print(f'UAV pos: {uav_pos}')

if len(converted_path) > 1:
    p = converted_path[1]
    uav.movements.goto([p[0], p[1]])

if uav.movements.in_target([goal[0], goal[1], uav_pos.z]):
    break
'''

# start_slam()
start_astar()
