#!/usr/bin/env python3
# -*- coding: utf-8 -*-

'''
# yml...
-gazebo: waitForRos; roslaunch mrs_simulation simulation.launch gui:=false 
-spawn: waitForSimulation; rosservice call /mrs_drone_spawner/spawn "1 $UAV_TYPE --enable-rangefinder --enable-rplidar --pos_file `pwd`/pos.yaml"
-control: waitForOdometry; roslaunch mrs_uav_general core.launch
-takeoff: waitForSimulation; roslaunch mrs_uav_general automatic_start.launch
-takeoff: 'waitForControl;
    rosservice call /$UAV_NAME/mavros/cmd/arming 1;
    sleep 2;
    rosservice call /$UAV_NAME/mavros/set_mode 0 offboard'
'''

import os
import subprocess
import rospy
from uav.uav import UAV
# os.system('roscore')

uav = UAV(uav_id=1)

def start():
    os.environ['UAV_NAME'] = 'uav1'
    os.environ['RUN_TYPE'] = 'simulation'
    os.environ['UAV_TYPE'] = 'f450'
    os.environ['WORLD_NAME'] = 'simulation_local'
    os.environ['SENSORS'] = 'garmin_down'
    os.environ['ODOMETRY_TYPE'] = 'gps'
    os.environ['PX4_SIM_SPEED_FACTOR'] = '1'

    roscore = subprocess.Popen(['roscore'])
    rospy.sleep(2.0)

    rospy.init_node('rl_mission', anonymous=True)

    start_nodes() # inicia o mrs...

    rospy.loginfo('Aguardando carregamento dos módulos do controlador de voo...')
    while rospy.get_time() <= 25.0 or uav.uav_info.get_active_tracker() == 'NullTracker':
        rospy.sleep(0.1)

    rospy.loginfo('Iniciando os testes...')
    rospy.sleep(5.0)

    print(uav.uav_info.get_uav_position())
    # uav.movements.goto([5.0, 1.0, 2.0])

    # teste matar e iniciar novamente...
    # kill_nodes()
    subprocess.call(['sh', './kill.sh'])
    rospy.sleep(2.0)
    start_nodes() # inicia o mrs...
    rospy.loginfo('Aguardando carregamento dos módulos do controlador de voo...')
    while rospy.get_time() <= 25.0 or uav.uav_info.get_active_tracker() == 'NullTracker':
        rospy.sleep(0.1)
    rospy.loginfo('Iniciando os testes...')
    rospy.sleep(5.0)
    print(uav.uav_info.get_uav_position())
    # uav.movements.goto([1.0, 1.0, 2.0])
    subprocess.call(['sh', './kill.sh'])
    rospy.sleep(2.0)
    roscore.terminate()

def start_nodes():
    subprocess.Popen(
        'roslaunch mrs_simulation simulation.launch gui:=true', 
        shell=True,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.STDOUT)
    rospy.sleep(5.0)
    subprocess.Popen(
        'rosservice call /mrs_drone_spawner/spawn "1 $UAV_TYPE --enable-rangefinder --enable-rplidar --pos 0 0 1 0"', 
        shell=True,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.STDOUT)
    subprocess.Popen('roslaunch mrs_uav_general automatic_start.launch', 
        shell=True,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.STDOUT)
    rospy.sleep(10.0)
    subprocess.Popen(
        'roslaunch mrs_uav_general core.launch', 
        shell=True,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.STDOUT)
    rospy.sleep(5.0)
    subprocess.Popen('rosservice call /$UAV_NAME/mavros/cmd/arming 1; sleep 2; rosservice call /$UAV_NAME/mavros/set_mode 0 offboard', 
        shell=True,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.STDOUT)
    rospy.sleep(5.0)

def kill_nodes():
    subprocess.call(['sh', './kill.sh'])
    rospy.sleep(2.0)

start()
