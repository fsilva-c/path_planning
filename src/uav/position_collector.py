#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from time import perf_counter
from gazebo_msgs.msg import ModelStates

class PositionCollector:
    def __init__(self) -> None:
        self.is_collect = False
        self.start_time = None

        rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback)

    def model_states_callback(self, data):
        if self.is_collect:
            model_name = 'uav1'
            if model_name in data.name:
                index = data.name.index(model_name)
                position = data.pose[index].position
                
                x = position.x
                y = position.y
                z = position.z

                with open(f'uav1-{self.start_time}.txt', 'a') as f:
                    f.write(f'{x}, {y}, {z}\n')

    def start_collecting(self):
        self.is_collect = True
        self.start_time = perf_counter()
        rospy.loginfo("[PositionCollector]: Iniciando a coleta de posições do drone")

    def stop_collecting(self):
        self.is_collect = False
        rospy.loginfo("[PositionCollector]: Parando a coleta de posições do drone")

if __name__ == "__main__":
    rospy.init_node('position_collector')
    try:
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            PositionCollector()
            rate.sleep()
        # rospy.spin()
    except rospy.ROSInterruptException:
        pass
