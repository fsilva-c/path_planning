#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from time import perf_counter
from mrs_msgs.msg import PositionCommand

# local que ficam os arquivos:
# /home/fs/.ros/

class PositionCollector:
    def __init__(self) -> None:
        self.is_collect = False
        self.start_time = None

        rospy.Subscriber(f'/uav1/control_manager/position_cmd', PositionCommand, self.callback_position)

    def callback_position(self, data):
        if self.is_collect:
            x = data.position.x
            y = data.position.y
            z = data.position.z
            with open(f'uav1-{self.start_time}.csv', 'a') as f:
                f.write(f'{rospy.get_time()}, {x}, {y}, {z}\n')

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
