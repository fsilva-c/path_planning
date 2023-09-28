import subprocess
import time

class ROSWaiter:
    def __init__(self, uav_name):
        self.uav_name = uav_name

    def wait_for_ros(self):
        # print('Waiting for ROS')
        while True:
            try:
                subprocess.check_call(
                    ['timeout', '6s', 'rosparam', 'get', '/run_id'], 
                    stdout=subprocess.DEVNULL, 
                    stderr=subprocess.DEVNULL)
                break
            except subprocess.CalledProcessError:
                # print('Waiting for /run_id')
                time.sleep(1)
        time.sleep(1)

    def wait_for_simulation(self):
        # print('Waiting for simulation')
        while True:
            try:
                subprocess.check_call(
                    ['timeout', '6s', 'rostopic', 'echo', '/gazebo/model_states', '-n', '1', '--noarr'], 
                    stdout=subprocess.DEVNULL, 
                    stderr=subprocess.DEVNULL)
                break
            except subprocess.CalledProcessError:
                # print('Waiting for simulation')
                time.sleep(1)
        time.sleep(1)

    def wait_for_odometry(self):
        # print('Waiting for odometry')
        while True:
            try:
                subprocess.check_call(
                    ['timeout', '6s', 'rostopic', 'echo', f'/{self.uav_name}/mavros/local_position/odom', '-n', '1', '--noarr'], 
                    stdout=subprocess.DEVNULL, 
                    stderr=subprocess.DEVNULL)
                break
            except subprocess.CalledProcessError:
                # print('Waiting for odometry')
                time.sleep(1)

    def wait_for_control(self):
        # print('Waiting for control')
        while True:
            try:
                subprocess.check_call(
                    ['timeout', '6s', 'rostopic', 'echo', f'/{self.uav_name}/control_manager/diagnostics', '-n', '1', '--noarr'], 
                    stdout=subprocess.DEVNULL, 
                    stderr=subprocess.DEVNULL)
                break
            except subprocess.CalledProcessError:
                # print('Waiting for control')
                time.sleep(1)
        while True:
            try:
                subprocess.check_call(
                    ['timeout', '6s', 'rostopic', 'echo', f'/{self.uav_name}/odometry/odom_main', '-n', '1', '--noarr'], 
                    stdout=subprocess.DEVNULL, 
                    stderr=subprocess.DEVNULL)
                break
            except subprocess.CalledProcessError:
                # print('Waiting for odom_main')
                time.sleep(1)

    def wait_for_offboard(self):
        # print('Waiting for offboard mode')
        while True:
            try:
                subprocess.check_call(
                    ['timeout', '6s', 'rostopic', 'echo', f'/{self.uav_name}/control_manager/offboard_on', '-n', '1', '--noarr'], 
                    stdout=subprocess.DEVNULL, 
                    stderr=subprocess.DEVNULL)
                break
            except subprocess.CalledProcessError:
                # print('Waiting for offboard mode')
                time.sleep(1)
                