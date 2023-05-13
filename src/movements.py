import rospy
import numpy as np
from uavinfo import UAVInfo
from mrs_msgs.srv import ReferenceStampedSrv, TrajectoryReferenceSrv
from geometry_msgs.msg import Point
from trajectory import list_to_trajectory

class Movements:
    def __init__(self, uav_id=1) -> None:
        self.uav_id = uav_id

        self.uav_info = UAVInfo(uav_id)

    def goto(self, target, heading=0.0) -> None:
        rospy.loginfo('GOTO uav...')

        srv_name = f'/uav{self.uav_id}/control_manager/reference'
        rospy.wait_for_service(srv_name)
        service_proxy = rospy.ServiceProxy(srv_name, ReferenceStampedSrv)

        if len(target) < 3: # se goto3D -> ponto envolve o eixo z
            target.append(self.uav_info.get_uav_position().z)
        
        msg_srv = ReferenceStampedSrv._request_class()
        msg_srv.reference.position = Point(target[0], target[1], target[2])
        msg_srv.reference.heading = heading

        try:
            service_proxy(msg_srv)
        except rospy.ServiceException as e:
            rospy.logerr(f'Erro ao chamar o serviço {srv_name}: {e}')

        while not self.in_target(target):
            pass

    def goto_trajectory(self, trajectory, loop=False) -> None:
        rospy.loginfo('GOTO trajectory uav...')

        srv_name = f'/uav{self.uav_id}/control_manager/trajectory_reference'
        rospy.wait_for_service(srv_name)
        service_proxy = rospy.ServiceProxy(srv_name, TrajectoryReferenceSrv)
        
        msg_srv = TrajectoryReferenceSrv._request_class()
        msg_srv.trajectory.fly_now = True
        msg_srv.trajectory.loop = loop
        msg_srv.trajectory.points = list_to_trajectory(trajectory)

        try:
            service_proxy(msg_srv)
        except rospy.ServiceException as e:
            rospy.logerr(f'Erro ao chamar o serviço {srv_name}: {e}')
        
        if not loop:
            while not self.in_target(trajectory[-1]):
                rospy.sleep(0.1)

    def in_target(self, target) -> None:
        uav_position = self.uav_info.get_uav_position()
        point_dist = np.linalg.norm(
            np.array([target[0], target[1], target[2]]) - np.array([uav_position.x, uav_position.y, uav_position.z])
        )
        return point_dist <= 0.3 # 30 cm
