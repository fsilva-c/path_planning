import rospy
import numpy as np
from uav.uav_info import UAVInfo
from mrs_msgs.srv import ReferenceStampedSrv, PathSrv, String, Vec1
from std_srvs.srv import Trigger
from mrs_msgs.msg import Reference
from geometry_msgs.msg import Point

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
            rospy.sleep(0.1)

    def set_velocity(self, mode):
        srv_name = f'/uav{self.uav_id}/constraint_manager/set_constraints'
        rospy.wait_for_service(srv_name)
        srv_set_mode = rospy.ServiceProxy(srv_name, String)
        req = String._request_class()
        req.value = mode
        srv_set_mode(req)

    def hover(self):
        srv_name = f'/uav{self.uav_id}/control_manager/hover'
        rospy.wait_for_service(srv_name)
        srv_hover = rospy.ServiceProxy(srv_name, Trigger)
        req = Trigger._request_class()
        srv_hover(req)

    def set_heading(self, heading):
        srv_name = f'/uav{self.uav_id}/control_manager/set_heading'
        rospy.wait_for_service(srv_name)
        srv_set_mode = rospy.ServiceProxy(srv_name, Vec1)
        req = Vec1._request_class()
        req.goal = heading
        srv_set_mode(req)
        dt = 0.1
        while abs(self.uav_info.get_heading() - heading) >= dt:
            pass

    def goto_trajectory(self, trajectory, fly_now=True, wait=False) -> None:
        rospy.loginfo('GOTO trajectory uav...')

        srv_name = f'/uav{self.uav_id}/trajectory_generation/path'
        rospy.wait_for_service(srv_name)
        service_proxy = rospy.ServiceProxy(srv_name, PathSrv)

        points = []
        for point in trajectory:
            reference = Reference()

            if len(point) < 3:
                point = list(point)
                point.append(self.uav_info.get_garmin_range())

            reference.position = Point(point[0], point[1], point[2])
            # reference.heading = 0.0
            points.append(reference)
        
        msg_srv = PathSrv._request_class()
        msg_srv.path.use_heading = True
        msg_srv.path.points = points
        msg_srv.path.fly_now = fly_now

        try:
            service_proxy(msg_srv)
        except rospy.ServiceException as e:
            rospy.logerr(f'Erro ao chamar o serviço {srv_name}: {e}')
        
        if wait:
            while not self.in_target(list(trajectory[-1])):
                rospy.sleep(0.1)

    def switch_controller(self, controller):
        srv_name = f'/uav{self.uav_id}/control_manager/switch_controller'
        rospy.wait_for_service(srv_name)
        srv_set_mode = rospy.ServiceProxy(srv_name, String)
        req = String._request_class()
        req.value = controller
        srv_set_mode(req)

    def in_target(self, target) -> None:
        uav_position = self.uav_info.get_uav_position()
        
        if len(target) < 3: # se goto3D -> ponto envolve o eixo z
            target.append(uav_position.z)

        point_dist = np.linalg.norm(
            np.array([target[0], target[1], target[2]]) - np.array([uav_position.x, uav_position.y, uav_position.z])
        )
        return point_dist <= 0.3 # 30 cm
