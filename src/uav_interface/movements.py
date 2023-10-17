import rospy
from uav_interface.uav_info import UAVInfo
from geometry.geometry import Geometry
from mrs_msgs.srv import ReferenceStampedSrv, PathSrv, String, Vec1, VelocityReferenceStampedSrv
from std_srvs.srv import Trigger, SetBool
from mavros_msgs.srv import CommandBool, SetMode
from mrs_msgs.msg import Reference
from geometry_msgs.msg import Point

class Movements:
    def __init__(self, uav_id: int=1) -> None:
        self.uav_id = uav_id

        self.uav_info = UAVInfo(uav_id)

        # msgs_srv
        self.srv_goto_trajectory_name = f'/uav{self.uav_id}/trajectory_generation/path'
        self.srv_apply_velocity_name = f'/uav{self.uav_id}/control_manager/velocity_reference'
        # rospy.wait_for_service(self.srv_goto_trajectory_name)
        self.srv_velocity_reference = rospy.ServiceProxy(self.srv_apply_velocity_name, VelocityReferenceStampedSrv)
        self.service_goto_trajectory = rospy.ServiceProxy(self.srv_goto_trajectory_name, PathSrv)

    def goto(self, target: list, heading: float=0.0) -> None:
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

    def takeoff(self):
        # rospy.loginfo("DECOLANDO UAV...")
            
        srv_name = f'/uav{self.uav_id}/uav_manager/takeoff'

        if self.uav_info.get_active_tracker() == "MpcTracker":
            return

        # checa se está em condições favoráveis a decolagem...
        # while not self.uav_info.get_motors_status():
        #     self.motors(True)

        while not self.uav_info.is_armed():
            self.arming(True)

        while self.uav_info.get_mode() != "OFFBOARD":
            self.set_mode(0, "OFFBOARD")

        rospy.wait_for_service(srv_name)
        srv_takeoff = rospy.ServiceProxy(srv_name, Trigger)
        req = Trigger._request_class()
        try:
            srv_takeoff(req)
        except rospy.ServiceException as e:
            rospy.logerr(f'Erro ao chamar o serviço {srv_name}: {e}')

        # "aguarde" enquanto não decolar...
        while self.uav_info.get_active_tracker() != "MpcTracker":
            rospy.sleep(0.1)
        
        # rospy.loginfo("UAV DECOLADO...")

    def land(self):
        rospy.loginfo("POUSANDO UAV...")

        # checa se está em condições favoráveis o pouso...
        if self.uav_info.get_active_tracker() != "MpcTracker":
            return

        srv_name = f'/uav{self.uav_id}/uav_manager/land'
        rospy.wait_for_service(srv_name)
        srv_land = rospy.ServiceProxy(srv_name, Trigger)
        req = Trigger._request_class()
        srv_land(req)
        
        # "aguarde" enquanto não pousar...
        while self.uav_info.get_active_tracker() != "NullTracker":
            rospy.sleep(0.1)

        self.motors(False) # desativa os motores...

    def apply_velocity(self, velocity):
        # rospy.wait_for_service(srv_name)
        req = VelocityReferenceStampedSrv._request_class()
        req.reference.reference.velocity = velocity
        try:
            self.srv_velocity_reference(req)
            # rospy.sleep(0.25)
        except rospy.ServiceException as e:
            rospy.logerr(f'Erro ao chamar o serviço {self.srv_apply_velocity_name}: {e}')

    def motors(self, status):
        srv_name = f'/uav{self.uav_id}/control_manager/motors'
        rospy.wait_for_service(srv_name)
        srv_motors = rospy.ServiceProxy(srv_name, SetBool)
        req = SetBool._request_class()
        req.data = status
        try:
            srv_motors(req)
        except rospy.ServiceException as e:
            rospy.logerr(f'Erro ao chamar o serviço {srv_name}: {e}')

    def arming(self, status):
        srv_name = f'/uav{self.uav_id}/mavros/cmd/arming'
        rospy.wait_for_service(srv_name)
        srv_arming = rospy.ServiceProxy(srv_name, CommandBool)
        req = CommandBool._request_class()
        req.value = status
        try:
            srv_arming(req)
        except rospy.ServiceException as e:
            rospy.logerr(f'Erro ao chamar o serviço {srv_name}: {e}')

    def set_mode(self, base_mode, custom_mode):
        srv_name = f'/uav{self.uav_id}/mavros/set_mode'
        rospy.wait_for_service(srv_name)
        srv_set_mode = rospy.ServiceProxy(srv_name, SetMode)
        req = SetMode._request_class()
        req.base_mode = base_mode
        req.custom_mode = custom_mode
        try:
            srv_set_mode(req)
        except rospy.ServiceException as e:
            rospy.logerr(f'Erro ao chamar o serviço {srv_name}: {e}')

    def set_velocity(self, mode) -> None:
        srv_name = f'/uav{self.uav_id}/constraint_manager/set_constraints'
        rospy.wait_for_service(srv_name)
        srv_set_mode = rospy.ServiceProxy(srv_name, String)
        req = String._request_class()
        req.value = mode
        try:
            srv_set_mode(req)
        except rospy.ServiceException as e:
            rospy.logerr(f'Erro ao chamar o serviço {srv_name}: {e}')

    def set_heading(self, heading):
        srv_name = f'/uav{self.uav_id}/control_manager/set_heading'
        rospy.wait_for_service(srv_name)
        srv_set_mode = rospy.ServiceProxy(srv_name, Vec1)
        req = Vec1._request_class()
        req.goal = heading
        srv_set_mode(req)
        dt = 0.1
        while abs(self.uav_info.get_heading() - heading) >= dt:
            rospy.sleep(0.1)

    def goto_trajectory(self, trajectory, fly_now=True, wait=False) -> None:
        rospy.loginfo('GOTO trajectory uav...')

        msg_srv = PathSrv._request_class()
        msg_srv.path.use_heading = False
        msg_srv.path.points = [Reference(p, 0.0) for p in trajectory]
        msg_srv.path.fly_now = fly_now

        try:
            self.service_goto_trajectory(msg_srv)
        except rospy.ServiceException as e:
            rospy.logerr(f'Erro ao chamar o serviço {self.srv_goto_trajectory_name}: {e}')
        
        if wait:
            last_point = trajectory[-1]
            while not self.in_target([last_point.x, last_point.y, last_point.z]):
                rospy.sleep(0.1)

    def switch_controller(self, controller):
        srv_name = f'/uav{self.uav_id}/control_manager/switch_controller'
        rospy.wait_for_service(srv_name)
        srv_set_mode = rospy.ServiceProxy(srv_name, String)
        req = String._request_class()
        req.value = controller
        srv_set_mode(req)

    def hover(self):
        srv_name = f'/uav{self.uav_id}/control_manager/hover'
        rospy.wait_for_service(srv_name)
        srv_hover = rospy.ServiceProxy(srv_name, Trigger)
        req = Trigger._request_class()
        srv_hover(req)
        rospy.sleep(1)

    def in_target(self, target) -> None:
        uav_position = self.uav_info.get_uav_position(tolist=True)
        
        if len(target) < 3: # se goto3D -> ponto envolve o eixo z
            target.append(uav_position.z)

        # 15 cm
        return Geometry.euclidean_distance(uav_position, target) <= 0.15
