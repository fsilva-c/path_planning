import rospy
from mrs_msgs.srv import ReferenceStampedSrv
from geometry_msgs.msg import Point

class Movements:
    def __init__(self, uav_id=1) -> None:
        self.uav_id = uav_id

    def goto(self, target: Point, heading=0.0) -> None:
        srv_name = f'/uav{self.uav_id}/control_manager/reference'
        rospy.wait_for_service(srv_name)
        srv_pos = rospy.ServiceProxy(srv_name, ReferenceStampedSrv)
        
        msg_srv = ReferenceStampedSrv._request_class()
        msg_srv.reference.position = target
        msg_srv.reference.heading = heading

        try:
            srv_pos(msg_srv)
        except rospy.ServiceException as e:
            rospy.logerr(f'Erro ao chamar o serviço {srv_name}: {e}')
