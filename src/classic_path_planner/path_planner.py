import rospy
from time import perf_counter
from uav_interface.uav import UAV
from fs_path_planning.msg import SphereCloud
from fs_path_planning.srv import Astar
from geometry.geometry import Geometry
from geometry.discrete_grid import DiscreteGrid
from geometry_msgs.msg import Point

class StatePlanner:
    PLANNING = 1
    MOVING = 2
    OBSTACLE_FOUND = 3
    GOAL_REACHED = 4
    COLLISION = 5

class PathPlanner:
    def __init__(
            self,
            uav_id=1,
            resolution=0.5,
            threshold=0.5
        ) -> None:
        self.uav = UAV(uav_id)
        self.threshold = threshold
        self.dg = DiscreteGrid(resolution)

        self.astar_service = rospy.ServiceProxy('/path_finder', Astar)

    def run(self, goal) -> None:
        self.current_state = StatePlanner.PLANNING # zera o estado...
        uav_position = self.uav.uav_info.get_uav_position(tolist=True)

        rospy.loginfo('[PathPlanner]: Start Path Planning...')
        rospy.loginfo(
            f'[PathPlanner]: Start {uav_position}; Goal: {goal}...'
        )

        while self.current_state != StatePlanner.GOAL_REACHED:
            if self.current_state == StatePlanner.PLANNING:
                self.path_plan(goal)
                self.current_state = StatePlanner.MOVING
            
            elif self.current_state == StatePlanner.MOVING:
                if self.distance_to_closest_obstacle() < self.threshold:
                    self.current_state = StatePlanner.OBSTACLE_FOUND
                if self.uav.movements.in_target(goal):
                    self.current_state = StatePlanner.GOAL_REACHED

            elif self.current_state == StatePlanner.OBSTACLE_FOUND:
                self.current_state = StatePlanner.PLANNING
            rospy.sleep(0.01)

        rospy.loginfo('[PathPlanner]: Finalizado Path Planning...')

    def distance_to_closest_obstacle(self):
        return min(self.uav.uav_info.get_laser_scan().ranges)

    def path_plan(self, goal) -> None:
        rospy.loginfo('[PathPlanner]: Path Planning...')

        uav_position = self.uav.uav_info.get_uav_position(tolist=True)

        rospy.loginfo('[PathPlanner]: Encontrando o caminho...')
        time_start = perf_counter()
        req = Astar._request_class()
        req.start = Point(*uav_position)
        req.goal = Point(*goal)
        resp = self.astar_service(req)
        rospy.loginfo('[PathPlanner]: Caminho encontrado...')
        rospy.loginfo(f'[PathPlanner]: O planejamento levou {round(perf_counter() - time_start, 5)}s para ser concluído...')
        self.uav.movements.goto_trajectory(resp.path.points, fly_now=True, wait=True)
