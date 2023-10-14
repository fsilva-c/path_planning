import rospy
from time import perf_counter
from uav_interface.uav import UAV
from fs_path_planning.srv import Astar
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
            uav_radius=0.5
        ) -> None:
        self.uav = UAV(uav_id)
        self.uav_radius = uav_radius
        self.dg = DiscreteGrid(resolution)

        self.n_replan = 0

        self.astar_service = rospy.ServiceProxy('/path_finder', Astar)

    def run(self, goal) -> None:
        self.current_state = StatePlanner.PLANNING # zera o estado...
        uav_position = self.uav.uav_info.get_uav_position(tolist=True)
        rospy.loginfo(
            f'[PathPlanner]: Start Path Planning... {uav_position}; Goal: {goal}...'
        )

        while self.current_state != StatePlanner.GOAL_REACHED or self.current_state == StatePlanner.COLLISION:
            if self.current_state == StatePlanner.PLANNING:
                self.path_plan(goal)
                rospy.sleep(10)
                self.current_state = StatePlanner.MOVING
            
            elif self.current_state == StatePlanner.MOVING:
                if self.distance_to_closest_obstacle() < self.uav_radius * 2:
                    rospy.loginfo('[PathPlanner]: Obstáculo próximo... Recalculando a rota...')
                    self.uav.movements.hover()
                    rospy.sleep(1)
                    self.current_state = StatePlanner.OBSTACLE_FOUND
                if self.uav.movements.in_target(goal):
                    self.current_state = StatePlanner.GOAL_REACHED
                    
            elif self.current_state == StatePlanner.OBSTACLE_FOUND:
                self.current_state = StatePlanner.PLANNING
                self.n_replan += 1

            if self.uav.uav_info.get_active_tracker() == 'NullTracker':
                rospy.loginfo('[PathPlanner]: RTT!!! COLISÃO!!!')
                self.current_state = StatePlanner.COLLISION
            
            rospy.sleep(0.01)

        rospy.loginfo('[PathPlanner]: Finalizado Path Planning...')
        rospy.loginfo(f'[PathPlanner]: Replanejou o caminho {self.n_replan} vezes...')

    def distance_to_closest_obstacle(self):
        return min(self.uav.uav_info.get_laser_scan().ranges)

    def path_plan(self, goal) -> None:
        rospy.loginfo('[PathPlanner]: Path Planning... Encontrando o caminho...')
        uav_position = self.uav.uav_info.get_uav_position(tolist=True)
        time_start = perf_counter()
        req = Astar._request_class()
        req.start = Point(*uav_position)
        req.goal = Point(*goal)
        resp = self.astar_service(req)
        rospy.loginfo(f'[PathPlanner]: Caminho encontrado... O planejamento levou {round(perf_counter() - time_start, 5)}s para ser concluído...')
        self.uav.movements.goto_trajectory(resp.path.points, fly_now=True)
