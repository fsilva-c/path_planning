import rospy
import sensor_msgs.point_cloud2 as pc2
from classic_path_planner.astar import AStar
from time import perf_counter
from uav_interface.uav import UAV
from sensor_msgs.msg import PointCloud2
from fs_path_planning.msg import SphereCloud
from geometry.geometry import Geometry
from geometry.discrete_grid import DiscreteGrid
from scipy.spatial import KDTree

class StatePlanner:
    PLANNING = 1
    MOVING = 2
    OBSTACLE_FOUND = 3
    GOAL_REACHED = 4
    COLLISION = 5

class PathPlanner:
    def __init__(
            self,
            goal,
            uav_id=1,
            resolution=0.5,
            threshold=0.5
        ) -> None:
        self.goal = goal
        self.uav = UAV(uav_id)
        self.threshold = threshold
        self.dg = DiscreteGrid(resolution)
        self.current_state = StatePlanner.PLANNING
        self.current_path = []
        self.start = []

         # obstaculos...
        self.point_cloud = []
        self.kdtree = None

        rospy.Subscriber('/fspp_classical/rplidar_3D', PointCloud2, self.callback_obstacles)
        rospy.Subscriber('/fspp_classical/spheres_cloud', SphereCloud, self.callback_sphere_cloud)

    def run(self) -> None:
        uav_position = self.uav.uav_info.get_uav_position()

        rospy.loginfo('[PathPlanner]: Start Path Planning...')
        rospy.loginfo(
            f'[PathPlanner]: Start {uav_position.x, uav_position.y, uav_position.z}; Goal: {self.goal}...'  # noqa: E501
        )

        while self.current_state != StatePlanner.GOAL_REACHED:
            if self.current_state == StatePlanner.PLANNING:
                self.path_plan()
                self.current_state = StatePlanner.MOVING
                rospy.sleep(10.0)
            
            elif self.current_state == StatePlanner.MOVING:
                if (not self.free_current_path() and 
                    self.distance_to_closest_obstacle() < self.threshold * 2):
                    self.current_state = StatePlanner.OBSTACLE_FOUND
                if self.uav.movements.in_target(list(self.goal)):
                    self.current_state = StatePlanner.GOAL_REACHED

            elif self.current_state == StatePlanner.OBSTACLE_FOUND:
                self.current_state = StatePlanner.PLANNING
            rospy.sleep(0.01)

        rospy.loginfo('[PathPlanner]: Finalizado Path Planning...')

    def distance_to_closest_obstacle(self):
        uav_position = self.uav.uav_info.get_uav_position(tolist=True)
        kdtree = self.obstacles()
        distance, _ = kdtree.query(uav_position, k=1)
        return distance
    

    def callback_sphere_cloud(self, data: SphereCloud):
        self.sphere_cloud = data



    def callback_obstacles(self, data: PointCloud2) -> None:
        pc_data = pc2.read_points(data, field_names=('x', 'y', 'z'), skip_nans=True)
        self.point_cloud = list(pc_data)
    
    def obstacles(self):
        return KDTree(self.point_cloud)

    def free_current_path(self) -> bool:
        kdtree = self.obstacles()
        result = kdtree.query_ball_point(self.current_path, self.threshold)
        return not any(result)

    def distance_to_start(self) -> float:
        uav_position = self.uav.uav_info.get_uav_position()
        return Geometry.euclidean_distance(self.start, [uav_position.x, uav_position.y, uav_position.z])

    def path_plan(self) -> None:
        rospy.loginfo('[PathPlanner]: Path Planning...')

        uav_position = self.uav.uav_info.get_uav_position()
        self.start = (uav_position.x, uav_position.y, uav_position.z)

        time_start = perf_counter()
        rospy.loginfo('[PathPlanner]: Encontrando o caminho...')
        # kdtree = self.obstacles()
        path = AStar(
            threshold=self.threshold,
            dg=self.dg,
            obstacles=self.sphere_cloud
        ).find_path(start=self.start, goal=self.goal)[2:]

        path.append(self.goal)
        self.current_path = path
        rospy.loginfo('[PathPlanner]: Caminho encontrado...')
        rospy.loginfo(f'[PathPlanner]: O planejamento levou {round(perf_counter() - time_start, 5)}s para ser concluído...')
        self.uav.movements.goto_trajectory(path, fly_now=False)
