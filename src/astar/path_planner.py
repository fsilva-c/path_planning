import rospy
from geometry.geometry import Geometry
import sensor_msgs.point_cloud2 as pc2
from time import perf_counter
from uav.uav import UAV
from geometry.discrete_grid import DiscreteGrid
from astar.astar import AStar

class PathPlanner:
    def __init__(self, goal, uav_id=1) -> None:
        self.goal = goal
        self.uav_id = uav_id
        self.dg = DiscreteGrid(resolution=0.5)
        self.uav = UAV(uav_id)
        self.start = None

    def run(self):
        uav_position = self.uav.uav_info.get_uav_position()

        rospy.loginfo('[PathPlanner]: Start Path Planning...')
        rospy.loginfo(
            f'[PathPlanner]: Start {uav_position.x, uav_position.y, uav_position.z}; Goal: {self.goal}...'  # noqa: E501
        )

        self.path_plan()

        while not self.uav.movements.in_target([self.goal[0], self.goal[1]]):
            if self.distance_to_start() >= 2.0:
                self.path_plan()
            rospy.sleep(0.1)

    def distance_to_start(self):
        uav_position = self.uav.uav_info.get_uav_position()
        return Geometry.norm(self.start, [uav_position.x, uav_position.y, uav_position.z])

    def path_plan(self):
        rospy.loginfo('[PathPlanner]: Path Planning...')
        time_start = perf_counter()

        uav_position = self.uav.uav_info.get_uav_position()
        self.start = (uav_position.x, uav_position.y, uav_position.z)

        # gira o drone pra frente do alvo...
        # dif_x = self.goal[0] - uav_position.x
        # dif_y = self.goal[1] - uav_position.y
        # ang_rad = math.atan2(dif_x, dif_y)
        # ang_rad = (ang_rad + math.pi) % (2 * math.pi) - math.pi
        # print(math.degrees(ang_rad - self.uav.uav_info.get_heading()))
        # return
        # self.uav.movements.set_heading()

        # get obstacles...
        obstacles = []
        point_cloud_2 = self.uav.uav_info.get_point_cloud_2()
        for p in pc2.read_points(point_cloud_2, field_names=('x', 'y', 'z'), skip_nans=True):
            x, y, z = p
            obstacles.append(self.dg.continuous_to_discrete([z, -x, y + uav_position.z]))
            # print([z, -x, y + uav_position.z])

        # astar = AStar(grid_map=self.discrete_grid, obstacles=obstacles)
        astar = AStar(obstacles=obstacles)
        path = astar.find_path(
            start=self.start,
            goal=self.goal
        )

        path.append(self.goal)
        print('path: ', path)
        rospy.loginfo(f'[PathPlanner]: O planejamento levou {round(perf_counter() - time_start, 5)}s para ser concluído...')        
        self.uav.movements.goto_trajectory(path, fly_now=True)
