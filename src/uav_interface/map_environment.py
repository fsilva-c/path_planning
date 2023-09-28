import numpy as np
import sensor_msgs.point_cloud2 as pc2
from scipy.optimize import minimize
from sklearn.cluster import DBSCAN
from geometry.laser_geometry import LaserProjection
from uav_interface.uav_info import UAVInfo

class MapEnvironment:
    def __init__(self, uav_id: int=1) -> None:
        self.uav_id = uav_id
        self.uav_info = UAVInfo(uav_id)
    
    def get_obstacles_rplidar(self): # rplidar...
        laser_scan = self.uav_info.get_laser_scan()
        pc2_msg = LaserProjection().projectLaser(laser_scan)
        for p in pc2.read_points(pc2_msg, field_names=('x', 'y'), skip_nans=True):
            yield p
    
    def get_tree_clusters(self): # pegar as árvores de forma agrupada...
        points = list(self.get_obstacles_rplidar())
        dbscan = DBSCAN(eps=0.5)
        dbscan.fit(points)
        labels = dbscan.labels_
        clusters = {}
        for i, label in enumerate(labels):
            if label not in clusters:
                clusters[label] = []
            clusters[label].append(points[i])
        return clusters
    
    def expand_obstacles(self):
        obstacles = []

        # Função de erro para minimização
        def erro_circulo(params, pontos):
            cx, cy, r = params
            erro = 0
            for x, y in pontos:
                erro += ((x - cx) ** 2 + (y - cy) ** 2 - r ** 2) ** 2
            return erro

        for _, cluster_points in self.get_tree_clusters().items():
            points_x, points_y = zip(*cluster_points)
            points = np.array([points_x, points_y]).T

            # Estimativa inicial do centro e raio (média das coordenadas)
            cx_initial = np.mean(points[:, 0])
            cy_initial = np.mean(points[:, 1])
            r_initial = np.mean(np.sqrt((points[:, 0] - cx_initial) ** 2 + (points[:, 1] - cy_initial) ** 2))

            # Minimização para encontrar o centro e o raio
            resultado = minimize(erro_circulo, [cx_initial, cy_initial, r_initial], args=(points,), method='L-BFGS-B')

            # Extrair os parâmetros do círculo otimizado
            cx_optimized, cy_optimized, r_optimized = resultado.x
            center_x, center_y = cx_optimized, cy_optimized
            raio = r_optimized
            n = 10
            angles = np.linspace(0, 2 * np.pi, n)

            # Calcula as coordenadas x e y dos pontos no círculo
            x_points = center_x + raio * np.cos(angles)
            y_points = center_y + raio * np.sin(angles)

            # Ângulos igualmente espaçados em torno do círculo
            angles = np.linspace(0, 2 * np.pi, n)
            inclination = np.linspace(0, np.pi, n)

            # Simulando a esfera
            theta, phi = np.meshgrid(angles, inclination)

            x_points = center_x + raio * np.sin(phi) * np.cos(theta)
            y_points = center_y + raio * np.sin(phi) * np.sin(theta)
            z_pontos = raio * np.cos(phi)

            # Flatten as coordenadas
            x_points = x_points.flatten()
            y_points = y_points.flatten()
            z_pontos = z_pontos.flatten()
            obstacles.extend(list(zip(x_points, y_points, z_pontos)))

        return obstacles