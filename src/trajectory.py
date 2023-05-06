from geometry_msgs.msg import Point
from mrs_msgs.msg import Reference

def list_to_trajectory(points):
    trajectory = []

    for point in points:
        reference = Reference()
        reference.position = Point(point[0], point[1], point[2])
        reference.heading = 0.0
        trajectory.append(reference)

    return trajectory

def load(file_path):
    trajectory = []

    with open(file_path) as f:
        for line in f.readlines():
            point = line.split()
            trajectory.append([float(point[0]), float(point[2]), float(point[2])])

    return trajectory
