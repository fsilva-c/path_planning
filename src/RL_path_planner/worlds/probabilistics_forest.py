import pathlib
import numpy as np
import xml.etree.ElementTree as ET

filepath = pathlib.Path(__file__).resolve().parent

MINR, MAXR = -50.0, 50.0
MINZ, MAXZ = 0.0, 6.0
N_TREES = 500
SAFE_DIST = 2.0

def find_safe_points(coords, safe_dist=2.0, dist=3.0, delta=0.1):
    while True:
        points = np.random.uniform(MINR, MAXR, (2, 2))
        dists = np.sqrt(np.sum((coords[:, np.newaxis, :] - points) ** 2, axis=2))
        if np.all(dists >= safe_dist) and np.abs(np.linalg.norm(points[0] - points[1]) - dist) <= delta:
            return tuple(points)

def apply_distribution():
    tree = ET.parse(f'{filepath}/base/forest.world')
    root = tree.getroot()
    heights = np.random.uniform(MINZ, MAXZ, N_TREES)
    coords = np.random.uniform(MINR, MAXR, (N_TREES, 2))
    for i, (model, (x, y)) in enumerate(zip(root.findall('.//model'), coords)):
        if model.get('name').startswith('tree'):
            pose_element = model.find('pose')
            if pose_element is not None:
                pose_element.text = f'{x} {y} {-heights[i]}'
    tree.write(f'{filepath}/probabilistics/forest.world')
    return coords
