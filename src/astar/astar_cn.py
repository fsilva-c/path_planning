import numpy as np
import heapq
import time

# PriorityQueue
class PriorityQueue:
    def __init__(self):
        self.elements =[]

    def empty(self):
        return not self.elements

    def put(self, coordination, priority):
        heapq.heappush(self.elements, (priority, coordination))

    def get(self,):
        return heapq.heappop(self.elements)[1]

class astar_3d_algo:
    def __init__(self,mapsize,start_point,end_point,cost_map):
        self.map_size=mapsize
        self.start_point = start_point
        self.end_point = end_point
        self.cost_map = cost_map

    # Get neighbors of current position
    def get_neighbors(self,current):
        neighbors = set()
        # Down
        if current[0]+1<self.map_size[0]:
            neighbors.add((current[0]+1,current[1], current[2]))
        # Left
        if current[1]-1>=0:
            neighbors.add((current[0],current[1]-1, current[2]))
        # Up
        if current[0]-1>=0:
            neighbors.add((current[0]-1,current[1], current[2]))
        # Right
        if current[1]+1<self.map_size[1]:
            neighbors.add((current[0],current[1]+1, current[2]))
        # Z-Up
        if current[2]+1<self.map_size[2]:
            neighbors.add((current[0], current[1], current[2]+1))
        # Z-Down
        if current[2]-1>=0:
            neighbors.add((current[0], current[1], current[2]-1))

        return neighbors

    def get_distance(self, current, end_point):
        distance = np.sqrt(np.abs(current[0]-end_point[0])**2 + np.abs(current[1]-end_point[1])**2 + np.abs(current[2]-end_point[2])**2)
        return distance

    # Find the path
    def find_path(self,):
        frontier = PriorityQueue()
        frontier.put(self.start_point,0)

        came_from = dict()
        came_from[self.start_point]=None

        cost_value = dict()
        cost_value[self.start_point]=0


        # Searching Grid Map
        st = time.time()
        while not frontier.empty():
            current = frontier.get()
            if current == self.end_point:
                break
            neighbors = self.get_neighbors(current)
            for next in neighbors:
                new_cost = cost_value[current] + self.cost_map[next]
                if next not in cost_value or new_cost < cost_value[next]:
                    cost_value[next] = new_cost 
                    # Dijkstra's Algorithm
                    # priority = new_cost
                    # A* Algorithm
                    priority = new_cost + self.get_distance(next, self.end_point)
                    frontier.put(next,priority)
                    came_from[next]=current
        et = time.time()
        print((et-st))

        # Reconstruction path --> (Backwards from the goal to the start)
        current = self.end_point
        path = []
        while current != self.start_point:
            path.append(current)
            current = came_from[current]
        path.append(self.start_point)
        path.reverse()

        return path
        

if __name__ == '__main__':
    # A* Algorithm
    # Define Map
    start_point = (0,0,9)
    end_point = (50,10,12)
    map_size = (200,200,200)
    map = np.zeros((map_size[0], map_size[1], map_size[2]))

    # A* Algorithm
    path_finder = astar_3d_algo(map_size,start_point,end_point,map)
    path = path_finder.find_path()
    print(path)