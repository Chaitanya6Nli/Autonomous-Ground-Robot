import numpy as np
from queue import PriorityQueue

class Node:
    def __init__(self, position, g_cost, h_cost, parent):
        self.position = position
        self.g_cost = g_cost
        self.h_cost = h_cost
        self.f_cost = g_cost + h_cost
        self.parent = parent

    def __lt__(self, other):
        return self.f_cost < other.f_cost

class AStarPlanner:
    def __init__(self, map_data, resolution, origin):
        self.map = map_data
        self.resolution = resolution
        self.origin = origin

    def plan(self, start, goal):
        start_node = Node(start, 0, self.heuristic(start, goal), None)
        open_list = PriorityQueue()
        open_list.put(start_node)
        closed_set = set()

        while not open_list.empty():
            current = open_list.get()

            if self.is_goal(current.position, goal):
                return self.reconstruct_path(current)

            closed_set.add(current.position)

            for neighbor in self.get_neighbors(current.position):
                if neighbor in closed_set:
                    continue

                g_cost = current.g_cost + 1
                h_cost = self.heuristic(neighbor, goal)
                neighbor_node = Node(neighbor, g_cost, h_cost, current)

                if not self.is_in_open_list(open_list, neighbor):
                    open_list.put(neighbor_node)
                else:
                    self.update_node(open_list, neighbor_node)

        return None  # No path found

    def heuristic(self, a, b):
        return np.sqrt((b[0] - a[0])**2 + (b[1] - a[1])**2)

    def is_goal(self, current, goal):
        return current == goal

    def get_neighbors(self, position):
        x, y = position
        neighbors = [
            (x-1, y), (x+1, y), (x, y-1), (x, y+1),
            (x-1, y-1), (x-1, y+1), (x+1, y-1), (x+1, y+1)
        ]
        return [n for n in neighbors if self.is_valid(n)]

    def is_valid(self, position):
        x, y = position
        if x < 0 or y < 0 or x >= self.map.shape[0] or y >= self.map.shape[1]:
            return False
        return self.map[x, y] == 0  # Assuming 0 is free space

    def is_in_open_list(self, open_list, position):
        return any(node.position == position for node in open_list.queue)

    def update_node(self, open_list, new_node):
        for i, node in enumerate(open_list.queue):
            if node.position == new_node.position and new_node.f_cost < node.f_cost:
                open_list.queue[i] = new_node
                break

    def reconstruct_path(self, node):
        path = []
        while node:
            path.append(node.position)
            node = node.parent
        return path[::-1]

    def world_to_map(self, world_coord):
        x = int((world_coord[0] - self.origin[0]) / self.resolution)
        y = int((world_coord[1] - self.origin[1]) / self.resolution)
        return (x, y)

    def map_to_world(self, map_coord):
        x = map_coord[0] * self.resolution + self.origin[0]
        y = map_coord[1] * self.resolution + self.origin[1]
        return (x, y)