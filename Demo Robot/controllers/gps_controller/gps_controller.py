"""wasd_controller controller."""

from controller import Robot, Keyboard
from enum import Enum
from math import sqrt
from collections import deque
from heapq import *
from time import perf_counter
import numpy as np

MAX_SPEED = 5
GRID_RESOLUTION = 0.15 # in meters

class GridNodeType(Enum):
    EMPTY = 1
    OBSTACLE = 2

class GridNode:
    def __init__(self, x, y, indices):
        self.x = x
        self.y = y
        self.row = indices[0]
        self.col = indices[1]
        self.type = GridNodeType.EMPTY
        self.distance = None
        self.step = None

    def __repr__(self):
        if self.type is GridNodeType.OBSTACLE:
            return f'\033[93m x \033[0m'
        elif self.step is not None:
            return f'{self.step: >3}'
        elif self.distance is not None:
            return f'{self.distance: >3}'
        else:
            return '   '

class GPSRobot:
    def __init__(self):
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())

        self.keyboard = Keyboard()
        self.keyboard.enable(self.timestep)

        self.camera = self.robot.getDevice('front_camera')
        self.camera.enable(self.timestep)

        self.wheels = []
        self.wheelsNames = ['left_front_wheel', 'right_front_wheel', 'left_rear_wheel', 'right_rear_wheel', 'left_mid_wheel', 'right_mid_wheel']
        for i in range(6):
            self.wheels.append(self.robot.getDevice(self.wheelsNames[i]))
            self.wheels[i].setPosition(float('inf'))
            self.wheels[i].setVelocity(0.0)

        self.gps = self.robot.getDevice('gps')
        self.gps.enable(self.timestep)

        self.compass = self.robot.getDevice('compass')
        self.compass.enable(self.timestep)

        self.coordinates = []
        self.tracking = False
        self.tracking_initiated = False
        self.tracking_start_position = None
        self.prefix = False
        self.planned = False
        self.initiate_movement = False
        self.finished_program = False

    def run_loop(self):
        while self.robot.step(self.timestep) != -1:
            y, _, x = self.gps.getValues()

            if self.tracking:
                self.coordinates.append((x, y))

            key = self.keyboard.getKey()
            if (key == ord('W')):
                self.move('forward')
            elif (key == ord('S')):
                self.move('backward')
            elif (key == ord('A')):
                self.turn('left')
            elif (key == ord('D')):
                self.turn('right')
            elif (key == ord('P')):
                self.prefix = True
                print('PREFIX ENGAGED')
            # start tracking boundary
            elif (key == ord('T')):
                if self.prefix and not self.tracking_initiated and not self.tracking:
                    print('STARTED TRACKING')
                    self.tracking = True
                    self.tracking_initiated = True
                    self.tracking_start_position = (x, y)

                    print('PREFIX DISENGAGED')
                    self.prefix = False
                elif self.prefix and self.tracking_initiated and self.tracking:
                    if coord_distance(self.tracking_start_position, (x, y)) > GRID_RESOLUTION:
                        print('CONTINUE TRACKING: MOVE CLOSER TO TRACKING START POSITION')
                        print(f'Current position: ({x}, {y}) \t -> Tracking start position: ({self.tracking_start_position[0]}, {self.tracking_start_position[1]})')
                    else:
                        print('FINISHED TRACKING')
                        self.tracking = False

                        self.grid = build_garden_grid(self.coordinates)
                        print('-----   GARDEN BOUNDARY   -----')
                        print_grid(self.grid)
            # start initial plan
            elif (key == ord('M')):
                if not self.planned:
                    self.planned = True

                    wavefront(self.grid, (x, y))
                    print('-----   WAVEFRONT (DISTANCE TRANSFORM)   -----')
                    print_grid(self.grid)

                    nodes_to_visit = initial_nodes_to_visit(self.grid, (x, y))
                    print('-----   NODES TO VISIT IN ORDER   -----')
                    print_grid(self.grid)

                    self.create_total_path(nodes_to_visit)
                    print('-----   COMPLETE PATH TO COVER ALL NODES TO BE VISITED   -----')
                    print(self.total_path)
            # initiate autonomous movement
            elif (key == ord('I')):
                if self.planned:
                    self.start = perf_counter()
                    print('MOVEMENT INITIATED')
                    self.initiate_movement = True
                else:
                    print('PLAN PATH FIRST (PRESS M)')
            else:
                self.move('stop')
                self.turn('stop')

            if self.initiate_movement and not self.finished_program:
                cur_row, cur_col = coords_to_grid_indices(self.grid, (x, y))
                cur_row = clamp(cur_row, 0, len(self.grid) - 1)
                cur_col = clamp(cur_row, 0, len(self.grid[0]) - 1)

                current_node = self.grid[cur_row][cur_col]
                target_node = self.total_path[0]

                if coord_distance((x, y), (target_node.x, target_node.y)) < GRID_RESOLUTION:
                    print(f'POPPED NODE TO VISIT, TARGETING NEW NODE: {target_node}')
                    self.total_path.popleft()

                    if len(self.total_path) == 0:
                        print('REACHED ALL NODES!!! END PROGRAM')
                        self.end = perf_counter()
                        print(f'Execution time: {self.end - self.start}')
                        self.finished_program = True
                else: # head towards target_node
                    robot_to_target = np.array([target_node.y - y, target_node.x - x])
                    robot_to_target_degrees = 360.0 - bearing_in_degrees(robot_to_target)
                    compass_vec = self.compass.getValues()
                    compass_vec = np.array([compass_vec[0], compass_vec[2]])
                    orientation_degrees = bearing_in_degrees(compass_vec)

                    v = rotate_vector_radians(robot_to_target, np.arctan2(compass_vec[1], compass_vec[0]))
                    signed_angle = np.arctan2(v[1], v[0])

                    # close enough, just go forward
                    if abs(orientation_degrees - robot_to_target_degrees) < 5:
                        self.move('forward')
                    # adjust heading
                    else:
                        if signed_angle < 0:
                            self.turn('left')
                        else:
                            self.turn('right')

    def move(self, direction):
        if direction == 'stop': inp = 0
        if direction == 'forward': inp = 1
        if direction == 'backward': inp = -1

        self.wheels[0].setVelocity(inp * MAX_SPEED)
        self.wheels[1].setVelocity(inp * MAX_SPEED)
        self.wheels[2].setVelocity(inp * MAX_SPEED)
        self.wheels[3].setVelocity(inp * MAX_SPEED)
        self.wheels[4].setVelocity(inp * MAX_SPEED)
        self.wheels[5].setVelocity(inp * MAX_SPEED)

    def turn(self, direction):
        if direction == 'stop': inp = 0
        if direction == 'right': inp = 1
        if direction == 'left': inp = -1

        self.wheels[0].setVelocity(inp * MAX_SPEED)
        self.wheels[1].setVelocity(-inp * MAX_SPEED)
        self.wheels[2].setVelocity(inp * MAX_SPEED)
        self.wheels[3].setVelocity(-inp * MAX_SPEED)
        self.wheels[4].setVelocity(inp * MAX_SPEED)
        self.wheels[5].setVelocity(-inp * MAX_SPEED)

    def create_total_path(self, nodes_to_visit):
        y, _, x = self.gps.getValues()
        start_row, start_col = coords_to_grid_indices(self.grid, (x, y))
        start_node = self.grid[start_row][start_col]

        self.total_path = deque()

        while len(nodes_to_visit) > 0:
            goal_node = nodes_to_visit.pop()
            path = self.path_to_node(start_node, goal_node)
            self.total_path += path
            start_node = self.total_path[-1]

    def path_to_node(self, start_node, goal_node):
        class AStarNode():
            def __init__(self, grid_node, parent_astar_node, g_cost, h_cost):
                self.grid_node = grid_node
                self.parent_astar_node = parent_astar_node
                self.g_cost = g_cost
                self.h_cost = h_cost

            def __repr__(self):
                return f'{self.grid_node} {self.g_cost}+{self.h_cost}'

            def __lt__(self, other):
                return self.g_cost < other.g_cost

        # manhattan distance
        def calculate_heuristic(node, goal_node):
            return abs(node.row - goal_node.row) + abs(node.col - goal_node.col)

        def astar_node_children(grid, astar_node, goal_grid_node):
            row = astar_node.grid_node.row
            col = astar_node.grid_node.col

            children = []

            # top
            if row != 0:
                top_grid_node = grid[row - 1][col]

                if top_grid_node.type is GridNodeType.EMPTY:
                    child_node = AStarNode(top_grid_node,
                                           astar_node,
                                           astar_node.g_cost + 1,
                                           calculate_heuristic(top_grid_node, goal_grid_node))
                    children.append(child_node)

            # right
            if col != len(grid[0]) - 1:
                right_grid_node = grid[row][col + 1]

                if right_grid_node.type is GridNodeType.EMPTY:
                    child_node = AStarNode(right_grid_node,
                                           astar_node,
                                           astar_node.g_cost + 1,
                                           calculate_heuristic(right_grid_node, goal_grid_node))
                    children.append(child_node)

            # bottom
            if row != len(grid) - 1:
                bottom_grid_node = grid[row + 1][col]

                if bottom_grid_node.type is GridNodeType.EMPTY:
                    child_node = AStarNode(bottom_grid_node,
                                           astar_node,
                                           astar_node.g_cost + 1,
                                           calculate_heuristic(bottom_grid_node, goal_grid_node))
                    children.append(child_node)

            # left
            if col != 0:
                left_grid_node = grid[row][col - 1]

                if left_grid_node.type is GridNodeType.EMPTY:
                    child_node = AStarNode(left_grid_node,
                                           astar_node,
                                           astar_node.g_cost + 1,
                                           calculate_heuristic(left_grid_node, goal_grid_node))
                    children.append(child_node)

            return children

        def frontier_contains(frontier, astar_node):
            for cost, frontier_astar_node in frontier:
                if astar_node.grid_node == frontier_astar_node.grid_node:
                    return (cost, frontier_astar_node)

            return None

        g_cost = 0
        h_cost = calculate_heuristic(start_node, goal_node)
        frontier = [(g_cost + h_cost, AStarNode(start_node, None, g_cost, h_cost))]
        explored = set()

        while frontier:
            astar_node = heappop(frontier)[1]

            # return path
            if astar_node.grid_node == goal_node:
                path = [astar_node.grid_node]
                current_astar_node = astar_node.parent_astar_node

                while current_astar_node is not None:
                    path.append(current_astar_node.grid_node)
                    current_astar_node = current_astar_node.parent_astar_node

                path.reverse()
                return path

            explored.add(astar_node.grid_node)

            for astar_child in astar_node_children(self.grid, astar_node, goal_node):
                cost_and_frontier_astar_node_tuple = frontier_contains(frontier, astar_child)
                astar_child_g_and_h_cost = astar_child.g_cost + astar_child.h_cost

                if astar_child.grid_node not in explored and cost_and_frontier_astar_node_tuple is None:
                    heappush(frontier, (astar_child_g_and_h_cost, astar_child))
                elif cost_and_frontier_astar_node_tuple is not None:
                    if astar_child_g_and_h_cost < cost_and_frontier_astar_node_tuple[0]:
                        del frontier[[astar_child.grid_node == frontier_astar_node.grid_node for _, frontier_astar_node in frontier].index(True)]
                        heapify(frontier)
                        heappush(frontier, (astar_child_g_and_h_cost, astar_child))

def coord_distance(a, b):
    x_delta = a[0] - b[0]
    y_delta = a[1] - b[1]

    return sqrt(x_delta**2 + y_delta**2)

def build_garden_grid(coordinates):
    top_y, right_x, bottom_y, left_x = find_extremes(coordinates)
    grid = build_empty_grid(coordinates, top_y, right_x, bottom_y, left_x)
    fill_grid_obstacle(coordinates, grid)

    return grid

def print_grid(grid):
    for row in grid:
        for node in row:
            print('|' + str(node), end='')

        print('|')

def find_extremes(coordinates):
    top_y = float('-inf')
    right_x = float('-inf')
    bottom_y = float('inf')
    left_x = float('inf')

    for (x, y) in coordinates:
        if y > top_y:
            top_y = y
        elif y < bottom_y:
            bottom_y = y

        if x > right_x:
            right_x = x
        elif x < left_x:
            left_x = x

    return (top_y, right_x, bottom_y, left_x)

def build_empty_grid(coordinates, top_y, right_x, bottom_y, left_x):
    num_rows = int(((top_y - bottom_y) / GRID_RESOLUTION) + 1)
    num_cols = int(((right_x - left_x) / GRID_RESOLUTION) + 1)

    grid = []

    for row in range(num_rows):
        grid.append([])

        for col in range(num_cols):
            node_x = left_x + ((col + 1) * GRID_RESOLUTION)
            node_y = top_y - ((row + 1) * GRID_RESOLUTION)

            grid[row].append(GridNode(node_x, node_y, (row, col)))

    return grid

def fill_grid_obstacle(obstacle_coordinates, grid):
    for (x, y) in obstacle_coordinates:
        row, col = coords_to_grid_indices(grid, (x, y))
        grid[row][col].type = GridNodeType.OBSTACLE

def wavefront(grid, robot_coords):
    row, col = coords_to_grid_indices(grid, robot_coords)
    start_node = grid[row][col]
    start_node.distance = 0

    cur_row, cur_col = coords_to_grid_indices(grid, robot_coords)
    explored = set()
    frontier = deque()
    frontier.appendleft((start_node, (row, col)))

    while len(frontier) > 0:
        current_node, (row, col) = frontier.pop()
        explored.add(current_node)

        # top
        if row != 0:
            top = grid[row - 1][col]

            if top.type is GridNodeType.EMPTY and not top in explored:
                top.distance = current_node.distance + 1
                frontier.appendleft((top, (row - 1, col)))

        # right
        if col != len(grid[0]) - 1:
            right = grid[row][col + 1]

            if right.type is GridNodeType.EMPTY and not right in explored:
                right.distance = current_node.distance + 1
                frontier.appendleft((right, (row, col + 1)))

        # bottom
        if row != len(grid) - 1:
            bottom = grid[row + 1][col]

            if bottom.type is GridNodeType.EMPTY and not bottom in explored:
                bottom.distance = current_node.distance + 1
                frontier.appendleft((bottom, (row + 1, col)))

        # left
        if col != 0:
            left = grid[row][col - 1]

            if left.type is GridNodeType.EMPTY and not left in explored:
                left.distance = current_node.distance + 1
                frontier.appendleft((left, (row, col - 1)))

def initial_nodes_to_visit(grid, robot_coords):
    cur_row, cur_col = coords_to_grid_indices(grid, robot_coords)
    current_node = grid[cur_row][cur_col]

    explored = set()
    current_node.step = 0

    path = deque()

    while True:
        explored.add(current_node)
        path.appendleft(current_node)

        next_node, (next_row, next_col) = find_highest_distance_neighbor(grid, current_node, (cur_row, cur_col), explored)

        # no direct neighbor node 
        if next_node is None:
            next_node, (next_row, next_col) = search_for_unvisited_node(grid, current_node, (cur_row, cur_col), explored)

            # truly no more nodes to visit
            if next_node is None:
                break

        next_node.step = current_node.step + 1
        current_node, (cur_row, cur_col) = next_node, (next_row, next_col)

    return path

def find_highest_distance_neighbor(grid, node, indices, explored):
    row, col = indices[0], indices[1]

    max_distance = -1
    max_distance_node = None
    max_dist_row, max_dist_col = -1, -1

    # top
    if row != 0:
        top = grid[row - 1][col]

        if top.type is GridNodeType.EMPTY and top.distance > max_distance and top not in explored:
            max_distance_node = top
            max_dist_row, max_dist_col = row - 1, col

    # right
    if col != len(grid[0]) - 1:
        right = grid[row][col + 1]

        if right.type is GridNodeType.EMPTY and right.distance > max_distance and right not in explored:
            max_distance_node = right
            max_dist_row, max_dist_col = row, col + 1

    # bottom
    if row != len(grid) - 1:
        bottom = grid[row + 1][col]

        if bottom.type is GridNodeType.EMPTY and bottom.distance > max_distance and bottom not in explored:
            max_distance_node = bottom
            max_dist_row, max_dist_col = row + 1, col

    # left
    if col != 0:
        left = grid[row][col - 1]

        if left.type is GridNodeType.EMPTY and left.distance > max_distance and left not in explored:
            max_distance_node = left
            max_dist_row, max_dist_col = row, col - 1

    return (max_distance_node, (max_dist_row, max_dist_col))

def search_for_unvisited_node(grid, node, indices, already_visited_nodes):
    row, col = indices[0], indices[1]

    explored = set()
    frontier = deque()
    frontier.appendleft((node, (row, col)))

    while len(frontier) > 0:
        current_node, (row, col) = frontier.pop()
        explored.add(current_node)

        # we have found a node we need to visit!
        if current_node not in already_visited_nodes:
            return (current_node, (row, col))

        # top
        if row != 0:
            top = grid[row - 1][col]

            if top.type is GridNodeType.EMPTY and not top in explored:
                top.distance = current_node.distance + 1
                frontier.appendleft((top, (row - 1, col)))

        # right
        if col != len(grid[0]) - 1:
            right = grid[row][col + 1]

            if right.type is GridNodeType.EMPTY and not right in explored:
                right.distance = current_node.distance + 1
                frontier.appendleft((right, (row, col + 1)))

        # bottom
        if row != len(grid) - 1:
            bottom = grid[row + 1][col]

            if bottom.type is GridNodeType.EMPTY and not bottom in explored:
                bottom.distance = current_node.distance + 1
                frontier.appendleft((bottom, (row + 1, col)))

        # left
        if col != 0:
            left = grid[row][col - 1]

            if left.type is GridNodeType.EMPTY and not left in explored:
                left.distance = current_node.distance + 1
                frontier.appendleft((left, (row, col - 1)))

    return (None, (-1, -1))


def coords_to_grid_indices(grid, coords):
    top_left_x = grid[0][0].x
    top_left_y = grid[0][0].y

    row = int(abs(coords[1] - top_left_y) / GRID_RESOLUTION)
    col = int(abs(coords[0] - top_left_x) / GRID_RESOLUTION)

    return (row, col)

def bearing_in_degrees(compass_vec):
    rad = np.arctan2(compass_vec[0], compass_vec[1])
    bearing = (rad - 1.5708) / np.pi * 180.0

    if bearing < 0.0:
        bearing += 360.0

    return bearing

def clamp(n, minimum, maximum):
    return max(minimum, min(n, maximum))

def rotate_vector_radians(vector, theta):
    rotation_matrix = np.array([
        [np.cos(theta), -np.sin(theta)],
        [np.sin(theta), np.cos(theta)]
    ])

    return np.dot(rotation_matrix, vector)

def main():
    robot = GPSRobot()
    robot.run_loop()

if __name__ == '__main__':
    main()
