"""wasd_controller controller."""

from controller import Robot, Keyboard
from enum import Enum
from math import sqrt
from collections import deque

MAX_SPEED = 5
GRID_RESOLUTION = 0.15 # in meters
ROUND_PRECISION = 2 # number of digits after decimal point in GRID_RESOLUTION, must keep in sync with GRID_RESOLUTION!

class GridNodeType(Enum):
    EMPTY = 1
    OBSTACLE = 2

class GridNode:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.type = GridNodeType.EMPTY
        self.distance = None
        self.step = None

    def __repr__(self):
        if self.type is GridNodeType.OBSTACLE:
            return '|\033[93m x \033[0m'
        elif self.step is not None:
            return f'|{self.step: ^3}'
        else:
            return '|   '

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

        self.coordinates = []
        self.tracking = False
        self.tracking_initiated = False
        self.tracking_start_position = None
        self.prefix = False
        self.planned = False

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
                        print(f'Current position: ({x}, {y}) | Tracking start position: ({self.tracking_start_position[0]}, {self.tracking_start_position[1]})')
                    else:
                        print('FINISHED TRACKING')
                        self.tracking = False

                        self.grid = build_garden_grid(self.coordinates)
                        print_grid(self.grid)
            # start initial plan
            elif (key == ord('M')):
                if not self.planned:
                    self.planned = True

                    wavefront(self.grid, (x, y))
                    self.nodes_to_cover = initial_path(self.grid, (x, y))

                    print_grid(self.grid)
            else:
                self.move('stop')
                self.turn('stop')

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
            print(node, end='')

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
    num_rows = int(((right_x - left_x) // GRID_RESOLUTION) + 1)
    num_cols = int(((top_y - bottom_y) // GRID_RESOLUTION) + 1)

    left_x = round(left_x, ROUND_PRECISION)
    top_y = round(top_y, ROUND_PRECISION)

    grid = []

    for row in range(num_rows):
        grid.append([])

        for col in range(num_cols):
            node_x = round(left_x + ((col + 1) * GRID_RESOLUTION), ROUND_PRECISION)
            node_y = round(top_y - ((row + 1) * GRID_RESOLUTION), ROUND_PRECISION)

            grid[row].append(GridNode(node_x, node_y))

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

def initial_path(grid, robot_coords):
    cur_row, cur_col = coords_to_grid_indices(grid, robot_coords)
    current_node = grid[cur_row][cur_col]

    explored = set()
    current_node.step = 0

    path = []

    while True:
        explored.add(current_node)
        path.append(current_node)

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

    row = int(round(abs(coords[0] - top_left_x), ROUND_PRECISION) / GRID_RESOLUTION)
    col = int(round(abs(coords[1] - top_left_y), ROUND_PRECISION) / GRID_RESOLUTION)

    return (row, col)

def main():
    robot = GPSRobot()
    robot.run_loop()

if __name__ == '__main__':
    main()
