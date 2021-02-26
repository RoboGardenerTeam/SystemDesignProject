"""wasd_controller controller."""

from controller import Robot, Keyboard
from enum import Enum
from math import sqrt

MAX_SPEED = 5
GRID_RESOLUTION = 0.2 # in meters
ROUND_PRECISION = 1 # number of digits after decimal point in GRID_RESOLUTION

class GridNodeState(Enum):
    EMPTY = 1
    OBSTACLE = 2

class GridNode:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.state = GridNodeState.EMPTY

    def __repr__(self):
        return f'({self.x}, {self.y}): {self.state.name}'

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
            elif (key == ord('T')):
                if not self.tracking_initiated and not self.tracking:
                    print('STARTED TRACKING')
                    self.tracking = True
                    self.tracking_initiated = True
                    self.tracking_start_position = (x, y)
                elif self.tracking_initiated and self.tracking:
                    if coord_distance(self.tracking_start_position, (x, y)) > GRID_RESOLUTION:
                        print('CONTINUE TRACKING, NOT CLOSE ENOUGH TO TRACKING START POSITION')
                    else:
                        print('FINISHED TRACKING')
                        self.tracking = False
                        self.grid = build_garden_grid(self.coordinates)
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
    print(grid)

    return grid

def print_grid(grid):
    for row in grid:
        for node in row:
            print(node, end=' ')

        print('')

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
    top_left_x = grid[0][0].x
    top_left_y = grid[0][0].y

    for (x, y) in obstacle_coordinates:
        row = int(round(abs(x - top_left_x), ROUND_PRECISION) / GRID_RESOLUTION)
        col = int(round(abs(y - top_left_y), ROUND_PRECISION) / GRID_RESOLUTION)

        grid[row][col].state = GridNodeState.OBSTACLE

def main():
    robot = GPSRobot()
    robot.run_loop()

if __name__ == '__main__':
    main()
