"""robot_controller controller."""

from controller import Robot
from time import perf_counter
from functools import reduce
from random import randrange
import numpy as np
import csv
import sys

class GridNode:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.visited = False

GRID_RESOLUTION = 0.15 # in meters

# set up robot
robot = Robot()

# get time steps iterator
timestep = int(robot.getBasicTimeStep())

# define maximum speed
MAX_SPEED = 5

# set up wheels motor on the robot
wheels = []
wheelsNames = ['left_front_wheel', 'right_front_wheel', 'left_rear_wheel', 'right_rear_wheel', 'left_mid_wheel', 'right_mid_wheel']
for i in range(6):
    wheels.append(robot.getDevice(wheelsNames[i]))
    wheels[i].setPosition(float('inf'))
    wheels[i].setVelocity(0.0)

# set up sensors on the robot 
sensorL1 = robot.getDevice('left_side_sensor')
sensorL2 = robot.getDevice('left_outer_sensor')
sensorL3 = robot.getDevice('left_mid_sensor')
sensorR1 = robot.getDevice('right_side_sensor')
sensorR2 = robot.getDevice('right_outer_sensor')
sensorR3 = robot.getDevice('right_mid_sensor')

# activate sensors
sensorL1.enable(timestep)
sensorL2.enable(timestep)
sensorL3.enable(timestep)
sensorR1.enable(timestep)
sensorR2.enable(timestep)
sensorR3.enable(timestep)

# set up front camera
camera = robot.getDevice('front_camera')

# activate camera
camera.enable(timestep)

# define movement in specific direction
def move(direction):
    if direction == 'stop': inp = 0
    if direction == 'forward': inp = 1
    if direction == 'backward': inp = -1
    wheels[0].setVelocity(inp * MAX_SPEED)
    wheels[1].setVelocity(inp * MAX_SPEED)
    wheels[2].setVelocity(inp * MAX_SPEED)
    wheels[3].setVelocity(inp * MAX_SPEED)
    wheels[4].setVelocity(inp * MAX_SPEED)
    wheels[5].setVelocity(inp * MAX_SPEED)

# define turning in specific direction
def turn(direction):
    if direction == 'stop': inp = 0
    if direction == 'right': inp = 1
    if direction == 'left': inp = -1
    wheels[0].setVelocity(inp * MAX_SPEED)
    wheels[1].setVelocity(-inp * MAX_SPEED)
    wheels[2].setVelocity(inp * MAX_SPEED)
    wheels[3].setVelocity(-inp * MAX_SPEED)
    wheels[4].setVelocity(inp * MAX_SPEED)
    wheels[5].setVelocity(-inp * MAX_SPEED)

# i think this function isn't actually quite right
def bearing_in_degrees(compass_vec):
    rad = np.arctan2(compass_vec[0], compass_vec[1])
    bearing = (rad - 1.5708) / np.pi * 180.0

    if bearing < 0.0:
        bearing += 360.0

    return bearing

def rotate_vector_radians(vector, theta):
    rotation_matrix = np.array([
        [np.cos(theta), -np.sin(theta)],
        [np.sin(theta), np.cos(theta)]
    ])

    return np.dot(rotation_matrix, vector)

# automatical navigation by sensors
def automatic_navigation():
    global reorienting
    global new_orientation

    left_sensors_value = np.array([sensor.getValue() for sensor in [sensorL1, sensorL2, sensorL3]])
    right_sensors_value = np.array([sensor.getValue() for sensor in [sensorR1, sensorR2, sensorR3]])

    if reorienting:
        compass_vec = compass.getValues()
        compass_vec = np.array([compass_vec[0], compass_vec[2]])

        new_orientation_vector = np.array([np.cos(np.deg2rad(new_orientation)), np.sin(np.deg2rad(new_orientation))])
        v = rotate_vector_radians(new_orientation_vector, -1 * np.arctan2(compass_vec[1], compass_vec[0]))
        signed_angle = np.arctan2(v[1], v[0])

        if abs(signed_angle) < 0.087: # 5 degrees
            reorienting = False
        elif signed_angle > 0:
            turn('left')
        else:
            turn('right')
    elif (np.any(left_sensors_value < 1000)) or (np.any(right_sensors_value < 1000)):
        new_orientation = randrange(0, 360)
        reorienting = True
    else:
        move('forward')

def build_empty_grid(top_y, right_x, bottom_y, left_x):
    num_rows = int(((top_y - bottom_y) / GRID_RESOLUTION) + 1)
    num_cols = int(((right_x - left_x) / GRID_RESOLUTION) + 1)

    grid = []

    for row in range(num_rows):
        grid.append([])

        for col in range(num_cols):
            node_x = left_x + ((col + 1) * GRID_RESOLUTION)
            node_y = top_y - ((row + 1) * GRID_RESOLUTION)

            grid[row].append(GridNode(node_x, node_y))

    return grid

def coords_to_grid_indices(grid, coords):
    top_left_x = grid[0][0].x
    top_left_y = grid[0][0].y

    row = int(abs(coords[1] - top_left_y) / GRID_RESOLUTION)
    col = int(abs(coords[0] - top_left_x) / GRID_RESOLUTION)

    return (row, col)

grid = build_empty_grid(0.7, .7, -0.7, -0.7) # chessboard environment really more like has width 0.9 + 0.9, but make it smaller since it's hard for robot to travel close to the edges
num_nodes = len(grid) * len(grid[0])
start = perf_counter()

gps = robot.getDevice('gps')
gps.enable(timestep)

compass = robot.getDevice('compass')
compass.enable(timestep)

reorienting = False
new_orientation = 0

revisiting_counter = 0
total_counter = 0
prev = None

csv_counter = 0 # counter to stop saving to csv file so often

def print_and_save_stats(csv_writer):
    progress_percentage = num_visited / num_nodes * 100
    revisit_percentage = revisiting_counter / total_counter * 100
    print(f'Progress percentage: {progress_percentage} ({num_visited}/{num_nodes})')
    print(f'Revisiting percentage: {revisit_percentage}')

    global csv_counter
    csv_counter += 1
    csv_counter %= 10

    if csv_counter == 0:
        csv_writer.writerow([f'{perf_counter()}', f'{progress_percentage}', f'{revisit_percentage}'])

with open('random_stats.csv', 'w', newline='\n') as csvfile:
    csv_writer = csv.writer(csvfile, delimiter=',', quotechar='\"', quoting=csv.QUOTE_MINIMAL)
    csv_writer.writerow(['Time', 'Progress %', 'Revisit %'])

    while robot.step(timestep) != -1:
        y, _, x = gps.getValues()

        cur_row, cur_col = coords_to_grid_indices(grid, (x, y))

        if grid[cur_row][cur_col].visited and grid[cur_row][cur_col] != prev:
            revisiting_counter += 1
        else:
            grid[cur_row][cur_col].visited = True
            prev = grid[cur_row][cur_col]

        total_counter += 1

        # very crusty lambdas, this just counts how many nodes in the grid are marked as visited
        num_visited = reduce(lambda acc, count: acc + count, map(lambda row: reduce(lambda acc, node: acc + 1 if node.visited else acc, row, 0), grid))

        automatic_navigation()
        print_and_save_stats(csv_writer)

        # visited x% of nodes, visiting last (100 - x)% takes super long for random robot
        if num_visited/num_nodes >= 0.98:
            end = perf_counter()
            print(f'Total execution time: {end - start}')
            sys.exit(0)
