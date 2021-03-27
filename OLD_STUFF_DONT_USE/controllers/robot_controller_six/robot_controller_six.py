"""robot_controller controller."""

from controller import Robot
import numpy as np

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
    
# automatical navigation by sensors
def automatic_navigation():

    left_sensors_value = np.array([sensor.getValue() for sensor in [sensorL1, sensorL2, sensorL3]])
    right_sensors_value = np.array([sensor.getValue() for sensor in [sensorR1, sensorR2, sensorR3]])
    
    if (np.any(left_sensors_value < 1000)):
        turn('right')
    elif (np.any(right_sensors_value < 1000)):
        turn('left')
    else:
        move('forward')

# main loop - in each time step, do following
while robot.step(timestep) != -1:

    automatic_navigation()
    
    pass
