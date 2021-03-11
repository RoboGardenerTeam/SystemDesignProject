"""robot_controller controller."""

from controller import Robot
import numpy as np

# set up robot
robot = Robot()

# get time steps iterator
timestep = int(robot.getBasicTimeStep())

# define maximum speed
MAX_SPEED = 10

# set up wheels motor on the robot
wheels = []
wheelsNames = ['left_front_wheel', 'right_front_wheel', 'left_rear_wheel', 
               'right_rear_wheel', 'left_mid_wheel', 'right_mid_wheel']
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

# activate camera and recognition
camera.enable(timestep)
camera.recognitionEnable(timestep)

# activate sensor
robot.batterySensorEnable(timestep)

# define movement in specific direction
def move(direction, control=1.0):

    if direction == 'stop': inp = 0 
    if direction == 'forward': inp = 1 
    if direction == 'backward': inp = -1
    
    wheels[0].setVelocity(inp * MAX_SPEED)
    wheels[1].setVelocity(inp * MAX_SPEED)
    wheels[2].setVelocity(inp * MAX_SPEED)
    wheels[3].setVelocity(inp * MAX_SPEED)
    wheels[4].setVelocity(inp * MAX_SPEED * control) 
    wheels[5].setVelocity(inp * MAX_SPEED * control)

# define turning in specific direction
def turn(direction, control=1.0):

    if direction == 'stop': inp = 0 
    if direction == 'right': inp = 1 
    if direction == 'left': inp = -1 
    
    wheels[0].setVelocity(inp * MAX_SPEED)
    wheels[1].setVelocity(-inp * MAX_SPEED)
    wheels[2].setVelocity(inp * MAX_SPEED)
    wheels[3].setVelocity(-inp * MAX_SPEED)
    wheels[4].setVelocity(inp * MAX_SPEED * control) 
    wheels[5].setVelocity(-inp * MAX_SPEED * control)
    
# automatical navigation by sensors
def automatic_navigation(control=1.0):

    left_sensors_value = np.array([s.getValue() for s in [sensorL1, sensorL2, sensorL3]])
    right_sensors_value = np.array([s.getValue() for s in [sensorR1, sensorR2, sensorR3]])
    
    if (np.any(left_sensors_value < 1000)):
        turn('right', control)
    elif (np.any(right_sensors_value < 1000)):
        turn('left', control)
    else:
        move('forward')

# define random navigation algorithm        
def random_navigation_algorithm():
  
    battery_value = robot.batterySensorGetValue()
    if battery_value >= 450000:
        automatic_navigation()
    else:
        automatic_navigation(0.35)

# main loop - in each time step, do following
while robot.step(timestep) != -1:
  
    random_navigation_algorithm()

    pass
