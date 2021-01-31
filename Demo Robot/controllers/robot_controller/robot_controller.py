"""robot_controller controller."""

from controller import Robot
import numpy as np

# set up robot
robot = Robot()

# get time steps iterator
timestep = int(robot.getBasicTimeStep())

# set up wheels motor on the robot
wheelFL = robot.getDevice('front_left_wheel')
wheelRL = robot.getDevice('rear_left_wheel')
wheelFR = robot.getDevice('front_right_wheel')
wheelRR = robot.getDevice('rear_right_wheel')

# activate wheels
wheelRL.setPosition(float('inf'))
wheelFR.setPosition(float('inf'))
wheelRR.setPosition(float('inf'))
wheelFL.setPosition(float('inf'))

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

# automatically navigation by sensors
def automatic_navigation():

    left_sensors_value = np.array([sensor.getValue() for sensor in [sensorL1, sensorL2, sensorL3]])
    right_sensors_value = np.array([sensor.getValue() for sensor in [sensorR1, sensorR2, sensorR3]])
    
    if (np.any(left_sensors_value < 1000)):
        wheelRL.setVelocity(5.0)
        wheelFL.setVelocity(5.0)
        wheelFR.setVelocity(0.0)
        wheelRR.setVelocity(0.0)
    elif (np.any(right_sensors_value < 1000)):
        wheelRL.setVelocity(0.0)
        wheelFL.setVelocity(0.0)
        wheelFR.setVelocity(5.0)
        wheelRR.setVelocity(5.0)
    else:
        wheelRL.setVelocity(5.0)
        wheelFR.setVelocity(5.0)
        wheelRR.setVelocity(5.0)
        wheelFL.setVelocity(5.0)

# main loop - in each time step, do following
while robot.step(timestep) != -1:

    automatic_navigation()
    
    pass
