"""my_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import numpy as np

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
wheelRL = robot.getDevice('rear_left_wheel')
wheelFR = robot.getDevice('front_right_wheel')
wheelRR = robot.getDevice('rear_right_wheel')
wheelFL = robot.getDevice('front_left_wheel')
sensor1 = robot.getDevice('ir_ext_left')
sensor2 = robot.getDevice('ir_ext_right')
sensor3 = robot.getDevice('ir_left')
sensor4 = robot.getDevice('ir_right')
sensor5 = robot.getDevice('ir_mid_left')
sensor6 = robot.getDevice('ir_mid_right')
wheelRL.setPosition(float('inf'))
wheelFR.setPosition(float('inf'))
wheelRR.setPosition(float('inf'))
wheelFL.setPosition(float('inf'))

camera = robot.getDevice('front_camera')
camera.enable(timestep)
sensor1.enable(timestep)
sensor2.enable(timestep)
sensor3.enable(timestep)
sensor4.enable(timestep)
sensor5.enable(timestep)
sensor6.enable(timestep)

while robot.step(timestep) != -1:
    
    left_sensor_value = np.array([sensor.getValue() for sensor in [sensor1, sensor3, sensor5]])
    right_sensor_value = np.array([sensor.getValue() for sensor in [sensor2, sensor4, sensor6]])
    
    if (np.sum(left_sensor_value < 1000) >= 1):
        wheelRL.setVelocity(4.0)
        wheelFL.setVelocity(4.0)
        wheelFR.setVelocity(0.0)
        wheelRR.setVelocity(0.0)
    elif (np.sum(right_sensor_value < 1000) >= 1):
        wheelRL.setVelocity(0.0)
        wheelFL.setVelocity(0.0)
        wheelFR.setVelocity(4.0)
        wheelRR.setVelocity(4.0)
    else:
        wheelRL.setVelocity(4.0)
        wheelFR.setVelocity(4.0)
        wheelRR.setVelocity(4.0)
        wheelFL.setVelocity(4.0)
    
    pass

# Enter here exit cleanup code.
