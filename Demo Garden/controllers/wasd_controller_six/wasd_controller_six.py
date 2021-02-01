"""wasd_controller controller."""

from controller import Robot, Keyboard

# define maximum speed.
MAX_SPEED = 5

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# set up the keyboard.
keyboard = Keyboard()
keyboard.enable(timestep)

# activate camera
camera = robot.getDevice('front_camera')
camera.enable(timestep)

# set up wheels motor.
wheels = []
wheelsNames = ['left_front_wheel', 'right_front_wheel', 'left_rear_wheel', 'right_rear_wheel', 'left_mid_wheel', 'right_mid_wheel']
for i in range(6):
    wheels.append(robot.getDevice(wheelsNames[i]))
    wheels[i].setPosition(float('inf'))
    wheels[i].setVelocity(0.0)

# define movement
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

# define turning
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

# Main loop:
# perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:

    key = keyboard.getKey()
    
    if (key == ord('W')):
        move('forward')
    elif (key == ord('S')):
        move('backward')  
    elif (key == ord('A')):
        turn('left')
    elif (key == ord('D')):
        turn('right')
    else:
        move('stop')
        turn('stop')    
    
    pass
