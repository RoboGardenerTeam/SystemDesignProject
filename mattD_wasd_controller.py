"""demo_controller controller."""


from controller import Robot, Keyboard
MAX_SPEED = 6
TIME_STEP = 64
# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

keyboard = Keyboard()
keyboard.enable(TIME_STEP)

wheels = []
wheelsNames = ['wheel1', 'wheel2', 'wheel3', 'wheel4']
for i in range(4):
    wheels.append(robot.getDevice(wheelsNames[i]))
    wheels[i].setPosition(float('inf'))
    wheels[i].setVelocity(0.0)
def move(inp):
    wheels[0].setVelocity(inp*0.5*MAX_SPEED)
    wheels[1].setVelocity(inp*0.5*MAX_SPEED)
    wheels[2].setVelocity(inp*0.5*MAX_SPEED)
    wheels[3].setVelocity(inp*0.5*MAX_SPEED)
    
    
def turn(inp):
    wheels[0].setVelocity(inp*0.5*MAX_SPEED)
    wheels[1].setVelocity(-inp*0.5*MAX_SPEED)
    wheels[2].setVelocity(inp*0.5*MAX_SPEED)
    wheels[3].setVelocity(-inp*0.5*MAX_SPEED)
    
movement = 0
turning = 0

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(TIME_STEP) != -1:
    key=keyboard.getKey()
    if (key==ord('W')):
        if (movement < 1):
            movement += 1
        print("W Pressed")
        move(movement)
    elif (key==ord('S')):
        if (movement >- 1):
            movement -= 1
        print("S Pressed")
        move(movement)  
    elif (key==ord('A')):
        if (turning >-1):
            turning -= 1
        print("A Pressed")
        turn(turning)
    elif (key==ord('D')):
        if (turning < 1):
            turning += 1
        print("D Pressed")
        turn(turning)
    else:
        movement = 0
        turning = 0
        move(movement)
        turn(turning)
    
    
    
pass

# Enter here exit cleanup code.
