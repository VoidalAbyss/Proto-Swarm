"""Test_Variables controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
#timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)

timestep = 64
max_speed = 6.28
    # You should insert a getDevice-like function in order to get the
    # instance of a device of the robot. Something like:
    #  motor = robot.getDevice('motorname')
    #  ds = robot.getDevice('dsname')
    #  ds.enable(timestep)
    
left_motor = robot.getDevice('motor_1')
right_motor = robot.getDevice('motor_2')
    
left_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
    
right_motor.setPosition(float('inf'))
right_motor.setVelocity(0.0)

gps = robot.getDevice('gps_custom')
gps.enable(timestep)



# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    gps_value = gps.getValues()
    print(gps_value)
    # Process sensor data here.
    left_speed = 0.5*max_speed
    right_speed = 0.5*max_speed
            
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)
    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
     
# Enter here exit cleanup code.
