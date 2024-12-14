"""line_following controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import numpy as np
import sys

# Create the Robot instance.
robot = Robot()


# Get the time step of the current world.
timestep = int(robot.getBasicTimeStep())


# Get the instance of a device of the robot
motor_left = robot.getDevice('left wheel motor')
motor_right = robot.getDevice('right wheel motor')


motor_left.setPosition(float('Inf'))
motor_right.setPosition(float('Inf'))


# Insert a getDevice-like function in order to get the
# instance of a device of the robot. Enable the ground sensors.
gs = []
for i in range(3):
    gs.append(robot.getDevice('gs' + str(i)))
    gs[-1].enable(timestep)



MAX_SPEED = 6.28 # maximum spped of the e-puck motors.

delta_x = 0.0 # displacement in x-direction.
r = 0.0201 # wheel radius.
delta_t = 32/1000 # length of a time step in webots.
d = 0.052 # distance between the center of both wheels.

x_w = 0.0 # inital robot x-coordinate in world frame where 
            # the robot is placed slightly above 
y_w = 0.028 # inital robot y-coordinate in world frame.

omega_z = 0.0 # rotation about the z-axis of the world frame
            # which is the same as the robot frame.
            
alpha = 1.57 # initial angle between the world and robot 
             # frames which we get from the angle field
             # of the e-puck rotation paramter in the scene tree.
             
over_the_line = False # boolean variable used to check if the 
                      # robot has crossed the line. 


# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:

    # Read the sensors:
    g = []
    for groundSensor in gs:
        g.append(groundSensor.getValue()) 
    
    
    # Process sensor data.
    if (g[0]>500 and g[1]<350 and g[2]>500): # drive straight
        phi_l_dot, phi_r_dot = MAX_SPEED, MAX_SPEED
    
    elif (g[0]<302 and g[1]<302 and g[2]<302): # over the line
        
        # Check of the robot has crossed the line; if it has,
        # set the wheel speeds to 0.0 and the over_the_line
        # variable to True, which is used to stop the simulation.
        if abs(((alpha/3.14)*180)) > 300:
            print("Over the line!")
            phi_l_dot, phi_r_dot = 0.0, 0.0
            over_the_line = True
        
    elif (g[2]<550): # turn right
        phi_l_dot, phi_r_dot = 0.32*MAX_SPEED, -0.05*MAX_SPEED
            
    elif (g[0]<350): # turn left
        phi_l_dot, phi_r_dot = -0.05*MAX_SPEED, 0.32*MAX_SPEED
        
    
    # Calculate the incremental delta_x and omega_z for the
    # current timestep and store them in variables called
    # delta_x_new and omega_z_new.
    delta_x_new = (r*(phi_l_dot + phi_r_dot))/2 * delta_t
    omega_z_new = (r*(phi_r_dot - phi_l_dot))/d * delta_t

   
   
    # Use the incremental delta_x and omega_z values calculated
    # above to calculate the new x, y and alpha in the world
    # coordinate frame.
    alpha += omega_z_new
    x_w += (np.cos(alpha) * delta_x_new)
    y_w += (np.sin(alpha) * delta_x_new)
    
    
    # Integrate/Add the current delta_x and omega_z values to
    # the cumulative values so far.
    omega_z += omega_z_new
    delta_x += delta_x_new
   
    
    # Calculate the position error w.r.t the starting point (0,0)
    # using the Euclidian distance.
    error = np.sqrt((x_w**2 + y_w**2))
   
   
    # Print out x_w, y_w and alpha to the console. The angle alpha
    # is converted from radians to degrees for easier display.
    print(
            f'x_w => {x_w:0,.20f},', 
            f'y_w => {y_w:0,.20f},', 
            f'alpha => {(alpha/3.14)*180}'
           )
    
    # The error to 3 decimal places is printed to the console. 
    print(f'error => {error*100:0,.3f}cm')
    print()
    
    
    # Actuator values are set.
    motor_left.setVelocity(phi_l_dot)
    motor_right.setVelocity(phi_r_dot)
    
    
    # Check if the robot has crossed the line, 
    # stop the simulation of it has.
    if over_the_line:
        sys.exit(0)
        
    
    pass

# Enter here exit cleanup code.
