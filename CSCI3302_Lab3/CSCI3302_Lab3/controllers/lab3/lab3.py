"""lab3 controller."""
# Copyright University of Colorado Boulder 2022
# CSCI 3302 "Introduction to Robotics" Lab 3 Base Code.

from controller import Robot, Motor
import math

# TODO: Fill out with correct values from Robot Spec Sheet (or inspect PROTO definition for the robot)
MAX_SPEED = 2.84 # [rad/s]
MAX_SPEED_MS = .22 # [m/s]
AXLE_LENGTH = .00160 # [m]



MOTOR_LEFT = 0 # Left wheel index
MOTOR_RIGHT = 1 # Right wheel index

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# The Turtlebot robot has two motors
part_names = ("left wheel motor", "right wheel motor")


# Set wheels to velocity control by setting target position to 'inf'
# You should not use target_pos for storing waypoints. Leave it unmodified and 
# use your own variable to store waypoints leading up to the goal
target_pos = ('inf', 'inf') 
robot_parts = []

for i in range(len(part_names)):
        robot_parts.append(robot.getDevice(part_names[i]))
        robot_parts[i].setPosition(float(target_pos[i]))

# Odometry
pose_x     = 0
pose_y     = 0
pose_theta = 0

# Rotational Motor Velocity [rad/s]
vL = 0
vR = 0

# TODO
# Create you state and goals (waypoints) variable here
# You have to MANUALLY figure out the waypoints, one sample is provided for you in the instructions

while robot.step(timestep) != -1:

    # STEP 2.1: Calculate error with respect to current and goal position
    pass   

    #Heading error
    #gole pose - current pose = position remaining. take atan2(y,x) and thats the theta we want 
    goal_pose = (3,4) #TODO set this later
    x = goal_pose[0] - pose_x
    y = goal_pose[1] - pose_y
    theta = math.atan2(y,x)
    #check is calcd theta = given theta
    if(pose_theta - theta < 0.5 and pose_theta - theta > -0.5): #if within 1 radian (might need to change later)
        pass
    
    # STEP 2.2: Feedback Controller
    pass
    
    # STEP 1: Inverse Kinematics Equations (vL and vR as a function dX and dTheta)
    # Note that vL and vR in code is phi_l and phi_r on the slides/lecture
    vL = (dX - (dTheta/2))/(MAX_SPEED_MS/MAX_SPEED)
    vR = (dX - (dTheta/2))/(MAX_SPEED_MS/MAX_SPEED)
    pass
    
    # STEP 2.3: Proportional velocities
    vL = 0 # Left wheel velocity in rad/s
    vR = 0 # Right wheel velocity in rad/s
    pass

    # STEP 2.4: Clamp wheel speeds
    pass


    
    # TODO
    # Use Your Lab 2 Odometry code after these 2 comments. We will supply you with our code next week 
    # after the Lab 2 deadline but you free to use your own code if you are sure about its correctness
    
    # NOTE that the odometry should ONLY be a function of 
    # (vL, vR, MAX_SPEED, MAX_SPEED_MS, timestep, AXLE_LENGTH, pose_x, pose_y, pose_theta)
    # Odometry code. Don't change speeds (vL and vR) after this line
    
    
    

    ########## End Odometry Code ##################
    
    ########## Do not change ######################
    # Bound pose_theta between [-pi, 2pi+pi/2]
    # Important to not allow big fluctuations between timesteps (e.g., going from -pi to pi)
    if pose_theta > 6.28+3.14/2: pose_theta -= 6.28
    if pose_theta < -3.14: pose_theta += 6.28
    ###############################################

    # TODO
    # Set robot motors to the desired velocities
    robot_parts[MOTOR_LEFT].setVelocity(0)
    robot_parts[MOTOR_RIGHT].setVelocity(0)

    