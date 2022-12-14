"""lab3 controller."""
# Copyright University of Colorado Boulder 2022
# CSCI 3302 "Introduction to Robotics" Lab 3 Base Code.

from controller import Robot, Motor
import math

# TODO: Fill out with correct values from Robot Spec Sheet (or inspect PROTO definition for the robot)
MAX_SPEED = 6.67 # [rad/s]
MAX_SPEED_MS = .22 # [m/s]
AXLE_LENGTH = .160 # [m]
AXEL_RADIUS = AXLE_LENGTH/2

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

global waypoint_counter
waypoint_counter = 0

for i in range(len(part_names)):
        robot_parts.append(robot.getDevice(part_names[i]))
        robot_parts[i].setPosition(float(target_pos[i]))

# Odometry
global pose_x, pose_y, pose_theta
pose_x     = -8
pose_y     = -5
pose_theta = 0.00231134 #radians

# Rotational Motor Velocity [rad/s]
global vL, vR, distance_error, bearing_error, heading_error
vL = 0
vR = 0


def feedback_controller():
    global vL, vR, distance_error, bearing_error, heading_error, waypoint_counter
    print("feedback_controller1", vL, vR)
    if distance_error > .015:
        distance_constant = .2
        if distance_error > .04:
            phi_l = (distance_error*distance_constant - (bearing_error*AXLE_LENGTH)/2)/AXEL_RADIUS
            phi_r = (distance_error*distance_constant + (bearing_error*AXLE_LENGTH)/2)/AXEL_RADIUS
        else:
            phi_l = (distance_error - (heading_error*AXLE_LENGTH)/2)/AXEL_RADIUS
            phi_r = (distance_error + (heading_error*AXLE_LENGTH)/2)/AXEL_RADIUS
            if (waypoint_counter == 0):
                waypoint_counter = waypoint_counter + 1

        if phi_l > phi_r:
            vL = (MAX_SPEED/4) * (phi_l/phi_r)
            vR = (MAX_SPEED/4)
        elif phi_l < phi_r:
            vL = (MAX_SPEED/4)
            vR = (MAX_SPEED/4) * (phi_r/phi_l)
        else:
            vL = MAX_SPEED/2
            vR = MAX_SPEED/2
    else:
        vL = 0
        vR = 0
        waypoint_counter = waypoint_counter + 1
    print("feedback_controller1", vL, vR)



def heading():
    global vL, vR, distance_error, bearing_error, heading_error
    if heading_error < -.01:
        vL = MAX_SPEED/2
        vR = -MAX_SPEED/2
    elif heading_error > .01:
        vL = -MAX_SPEED/2
        vR = MAX_SPEED/2
    else:
        vL = 0
        vR = 0
    print("heading", vL, vR)

def distance():
    global vL, vR, distance_error, bearing_error, heading_error
    if distance_error > .01:
        vL = MAX_SPEED/2
        vR = MAX_SPEED/2
    elif distance_error < .01:
        vL = 0
        vR = 0
        heading()
    print("distance", vL, vR)
            

def bearing():
    global vL, vR, distance_error, bearing_error, heading_error
    if bearing_error < -.01:
        vL = MAX_SPEED/2
        vR = -MAX_SPEED/2
    elif bearing_error > .01:
        vL = -MAX_SPEED/2
        vR = MAX_SPEED/2
    else:
        vL = 0
        vR = 0
        distance()
    print("bearing", vL, vR)


# TODO
# Create you state and goals (waypoints) variable here
# You have to MANUALLY figure out the waypoints, one sample is provided for you in the instructions
waypoint_1 = (-6,-6.00)
waypoint_2 = (-3.5,-3)
waypoints = [waypoint_1,waypoint_2]
while robot.step(timestep) != -1:

    # STEP 2.1: Calculate error with respect to current and goal position
    distance_error = math.sqrt((pose_x - waypoints[waypoint_counter][0])**2+(pose_y - waypoints[waypoint_counter][1])**2)
    bearing_error = math.atan((pose_y - waypoints[waypoint_counter][1])/(pose_x - waypoints[waypoint_counter][0])) - pose_theta
    heading_error = math.atan2((waypoints[waypoint_counter][1] - pose_y),(waypoints[waypoint_counter][0] - pose_x))
    bearing()
    
    # STEP 2.2: Feedback Controller
    #move to feedback controller class
    feedback_controller()
    print("default1", vL, vR)
    
    # STEP 1: Inverse Kinematics Equations (vL and vR as a function dX and dTheta)
    # Note that vL and vR in code is phi_l and phi_r on the slides/lecture
    #vL = (dX - (dTheta/2))/(MAX_SPEED_MS/MAX_SPEED)
    #vR = (dX - (dTheta/2))/(MAX_SPEED_MS/MAX_SPEED)
    pass
    
    # STEP 2.3: Proportional velocities
    #vL = 0 # Left wheel velocity in rad/s
    #vR = 0 # Right wheel velocity in rad/s
    pass

    # STEP 2.4: Clamp wheel speeds
    pass


    
    # TODO
    # Use Your Lab 2 Odometry code after these 2 comments. We will supply you with our code next week 
    # after the Lab 2 deadline but you free to use your own code if you are sure about its correctness
    
    # NOTE that the odometry should ONLY be a function of 
    # (vL, vR, MAX_SPEED, MAX_SPEED_MS, timestep, AXLE_LENGTH, pose_x, pose_y, pose_theta)
    # Odometry code. Don't change speeds (vL and vR) after this line

    distL = vL/MAX_SPEED * MAX_SPEED_MS * timestep/1000.0

    distR = vR/MAX_SPEED * MAX_SPEED_MS * timestep/1000.0

    pose_x += (distL+distR) / 2.0 * math.cos(pose_theta)

    pose_y += (distL+distR) / 2.0 * math.sin(pose_theta)

    pose_theta += (distR-distL)/AXLE_LENGTH
    print("default2", vL, vR)
    

    ########## End Odometry Code ##################
    
    ########## Do not change ######################
    # Bound pose_theta between [-pi, 2pi+pi/2]
    # Important to not allow big fluctuations between timesteps (e.g., going from -pi to pi)
    if pose_theta > 6.28+3.14/2: pose_theta -= 6.28
    if pose_theta < -3.14: pose_theta += 6.28
    ###############################################

    # TODO
    # Set robot motors to the desired velocities
    robot_parts[MOTOR_LEFT].setVelocity(vL)
    robot_parts[MOTOR_RIGHT].setVelocity(vR)

    