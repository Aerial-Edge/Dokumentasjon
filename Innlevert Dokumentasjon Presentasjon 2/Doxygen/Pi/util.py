import time
import numpy as np
from pymavlink_msgs.msg import DronePose
import qtm

def calculatePosition2(positions, current_yaw):
    """
    Mock function for position calculation, currently returns a constant value.
    This code is borrowed from Hilde Marie Moholt.

    :param positions: The positions of the drone.
    :param current_yaw: The current yaw angle of the drone.
    :return: Returns a constant value of 2.
    """
    return 2

# Initialize some global variables
start_time_ns = time.time_ns() # Current time in nanoseconds
start_time = time.time() # Current time in seconds
time_prev = 0 # Previous timestamp
prev_vel = np.zeros(3) # Previous velocity
yaw = 0 # Current yaw angle
data_failure_prev=True # Previous data failure flag

def calculatePosition(positions, current_yaw):
    """
    Function to calculate the drone's position and velocity.

    :param positions: The positions of the drone.
    :param current_yaw: The current yaw angle of the drone.
    :return: Returns a DronePose object containing position, velocity, acceleration, yaw, and yaw velocity.
    """
    # Declare the use of global variables
    global time_prev, prev_pos, prev_vel, yaw, has_initialized, data_failure_prev

    # Initialize some variables
    data_failure = False
    qualisys_yaw=current_yaw
    time_now = time.time() - start_time
    time_dif = time_now - time_prev
    time_dif_hz = 1/(time_dif)
    q_pos=positions

    # Check for invalid data
    if (np.sum(np.isnan(q_pos))>0):
        data_failure=True
        data_failure_prev=True
        return DronePose()
    elif data_failure_prev:
        prev_pos=positions
    data_failure_prev=False

    # Calculate velocity and acceleration
    vel = (q_pos - prev_pos)*time_dif_hz
    acc = (vel-prev_vel)*time_dif_hz

    # Handle yaw wraparound
    diff = qualisys_yaw - yaw
    if(diff > np.pi):
        diff -= 2*np.pi
    elif(diff < -np.pi):
        diff += 2*np.pi
    yaw_vel = diff/time_dif

    # Populate DronePose message
    positionalData = DronePose()
    positionalData.pos.x = q_pos[0]
    positionalData.pos.y = q_pos[1]
    positionalData.pos.z = q_pos[2]
    positionalData.vel.x = vel[0]
    positionalData.vel.y = vel[1]
    positionalData.vel.z = vel[2]
    positionalData.accel.x = acc[0]
    positionalData.accel.y = acc[1]
    positionalData.accel.z = acc[2]
    positionalData.yaw.data = qualisys_yaw
    positionalData.yawVel.data = yaw_vel

    # Update global variables
    time_prev = time_now
    prev_pos= q_pos
    yaw = qualisys_yaw
    prev_vel = vel

    if not data_failure:
        has_initialized = True
    
    return positionalData

######################
def angle_diff(angle_1, angle_2):
    """
    Compute the difference between two angles, handling wraparound.

    :param angle_1: The first angle, in radians.
    :param angle_2: The second angle, in radians.
    :return: The difference between the two angles, in radians, wrapped to [-pi, pi].
    """
    diff=angle_1-angle_2
    while (diff > np.pi):
        diff -= 2*np.pi
    while (diff < -np.pi):
        diff += 2*np.pi
    return diff

def calculate_yaw(rot_matrix):
    """
    Function to calculate yaw angle from a rotation matrix.
    Returns None if we have a failure at any point.

    :param rot_matrix: A 3x3 rotation matrix.
    :return: The calculated yaw angle, or None if a calculation error occurred.
    """
    if np.isnan(rot_matrix).any():
        return None
    pitch = np.arcsin(rot_matrix[0,2]) # Formula to calculate pitch
    vinkel=np.clip(rot_matrix[0,0]/np.cos(pitch),-1,1)
    current_yaw = np.arccos(vinkel)   # Formula to calculate yaw
    if np.isnan(current_yaw):
        return None
    if rot_matrix[1,0] < 0: # Checks if yaw value should be positive or negative
        current_yaw *= -1
    return current_yaw

def rotation_matrix(yaw,pitch,roll):
    """
    Function to calculate a rotation matrix given yaw, pitch, and roll.
    This function is not used in qualisys, but is used to test qualisys.

    :param yaw: The yaw angle.
    :param pitch: The pitch angle.
    :param roll: The roll angle.
    :return: A 3x3 rotation matrix.
    """
    return np.matrix([[np.cos(yaw)*np.cos(pitch), (np.cos(yaw)*np.sin(pitch)*np.sin(roll))-(np.sin(yaw)*np.cos(roll)), (np.cos(yaw)*np.sin(pitch)*np.cos(roll))+(np.sin(yaw)*np.sin(roll))],
    [np.sin(yaw)*np.cos(pitch), (np.sin(yaw)*np.sin(pitch)*np.sin(roll))+(np.cos(yaw)*np.cos(roll)), (np.sin(yaw)*np.sin(pitch)*np.cos(roll))-(np.cos(yaw)*np.sin(roll))],
    [-np.sin(pitch),np.cos(pitch)*np.sin(roll),np.cos(pitch)*np.cos(roll)]])