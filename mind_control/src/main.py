#!/usr/bin/env python3

import sys
import rospy
import trajectory_msgs.msg
from control_msgs.msg import FollowJointTrajectoryActionResult
from flask import Flask, request # Create a web server receiving HTTP POST requests
import logging                   # Suppress Flask logging messages
from tqdm import tqdm            # Show progress bars
import time
import threading

# Configure logging
log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)

# Flask application instance
app = Flask(__name__)

# ROS node name
NODE_NAME = 'mind_control'

# Variables and Constants
lock = threading.Lock()  # Lock for synchronization or lock = False
gripper_state = {'left': None, 'right': None}
initial_timestamp = None
GRIPPER_CLOSE_POSE = 0.0
GRIPPER_OPEN_POSE = 0.09
TOLERANCE = 0.01
tilt = "center"
MAX_BUILD = 50 # Threshold for triggering an action based on received inputs
prev_time = 0
openning = 0
closing = 0

# Robot Joint Configuration
GRIPPER_L_NAMES = ['gripper_left_finger_joint', 'gripper_left_inner_finger_joint']
GRIPPER_R_NAMES = ['gripper_right_finger_joint', 'gripper_right_inner_finger_joint']

# ROS Node Initialization
rospy.init_node(NODE_NAME, disable_signals=True)
print('Running as:', NODE_NAME)

# Publishers
gripper_l_pub = rospy.Publisher('/parallel_gripper_left_controller/command', trajectory_msgs.msg.JointTrajectory, queue_size=10)
gripper_r_pub = rospy.Publisher('/parallel_gripper_right_controller/command', trajectory_msgs.msg.JointTrajectory, queue_size=10)

# Subscribers and callback functions
def gripper_state_callback(data):
    global gripper_state
    gripper_state['left'] = data
    gripper_state['right'] = data

gripper_l_sub = rospy.Subscriber('/parallel_gripper_left_controller/state', FollowJointTrajectoryActionResult, gripper_state_callback)
gripper_r_sub = rospy.Subscriber('/parallel_gripper_right_controller/state', FollowJointTrajectoryActionResult, gripper_state_callback)

def wait_for_controller(controller):
    while controller.get_num_connections() == 0:
        rospy.loginfo(f'Waiting for {controller.resolved_name} controller...')
        rospy.sleep(1.0)

wait_for_controller(gripper_l_pub)
wait_for_controller(gripper_r_pub)
wait_for_controller(gripper_l_sub)
wait_for_controller(gripper_r_sub)

def check_gripper_position(joint_name, target_position):
    global gripper_state
    state = gripper_state.get('left') if joint_name in GRIPPER_L_NAMES else gripper_state.get('right')

    if state:
        actual_position = state.actual.positions[0]
        return abs(actual_position - target_position) < TOLERANCE
    return False

def move_gripper(position):
    global gripper_l_pub, gripper_r_pub, tilt

    with lock:
        current_tilt = tilt
        action_name = 'Open' if position == GRIPPER_OPEN_POSE else 'Clos'
        rospy.loginfo(f'{action_name}ing gripper to position: {position}')
        print(f'{action_name}ing gripper to position: {position}')

        traj_left = trajectory_msgs.msg.JointTrajectory()
        traj_right = trajectory_msgs.msg.JointTrajectory()

        traj_left.joint_names = GRIPPER_L_NAMES
        traj_right.joint_names = GRIPPER_R_NAMES

        p = trajectory_msgs.msg.JointTrajectoryPoint()
        p.positions = [position, position]
        p.time_from_start = rospy.Duration(2)

        traj_left.points = [p]
        traj_right.points = [p]

        if current_tilt == "center":
            gripper_l_pub.publish(traj_left)
            gripper_r_pub.publish(traj_right)

            # Wait for gripper to reach the target position
            while not check_gripper_position('gripper_left_finger_joint', position):
                rospy.sleep(0.1)

            while not check_gripper_position('gripper_right_finger_joint', position):
                rospy.sleep(0.1)

        elif current_tilt == "left":
            gripper_l_pub.publish(traj_left)

            # Wait for gripper to reach the target position
            while not check_gripper_position('gripper_left_finger_joint', position):
                rospy.sleep(0.1)

        elif current_tilt == "right":
            gripper_r_pub.publish(traj_right)

            # Wait for gripper to reach the target position
            while not check_gripper_position('gripper_right_finger_joint', position):
                rospy.sleep(0.1)

## Flask Web Server Endpoints
# Node-RED input data from the Emotiv headset
# Gripper control
@app.route('/emotiv/gripper/open', methods=['POST'])
def handle_open_gripper():
    global lock, openning, prev_time, initial_timestamp

    if lock.locked():
        return 'LOCKED'

    try:
        request_data = request.get_json()
        timestamp = request_data['timestamp']
        print(f"Received timestamp for open: {timestamp}")

        if openning == 0:
            timestamp_seconds = time.mktime(time.strptime(timestamp, "%Y-%m-%dT%H:%M:%S.%fZ"))
            initial_timestamp = timestamp_seconds
            print(f"Initial timestamp for open: {initial_timestamp}")

        # Progress bar initialization
        with tqdm(total=MAX_BUILD) as pbar:
            # Time calculation
            time_now = time.time() * 1000
            time_diff = time_now - prev_time
            if time_diff > 2000:
                print('Reset.')
                rospy.loginfo('Reset.')
                openning = 0
                initial_timestamp = timestamp_seconds  # Reset initial timestamp

            # openning and timestamp update
            openning += 1
            pbar.update(openning)
            prev_time = time_now

            if openning == MAX_BUILD:
                openning = 0
                print('[', time_now, ']', 'Signal received, opening gripper...')
                rospy.loginfo(f'[{time_now}] Signal received, opening gripper...')
                move_gripper(GRIPPER_OPEN_POSE)
                duration = (time.time() * 1000) - initial_timestamp
                print(f'Gripper opened at {time_now}. Action duration: {duration}')
                return f'Gripper opened at {time_now}. Action duration: {duration}', 200

        return 'Progressing', 200

    except Exception as e:
        logging.error(f"Error in /emotiv/gripper/open endpoint: {e}", exc_info=True)
        print(f"Error in /emotiv/gripper/open endpoint: {e}")
        return 'Error', 500

@app.route('/emotiv/gripper/close', methods=['POST'])
def handle_close_gripper():
    global lock, closing, prev_time, initial_timestamp

    if lock.locked():
        return 'LOCKED'

    try:
        request_data = request.get_json()
        timestamp = request_data['timestamp']
        print(f"Received timestamp for close: {timestamp}")

        if closing == 0:
            timestamp_seconds = time.mktime(time.strptime(timestamp, "%Y-%m-%dT%H:%M:%S.%fZ"))
            initial_timestamp = timestamp_seconds
            print(f"Initial timestamp for close: {initial_timestamp}")

        # Progress bar initialization
        with tqdm(total=MAX_BUILD) as pbar:
            # Time calculation
            time_now = time.time() * 1000
            time_diff = time_now - prev_time
            if time_diff > 2000:
                print('Reset.')
                rospy.loginfo('Reset.')
                closing = 0
                initial_timestamp = timestamp_seconds  # Reset initial timestamp

            # closing and timestamp update
            closing += 1
            pbar.update(closing)
            prev_time = time_now

            if closing == MAX_BUILD:
                closing = 0
                print('[', time_now, ']', 'Signal received, closing gripper...')
                rospy.loginfo(f'[{time_now}] Signal received, closing gripper...')
                move_gripper(GRIPPER_CLOSE_POSE)
                duration = (time.time() * 1000) - initial_timestamp
                print(f'Gripper closed at {time_now}. Action duration: {duration}')
                return f'Gripper closed at {time_now}. Action duration: {duration}', 200

        return 'Progressing', 200

    except Exception as e:
        logging.error(f"Error in /emotiv/gripper/close endpoint: {e}", exc_info=True)
        print(f"Error in /emotiv/gripper/close endpoint: {e}")
        return 'Error', 500

# Magnetometer data
@app.route('/emotiv/magnetometer/X', methods=['POST'])
def magnetometerX():
    request_data = request.get_json()
    magX = request_data['magX']

    return 'Magnetometer X data received', 200

@app.route('/emotiv/magnetometer/Y', methods=['POST'])
def magnetometerY():
    global tilt

    request_data = request.get_json()
    magY = request_data['magY']
    print(f'Magnetometer Y data received: {magY}')

    # Determine tilt direction to control both arms, the left or right one
    tilt = "center"
    if magY < 8110:
        tilt = "left"
    elif magY > 8190:
        tilt = "right"
    print(f'Tilt direction set to: {tilt}')

    return 'Magnetometer Y data received, tilt direction set to: {tilt}', 200

@app.route('/emotiv/magnetometer/Z', methods=['POST'])
def magnetometerZ():
    request_data = request.get_json()
    magZ = request_data['magZ']

    return 'Magnetometer Z data received', 200

# Starts the Flask web server 
# on all available IP addresses (0.0.0.0) and port 5014
if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5014)
