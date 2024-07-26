#!/usr/bin/env python3

import sys
import rospy
import trajectory_msgs.msg
from control_msgs.msg import FollowJointTrajectoryActionResult
from flask import Flask, request
import logging
from tqdm import tqdm
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
lock = threading.Lock()
gripper_state = {'left': None, 'right': None}
GRIPPER_CLOSE_POSE = 0.0
GRIPPER_OPEN_POSE = 0.09
TOLERANCE = 0.01
tilt = "center"
MAX_BUILD = 50
prev_time = 0
openning = 0
closing = 0

GRIPPER_L_NAMES = ['gripper_left_finger_joint', 'gripper_left_inner_finger_joint']
GRIPPER_R_NAMES = ['gripper_right_finger_joint', 'gripper_right_inner_finger_joint']

rospy.init_node(NODE_NAME, disable_signals=True)
print('Running as:', NODE_NAME)

gripper_l_pub = rospy.Publisher('/parallel_gripper_left_controller/command', trajectory_msgs.msg.JointTrajectory, queue_size=10)
gripper_r_pub = rospy.Publisher('/parallel_gripper_right_controller/command', trajectory_msgs.msg.JointTrajectory, queue_size=10)

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
        action_name = 'Opening' if position == GRIPPER_OPEN_POSE else 'Closing'
        rospy.loginfo(f'{action_name} gripper to position: {position}')

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

            while not check_gripper_position('gripper_left_finger_joint', position):
                rospy.sleep(0.1)

            while not check_gripper_position('gripper_right_finger_joint', position):
                rospy.sleep(0.1)

        elif current_tilt == "left":
            gripper_l_pub.publish(traj_left)
            while not check_gripper_position('gripper_left_finger_joint', position):
                rospy.sleep(0.1)

        elif current_tilt == "right":
            gripper_r_pub.publish(traj_right)
            while not check_gripper_position('gripper_right_finger_joint', position):
                rospy.sleep(0.1)

        return f'Gripper {action_name.lower()}ed at {time.time() * 1000}', 200

def handle_gripper_action(build_counter, action_func):
    global prev_time

    try:
        request_data = request.get_json()
        timestamp = request_data['timestamp']

        with tqdm(total=MAX_BUILD) as pbar:
            time_now = time.time() * 1000
            time_diff = time_now - prev_time
            if time_diff > 2000:
                print('Reset.')
                rospy.loginfo('Reset.')
                build_counter = 0

            build_counter += 1
            pbar.update(build_counter)
            prev_time = time_now

            if build_counter == MAX_BUILD:
                build_counter = 0
                print(f'[{time_now}] Signal received, performing action...')
                rospy.loginfo(f'[{time_now}] Signal received, performing action...')
                return action_func()

        return 'Progressing', 200

    except Exception as e:
        logging.error(f"Error in gripper action endpoint: {e}", exc_info=True)
        return 'Error', 500

@app.route('/emotiv/gripper/open', methods=['POST'])
def handle_open_gripper():
    global openning

    if lock.locked():
        return 'LOCKED'
        
    return handle_gripper_action(openning, lambda: move_gripper(GRIPPER_OPEN_POSE))

@app.route('/emotiv/gripper/close', methods=['POST'])
def handle_close_gripper():
    global closing

    if lock.locked():
        return 'LOCKED'

    return handle_gripper_action(closing, lambda: move_gripper(GRIPPER_CLOSE_POSE))

@app.route('/emotiv/gyroscope/X', methods=['POST'])
def gyroscopeX():
    request_data = request.get_json()
    gyrX = request_data['gyrX']
    return 'Gyroscope X data received', 200

@app.route('/emotiv/gyroscope/Y', methods=['POST'])
def gyroscopeY():
    request_data = request.get_json()
    gyrY = request_data['gyrY']
    return 'Gyroscope Y data received', 200

@app.route('/emotiv/gyroscope/Z', methods=['POST'])
def gyroscopeZ():
    global tilt

    request_data = request.get_json()
    gyrZ = request_data['gyrZ']

    tilt = "center"
    if gyrZ > 0.1:
        tilt = "left"
    elif gyrZ < -0.1:
        tilt = "right"

    return 'Gyroscope Z data received', 200

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5014)
