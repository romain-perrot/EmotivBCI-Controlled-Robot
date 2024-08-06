#!/usr/bin/env python3

import rospy
import trajectory_msgs.msg
from control_msgs.msg import JointTrajectoryControllerState
from datetime import datetime, timezone
from flask import Flask, request, jsonify   # Create a web server receiving HTTP POST requests
import logging                              # Suppress Flask logging messages
from tqdm import tqdm                       # Show progress bars
import threading
import csv
import numpy as np
from joblib import load

# Configure logging
log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)

# Flask application instance
app = Flask(__name__)

# ROS node name
NODE_NAME = 'mind_control'

# Variables and Constants
MAX_BUILD = 50 # Threshold for triggering an action based on received inputs
gripper_state = {'left_arm': None, 'right_arm': None}
lock = threading.Lock()  # Lock for synchronization
initial_timestamp = None
GRIPPER_CLOSE_POSE = 0.0
GRIPPER_OPEN_POSE = 0.09
PRECISION = 0.001
TOLERANCE = 0.01
tilt = "both_arm"
prev_time = None
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
def gripper_state_callback(data, gripper_side):
    global gripper_state
    gripper_state[gripper_side] = data

gripper_l_sub = rospy.Subscriber('/parallel_gripper_left_controller/state', JointTrajectoryControllerState, gripper_state_callback, callback_args='left')
gripper_r_sub = rospy.Subscriber('/parallel_gripper_right_controller/state', JointTrajectoryControllerState, gripper_state_callback, callback_args='right')

def wait_for_controller(controller):
    while controller.get_num_connections() == 0:
        rospy.loginfo(f'Waiting for {controller.resolved_name} controller...')
        rospy.sleep(1.0)
    rospy.loginfo(f'{controller.resolved_name} controller is now connected.')

wait_for_controller(gripper_l_pub)
wait_for_controller(gripper_r_pub)

def check_gripper_position(joint_name, target_position):
    global gripper_state
    state = gripper_state.get('left_arm') if joint_name in GRIPPER_L_NAMES else gripper_state.get('right_arm')

    if state:
        actual_position = state.actual.positions[0]
        return abs(actual_position - target_position) < TOLERANCE
    return False

def move_gripper(position):
    global gripper_l_pub, gripper_r_pub, tilt, current_tilt

    with lock:
        current_tilt = tilt
        action_name = 'Open' if position == GRIPPER_OPEN_POSE else 'Clos'
        rospy.loginfo(f'{action_name}ing gripper to position: {position}')

        traj_left = trajectory_msgs.msg.JointTrajectory()
        traj_right = trajectory_msgs.msg.JointTrajectory()

        traj_left.joint_names = GRIPPER_L_NAMES
        traj_right.joint_names = GRIPPER_R_NAMES

        p = trajectory_msgs.msg.JointTrajectoryPoint()
        p.positions = [position, position]
        p.time_from_start = rospy.Duration(2)

        traj_left.points = [p]
        traj_right.points = [p]

        if current_tilt == "both_arm":
            gripper_l_pub.publish(traj_left)
            gripper_r_pub.publish(traj_right)

            # Wait for gripper to reach the target position
            while not check_gripper_position('gripper_left_finger_joint', position):
                rospy.sleep(0.1)

            while not check_gripper_position('gripper_right_finger_joint', position):
                rospy.sleep(0.1)

        elif current_tilt == "left_arm":
            gripper_l_pub.publish(traj_left)

            # Wait for gripper to reach the target position
            while not check_gripper_position('gripper_left_finger_joint', position):
                rospy.sleep(0.1)

        elif current_tilt == "right_arm":
            gripper_r_pub.publish(traj_right)

            # Wait for gripper to reach the target position
            while not check_gripper_position('gripper_right_finger_joint', position):
                rospy.sleep(0.1)
            
        return current_tilt

def format_duration(duration):
    hours, remainder = divmod(duration.total_seconds(), 3600)
    minutes, seconds = divmod(remainder, 60)
    milliseconds = duration.microseconds // 1000
    return f'{int(hours)}h {int(minutes)}m {int(seconds)}s {milliseconds}ms'

def log_result(initial_timestamp, end_timestamp, duration, action_tilt):
    fieldnames = ['Start time', 'End time', 'Duration', 'Arm controlled', 'Success <= 15s', 'Accuracy', 'Error rate']

    # Command Accuracy: Track how often the robot correctly interprets and executes the commands as intended.
    # Accuracy = Total Number of Predictions/Number of Correct Predictions × 100
    # Error Rate: Calculate the percentage of incorrect actions taken by the robot.

    # Open the file in append mode
    with open('gripper_action_log.csv', mode='a', newline='') as file:
        writer = csv.DictWriter(file, fieldnames=fieldnames)
        
        # Write header only if file is empty
        file_empty = file.tell() == 0
        if file_empty:
            writer.writeheader()

        # Check if the duration is within the threshold of 15 seconds
        threshold_succeeded = duration.total_seconds() <= 15

        # Write the log entry
        writer.writerow({
            'Start time': initial_timestamp,
            'End time': end_timestamp,
            'Duration': duration,
            'Arm controlled': action_tilt,
            'Success <= 15s': threshold_succeeded,
            'Accuracy': '',  # Leave this empty for manual input
            'Error rate': ''  # Leave this empty for manual input
        })

def handle_neutral_state():
    print('In neutral state, awaiting commands')
    return 'In neutral state, awaiting commands'

def handle_open_gripper():
    global lock, openning, prev_time, initial_timestamp

    if lock.locked():
        return 'LOCKED'

    try:
        request_data = request.get_json()
        timestamp_str = request_data['timestamp']
        print(f"Received timestamp for open: {timestamp_str}")

        if openning == 0:
            timestamp = datetime.strptime(timestamp_str, "%Y-%m-%dT%H:%M:%S.%fZ").replace(tzinfo=timezone.utc)
            initial_timestamp = timestamp
            print(f"Initial timestamp for open: {initial_timestamp}")

        # Progress bar initialization
        with tqdm(total=MAX_BUILD) as pbar:
            # Time calculation
            time_now = datetime.now(timezone.utc)
            time_diff = (time_now - prev_time).total_seconds() if prev_time else 0
            if time_diff > 2:
                rospy.loginfo('Reset.')
                openning = 0
                initial_timestamp = timestamp

            # Update openning and prev_time
            openning += 1
            pbar.update(openning)
            prev_time = time_now

            # Start action
            if openning == MAX_BUILD:
                openning = 0
                rospy.loginfo(f'[{time_now}] Signal received, opening gripper...')
                action_tilt = move_gripper(GRIPPER_OPEN_POSE)
                end_timestamp = datetime.now(timezone.utc)
                duration = datetime.now(timezone.utc) - initial_timestamp
                formatted_duration = format_duration(duration)
                print(f'Gripper opened at {end_timestamp} with starting time at: {initial_timestamp}. Action duration: {formatted_duration}')

                # Log the result
                log_result(initial_timestamp, end_timestamp, formatted_duration, action_tilt)

                return f'Gripper opened at {end_timestamp} with starting time at: {initial_timestamp}. Action duration: {formatted_duration}', 200

        return 'Progressing', 200

    except Exception as e:
        logging.error(f"Error in /emotiv/gripper/open endpoint: {e}", exc_info=True)
        return 'Error', 500

def handle_close_gripper():
    global lock, closing, prev_time, initial_timestamp

    if lock.locked():
        return 'LOCKED'

    try:
        request_data = request.get_json()
        timestamp_str = request_data['timestamp']
        print(f"Received timestamp for close: {timestamp_str}")

        if closing == 0:
            timestamp = datetime.strptime(timestamp_str, "%Y-%m-%dT%H:%M:%S.%fZ").replace(tzinfo=timezone.utc)
            initial_timestamp = timestamp
            print(f"Initial timestamp for close: {initial_timestamp}")

        # Progress bar initialization
        with tqdm(total=MAX_BUILD) as pbar:
            # Time calculation
            time_now = datetime.now(timezone.utc)
            time_diff = (time_now - prev_time).total_seconds() if prev_time else 0
            if time_diff > 2:
                rospy.loginfo('Reset.')
                closing = 0
                initial_timestamp = timestamp

            # Update closing and prev_time
            closing += 1
            pbar.update(closing)
            prev_time = time_now

            # Start action
            if closing == MAX_BUILD:
                closing = 0
                rospy.loginfo(f'[{time_now}] Signal received, closing gripper...')
                action_tilt = move_gripper(GRIPPER_CLOSE_POSE)
                end_timestamp = datetime.now(timezone.utc)
                duration = datetime.now(timezone.utc) - initial_timestamp
                formatted_duration = format_duration(duration)
                print(f'Gripper closed at {end_timestamp} with starting time at: {initial_timestamp}. Action duration: {formatted_duration}')

                # Log the result
                log_result(initial_timestamp, end_timestamp, formatted_duration, action_tilt)

                return f'Gripper closed at {end_timestamp} with starting time at: {initial_timestamp}. Action duration: {formatted_duration}', 200

        return 'Progressing', 200

    except Exception as e:
        logging.error(f"Error in /emotiv/gripper/close endpoint: {e}", exc_info=True)
        return 'Error', 500

def increment_gripper():
    global current_gripper_position

    try:
        increment = float(request.args.get('value', PRECISION))
        new_position = min(current_gripper_position + increment, GRIPPER_OPEN_POSE)
        move_gripper(new_position)
        current_gripper_position = new_position
        return jsonify({'status': 'success', 'new_position': new_position}), 200

    except Exception as e:
        logging.error(f"Error in /emotiv/gripper/increment endpoint: {e}", exc_info=True)
        return jsonify({'status': 'error', 'message': str(e)}), 500

def decrement_gripper():
    global current_gripper_position

    try:
        decrement = float(request.args.get('value', PRECISION))
        new_position = max(current_gripper_position - decrement, GRIPPER_CLOSE_POSE)
        move_gripper(new_position)
        current_gripper_position = new_position
        return jsonify({'status': 'success', 'new_position': new_position}), 200

    except Exception as e:
        logging.error(f"Error in /emotiv/gripper/decrement endpoint: {e}", exc_info=True)
        return jsonify({'status': 'error', 'message': str(e)}), 500

## Flask Web Server Endpoints
# Node-RED input data from the Emotiv headset

# Acceleration data
@app.route('/emotiv/acceleration/x', methods=['POST'])
def accelerationX():
    request_data = request.get_json()
    accX = request_data['accX']

    return 'Acceleration X data received', 200

@app.route('/emotiv/acceleration/y', methods=['POST'])
def accelerationY():
    global tilt

    request_data = request.get_json()
    accY = request_data['accY']
    #print(f'Acceleration Y data received: {accY}')

    # Determine tilt direction to control both arms, the left or right one
    tilt = "both_arm"
    if accY < 7500:
        tilt = "left_arm"
    elif accY > 8500:
        tilt = "right_arm"
    #print(f'Tilt direction set to: {tilt}')

    return f'Acceleration Y data received, tilt direction set to: {tilt}', 200

@app.route('/emotiv/acceleration/z', methods=['POST'])
def accelerationZ():
    request_data = request.get_json()
    accZ = request_data['accZ']

    return 'Acceleration Z data received', 200

# Prediction Route
model = load('trained_model_logistic.joblib')
scaler = load('scaler_logistic.joblib')

@app.route('/predict', methods=['POST'])
def predict():
    data = request.json
    # Extract frequency band powers from the JSON request
    features = np.array([data['alpha_0'], data['alpha_1'], data['alpha_2'], data['alpha_3'], data['alpha_4'], data['alpha_5'], data['alpha_6'], data['alpha_7'], data['alpha_8'], data['alpha_9'], data['alpha_10'], data['alpha_11'], data['alpha_12'], data['alpha_13'], 
                     data['beta_low_0'], data['beta_low_1'], data['beta_low_2'], data['beta_low_3'], data['beta_low_4'], data['beta_low_5'], data['beta_low_6'], data['beta_low_7'], data['beta_low_8'], data['beta_low_9'], data['beta_low_10'], data['beta_low_11'], data['beta_low_12'], data['beta_low_13'], 
                     data['beta_high_0'], data['beta_high_1'], data['beta_high_2'], data['beta_high_3'], data['beta_high_4'], data['beta_high_5'], data['beta_high_6'], data['beta_high_7'], data['beta_high_8'], data['beta_high_9'], data['beta_high_10'], data['beta_high_11'], data['beta_high_12'], data['beta_high_13'], 
                     data['gamma_0'], data['gamma_1'], data['gamma_2'], data['gamma_3'], data['gamma_4'], data['gamma_5'], data['gamma_6'], data['gamma_7'], data['gamma_8'], data['gamma_9'], data['gamma_10'], data['gamma_11'], data['gamma_12'], data['gamma_13'], 
                     data['theta_0'], data['theta_1'], data['theta_2'], data['theta_3'], data['theta_4'], data['theta_5'], data['theta_6'], data['theta_7'], data['theta_8'], data['theta_9'], data['theta_10'], data['theta_11'], data['theta_12'], data['theta_13']]).reshape(1, -1)
    features_scaled = scaler.transform(features)

    # Predict using the loaded model
    prediction = model.predict(features_scaled)

    # Map prediction to control actions
    if prediction == 'neutral':
        handle_neutral_state()
    elif prediction == 'open':
        handle_open_gripper()
        #increment_gripper()
    elif prediction == 'close':
        handle_close_gripper()
        #decrement_gripper()

    # Return the prediction as JSON
    return jsonify({'prediction': int(prediction[0])})

# Starts the Flask web server 
# on all available IP addresses (0.0.0.0) and port 5014
if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5014)
