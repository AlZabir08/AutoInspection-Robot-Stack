import sys
import os
import math
import random
import csv
import time
from controller import Robot

controller_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.abspath(os.path.join(controller_dir, '..', 'sweep_planner')))
sys.path.append(os.path.abspath(os.path.join(controller_dir, '..', 'path_follower')))

from sweep_planner import planning
from path_follower import compute_motor_speeds

robot = Robot()
TIMESTEP = int(robot.getBasicTimeStep())

# CSV logging setup
log_filename = os.path.join(controller_dir, 'robot_log.csv')
log_file = open(log_filename, mode='w', newline='')
csv_writer = csv.writer(log_file)
csv_writer.writerow([
    'Timestamp', 'Pos_X', 'Pos_Y', 'Target_X', 'Target_Y',
    'IR_Detected', 'Vision_Detected', 'High_Threat',
    'Blocked_WP', 'Current_Waypoint_Index'
])
start_time = time.time()

# Motors
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

# Sensors
gps = robot.getDevice('gps')
gps.enable(TIMESTEP)
imu = robot.getDevice('inertial unit')
imu.enable(TIMESTEP)
camera = robot.getDevice('camera')
camera.enable(TIMESTEP)
camera.recognitionEnable(TIMESTEP)

ir_sensor_names = ['ps0', 'ps1', 'ps2', 'ps5', 'ps6', 'ps7']
ir_sensors = [robot.getDevice(name) for name in ir_sensor_names]
for sensor in ir_sensors:
    sensor.enable(TIMESTEP)

for _ in range(10):
    robot.step(TIMESTEP)

# Sweep Path
ox = [0.0, 1, 1, 0.0, 0.0]
oy = [0.0, 0.0, 1, 1, 0.0]
resolution = 0.1
px, py = planning(ox, oy, resolution)
waypoints = list(zip(px, py))
print("Sweep path ready with", len(waypoints), "waypoints.")

# Variables
MAX_SPEED = 3.0
OBSTACLE_THRESHOLD = 80
current_wp_index = 0
blocked = set()

def is_close(pos):
    return abs(pos[0]) < 0.1 and pos[2] < 0.5

def is_dynamic(obj):
    return True  # Simplified — treat all objects as dynamic for now

def replan_from(current_position):
    global current_wp_index, waypoints
    min_dist = float('inf')
    best_index = 0
    for i, wp in enumerate(waypoints):
        if wp in blocked:
            continue
        dist = math.hypot(current_position[0] - wp[0], current_position[1] - wp[1])
        if dist < min_dist:
            min_dist = dist
            best_index = i
    current_wp_index = best_index

while robot.step(TIMESTEP) != -1:
    position = gps.getValues()
    yaw = imu.getRollPitchYaw()[2]

    if current_wp_index >= len(waypoints):
        print("Sweep complete.")
        left_motor.setVelocity(0)
        right_motor.setVelocity(0)
        break

    target = waypoints[current_wp_index]
    dx = target[0] - position[0]
    dy = target[1] - position[1]
    dist = math.hypot(dx, dy)

    if dist < 0.05:
        print("Reached", target)
        current_wp_index += 1
        continue

    ir_values = [s.getValue() for s in ir_sensors]
    ir_detected = any(v > OBSTACLE_THRESHOLD for v in ir_values)

    num_objects = camera.getRecognitionNumberOfObjects()
    objects = camera.getRecognitionObjects()

    vision_detected = False
    high_threat = False
    for obj in objects:
        if is_close(obj.getPosition()):
            vision_detected = True
            if is_dynamic(obj):
                high_threat = True
                break

    if ir_detected and high_threat:
        blocked_cell = waypoints[current_wp_index]
        if blocked_cell not in blocked:
            print(f"Critical obstacle near {blocked_cell}. Blocking and replanning.")
            blocked.add(blocked_cell)
        direction = random.choice([-1, 1])
        left_motor.setVelocity(direction * MAX_SPEED * 0.5)
        right_motor.setVelocity(-direction * MAX_SPEED * 0.5)
        for _ in range(10): robot.step(TIMESTEP)
        replan_from(position)

    elif ir_detected or vision_detected:
        print("Obstacle ahead. Slowing down.")
        left_motor.setVelocity(MAX_SPEED * 0.3)
        right_motor.setVelocity(MAX_SPEED * 0.3)

    else:
        left_speed, right_speed = compute_motor_speeds(position, yaw, target, max_speed=MAX_SPEED)
        left_speed = max(-MAX_SPEED, min(left_speed, MAX_SPEED))
        right_speed = max(-MAX_SPEED, min(right_speed, MAX_SPEED))
        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)

    # === CSV Logging ===
    timestamp = round(time.time() - start_time, 2)
    csv_writer.writerow([
        timestamp,
        round(position[0], 3), round(position[1], 3),
        round(target[0], 3), round(target[1], 3),
        int(ir_detected),
        int(vision_detected),
        int(high_threat),
        int(waypoints[current_wp_index] in blocked if current_wp_index < len(waypoints) else 0),
        current_wp_index
    ])

# Close CSV file on shutdown
log_file.close()
