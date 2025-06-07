#!/usr/bin/env python3
import time
import math
import threading
import socket
from ev3dev2.motor import LargeMotor, MediumMotor, SpeedPercent, OUTPUT_C, OUTPUT_B, OUTPUT_A,OUTPUT_D
from ev3dev2.sensor.lego import UltrasonicSensor, GyroSensor, ColorSensor
from ev3dev2.sensor import INPUT_2, INPUT_3, INPUT_1, INPUT_4
import queue

###############################################################################
# Hardware Setup
###############################################################################
motor_left = LargeMotor(OUTPUT_C)
motor_right = LargeMotor(OUTPUT_B)
motor_head = MediumMotor(OUTPUT_D)

ultra_front = UltrasonicSensor(INPUT_2)
ultra_back  = UltrasonicSensor(INPUT_3)
color_sensor = ColorSensor(INPUT_1)
gyro_sensor  = GyroSensor(INPUT_4)

color_sensor.mode = "COL-COLOR"
gyro_sensor.mode  = "GYRO-RATE"
time.sleep(1)
gyro_sensor.mode  = "GYRO-ANG"
time.sleep(1)

motor_left.reset()
motor_right.reset()

WHEEL_DIAMETER_MM = 56.0
WHEEL_CIRCUM_MM   = math.pi * WHEEL_DIAMETER_MM
WHEELBASE_MM      = 120.0

robot_x = 0.0
robot_y = 0.0
robot_heading = 0.0
current_left_speed  = 0
current_right_speed = 0

force_stop   = False
stop_move_to = False

SCAN_ANGLES = [-45, -30, -15, 0, 15, 30, 45]
occupancy_map = {}

lock = threading.Lock()

DANGER_ZONE = 8.0

# State machine modes
robot_mode    = "DRIVING"  # or "STOPPED"
stop_end_time = 0

# A base driving speed used by combined commands.
BASE_SPEED = 40

###############################################################################
# Globals for pending SIGN_ACTION commands (for combination)
###############################################################################
pending_sign_action = None       # Will hold a list of actions (strings)
pending_sign_action_timer = None # A Timer object
pending_lock = threading.Lock()  # To protect the above globals

###############################################################################
# Action Queue for Asynchronous Commands
###############################################################################
actions_queue = queue.Queue()

###############################################################################
# Safety Watchdog Globals
###############################################################################
# Update this variable each time a command from the vision model is received.
last_model_command_time = time.time()
MODEL_COMMAND_TIMEOUT = 3.0  # seconds timeout to assume "no model detection"

# Safety sensor thresholds (adjust as needed)
SAFE_FRONT_DISTANCE = 25    # centimeters
SAFE_BACK_DISTANCE  = 25    # centimeters
# For example, if the color sensor sees red, we want to stop.
RED_COLOR = ColorSensor.COLOR_RED

###############################################################################
# Motor Helpers and Soft Stop
###############################################################################
def soft_stop():
    """
    Gradually decelerate the robot instead of an abrupt stop.
    Adjust 'steps' and 'delay' as needed.
    """
    global current_left_speed, current_right_speed
    steps = 5         # Number of deceleration steps
    delay = 0.1       # Seconds between each step

    # Capture current speeds
    initial_left = current_left_speed
    initial_right = current_right_speed

    # Gradually reduce speed
    for step in range(steps, 0, -1):
        new_left = int(initial_left * step / steps)
        new_right = int(initial_right * step / steps)
        motor_left.on(SpeedPercent(new_left))
        motor_right.on(SpeedPercent(new_right))
        time.sleep(delay)
    # Finally, ensure the motors are turned off
    motor_left.off()
    motor_right.off()
    current_left_speed = 0
    current_right_speed = 0

def actions_runner():
    """
    Thread that pulls motor actions (turn, stop, drive, etc.)
    from actions_queue and executes them.
    """
    global current_left_speed, current_right_speed
    while True:
        action = actions_queue.get()
        if action is None:
            break

        a_type = action["type"]

        if a_type == "TURN":
            direction = action.get("direction", "LEFT")
            angle_val = action.get("angle", 30)
            turn_by_angle(direction, angle_val)

        elif a_type == "STOP":
            # Instead of an immediate motor off, perform a gradual deceleration.
            soft_stop()

        elif a_type == "DRIVE":
            l_spd = action.get("l_spd", 0)
            r_spd = action.get("r_spd", 0)
            current_left_speed = l_spd
            current_right_speed = r_spd
            motor_left.on(SpeedPercent(l_spd))
            motor_right.on(SpeedPercent(r_spd))

        actions_queue.task_done()

###############################################################################
# Safe Sensor Reading
###############################################################################
def safe_gyro_angle():
    global robot_heading
    try:
        return gyro_sensor.angle
    except:
        return robot_heading

def get_safe_distance(sensor):
    try:
        return sensor.distance_centimeters
    except:
        return 999

###############################################################################
# Worker Threads
###############################################################################
def ultrasonic_scanner():
    while True:
        for angle in SCAN_ANGLES:
            motor_head.on_for_degrees(SpeedPercent(10), -angle, block=True)
            time.sleep(0.2)
            dist = get_safe_distance(ultra_front)
            with lock:
                occupancy_map[angle] = dist
        motor_head.on_for_degrees(SpeedPercent(10), -sum(SCAN_ANGLES), block=True)
        time.sleep(0.2)

def position_tracker():
    global robot_x, robot_y, robot_heading
    prev_left  = motor_left.position
    prev_right = motor_right.position
    while True:
        time.sleep(0.1)
        with lock:
            curr_left  = motor_left.position
            curr_right = motor_right.position
            delta_left  = curr_left - prev_left
            delta_right = curr_right - prev_right
            prev_left   = curr_left
            prev_right  = curr_right

            dist_left  = (delta_left  / 360.0) * WHEEL_CIRCUM_MM
            dist_right = (delta_right / 360.0) * WHEEL_CIRCUM_MM
            robot_heading = safe_gyro_angle()
            dist_mean = (dist_left + dist_right) / 2.0
            robot_x += dist_mean * math.cos(math.radians(robot_heading))
            robot_y += dist_mean * math.sin(math.radians(robot_heading))

def collision_watcher():
    global force_stop
    while True:
        time.sleep(0.2)
        f_dist = get_safe_distance(ultra_front)
        b_dist = get_safe_distance(ultra_back)
        with lock:
            if occupancy_map:
                min_scan = min(occupancy_map.values())
            else:
                min_scan = 999
        if (f_dist < DANGER_ZONE) or (b_dist < DANGER_ZONE) or (min_scan < DANGER_ZONE):
            force_stop = True
            motor_left.off()
            motor_right.off()
            continue
        else:
            force_stop = False

###############################################################################
# Motor Helpers
###############################################################################
def turn_by_angle(direction, angle=30):
    start_angle = safe_gyro_angle()
    if direction.upper() == "LEFT":
        target_angle = start_angle - angle
        left_speed   = 10
        right_speed  = 30
    else:
        target_angle = start_angle + angle
        left_speed   = 30
        right_speed  = 10

    motor_left.on(SpeedPercent(left_speed))
    motor_right.on(SpeedPercent(right_speed))
    while True:
        current_angle = safe_gyro_angle()
        if direction.upper() == "LEFT" and current_angle <= target_angle:
            break
        if direction.upper() == "RIGHT" and current_angle >= target_angle:
            break
        time.sleep(0.01)
    motor_left.off()
    motor_right.off()

def update_robot_state():
    global robot_mode, stop_end_time
    if robot_mode == "STOPPED":
        if time.time() >= stop_end_time:
            robot_mode = "DRIVING"

###############################################################################
# Process Pending SIGN_ACTION Commands
###############################################################################
def process_pending_sign_action():
    global pending_sign_action, pending_sign_action_timer, robot_mode, stop_end_time, BASE_SPEED
    with pending_lock:
        actions = [a.upper() for a in pending_sign_action] if pending_sign_action is not None else []
        pending_sign_action = None
        pending_sign_action_timer = None
    if "STOP" in actions:
        if "MAND_LEFT" in actions:
            robot_mode = "STOPPED"
            stop_end_time = time.time() + 3.0
            actions_queue.put({"type": "STOP"})
            time.sleep(0.5)
            actions_queue.put({"type": "DRIVE", "l_spd": 10, "r_spd": 25})
            time.sleep(4.0)
            actions_queue.put({"type": "DRIVE", "l_spd": BASE_SPEED, "r_spd": BASE_SPEED})
            print("[EV3] Processed combined SIGN_ACTION: STOP MAND_LEFT")
            return
        elif "MAND_RIGHT" in actions:
            robot_mode = "STOPPED"
            stop_end_time = time.time() + 3.0
            actions_queue.put({"type": "STOP"})
            time.sleep(0.5)
            actions_queue.put({"type": "DRIVE", "l_spd": 25, "r_spd": 10})
            time.sleep(4.0)
            actions_queue.put({"type": "DRIVE", "l_spd": BASE_SPEED, "r_spd": BASE_SPEED})
            print("[EV3] Processed combined SIGN_ACTION: STOP MAND_RIGHT")
            return
        elif any(a.startswith("SPEED_") for a in actions):
            speed_val = None
            for a in actions:
                if a.startswith("SPEED_"):
                    try:
                        speed_val = int(a.split("_")[1])
                        break
                    except:
                        continue
            if speed_val is None:
                speed_val = BASE_SPEED
            robot_mode = "STOPPED"
            stop_end_time = time.time() + 3.0
            actions_queue.put({"type": "STOP"})
            time.sleep(0.5)
            actions_queue.put({"type": "DRIVE", "l_spd": 25, "r_spd": 10})
            time.sleep(4.0)
            actions_queue.put({"type": "DRIVE", "l_spd": speed_val, "r_spd": speed_val})
            print("[EV3] Processed combined SIGN_ACTION: STOP SPEED_{}".format(speed_val))
            return
        else:
            robot_mode = "STOPPED"
            stop_end_time = time.time() + 3.0
            actions_queue.put({"type": "STOP"})
            print("[EV3] Processed SIGN_ACTION: STOP")
            return
    else:
        if not actions:
            return
        action = actions[0]
        if action == "SLOW":
            actions_queue.put({"type": "DRIVE", "l_spd": 10, "r_spd": 10})
            print("[EV3] Processed SIGN_ACTION: SLOW")
        elif action == "MAND_LEFT":
            actions_queue.put({"type": "TURN", "direction": "LEFT", "angle": 30})
            print("[EV3] Processed SIGN_ACTION: MAND_LEFT")
        elif action == "MAND_RIGHT":
            actions_queue.put({"type": "TURN", "direction": "RIGHT", "angle": 30})
            print("[EV3] Processed SIGN_ACTION: MAND_RIGHT")
        elif action == "MAND_STRAIGHT":
            print("[EV3] Processed SIGN_ACTION: MAND_STRAIGHT (no action)")
        elif action.startswith("SPEED_"):
            try:
                speed_val = int(action.split("_")[1])
                actions_queue.put({"type": "DRIVE", "l_spd": speed_val, "r_spd": speed_val})
                print("[EV3] Processed SIGN_ACTION: SPEED_{}".format(speed_val))
            except:
                print("[EV3] Error processing SIGN_ACTION: Invalid SPEED value")
        else:
            print("[EV3] Unknown SIGN_ACTION: {}".format(action))

###############################################################################
# Safety Monitor Thread (Sensor-Based Fallback)
###############################################################################
def safety_monitor():
    """
    This thread checks if the vision/model commands have not been received for a set timeout
    and also checks critical sensor readings. If any dangerous condition is detected,
    it posts a STOP command (or other safety maneuver) to the actions queue.
    """
    global last_model_command_time
    while True:
        current_time = time.time()
        # If no model command has been received recently, trigger safety fallback.
        if (current_time - last_model_command_time) > MODEL_COMMAND_TIMEOUT:
            # Read sensor values directly
            front_dist = get_safe_distance(ultra_front)
            back_dist = get_safe_distance(ultra_back)
            current_color = color_sensor.color
            # Debug logging:
            # print("Safety Monitor: Front {:.1f}, Back {:.1f}, Color {}".format(front_dist, back_dist, current_color))
            if front_dist < SAFE_FRONT_DISTANCE or back_dist < SAFE_BACK_DISTANCE or current_color == RED_COLOR:
                print("[Safety] Dangerous condition detected! Overriding with STOP.")
                actions_queue.put({"type": "STOP"})
        time.sleep(0.5)

###############################################################################
# Command Handler
###############################################################################
def handle_command(cmd):
    global force_stop, stop_move_to, robot_mode, stop_end_time, pending_sign_action, pending_sign_action_timer, last_model_command_time
    # Update the last model command time every time we process a command.
    last_model_command_time = time.time()
    
    cmd = cmd.strip()
    parts = cmd.split()
    if not parts:
        return "ERR Empty command"

    main_cmd = parts[0].upper()

    update_robot_state()

    if force_stop and main_cmd in ["DRIVE", "MOVE", "TURN"] and main_cmd != "STOP":
        return "FORCE_STOP collision override"

    if robot_mode == "STOPPED" and main_cmd in ["DRIVE", "TURN"]:
        return "IGNORED STOPPED mode"

    try:
        if main_cmd == "DRIVE":
            if len(parts) != 3:
                return "ERR Usage: DRIVE <L> <R>"
            l_spd = int(parts[1])
            r_spd = int(parts[2])
            actions_queue.put({"type": "DRIVE", "l_spd": l_spd, "r_spd": r_spd})
            return "OK DRIVE {} {} (scheduled)".format(l_spd, r_spd)

        elif main_cmd == "TURN":
            direction = "LEFT"
            angle_val = 30
            if len(parts) >= 2:
                direction = parts[1].upper()
            if len(parts) == 3:
                angle_val = int(parts[2])
            if direction not in ["LEFT", "RIGHT"]:
                return "ERR Unknown TURN direction"
            actions_queue.put({"type": "TURN", "direction": direction, "angle": angle_val})
            return "OK TURN {} {} (scheduled)".format(direction, angle_val)

        elif main_cmd == "STOP":
            stop_move_to = True
            actions_queue.put({"type": "STOP"})
            return "OK STOP scheduled"

        elif main_cmd == "SIGN_ACTION":
            if len(parts) >= 2:
                with pending_lock:
                    if pending_sign_action is None:
                        pending_sign_action = parts[1:]
                        pending_sign_action = [a.upper() for a in pending_sign_action]
                        pending_sign_action_timer = threading.Timer(0.3, process_pending_sign_action)
                        pending_sign_action_timer.start()
                        return "OK SIGN_ACTION pending"
                    else:
                        new_actions = [a.upper() for a in parts[1:]]
                        pending_sign_action.extend(new_actions)
                        if pending_sign_action_timer is not None:
                            pending_sign_action_timer.cancel()
                        pending_sign_action_timer = threading.Timer(0.3, process_pending_sign_action)
                        pending_sign_action_timer.start()
                        return "OK SIGN_ACTION updated pending"
            else:
                return "ERR Usage: SIGN_ACTION <STOP/SLOW/...>"

        elif main_cmd == "READ_SENSORS":
            f_dist = get_safe_distance(ultra_front)
            b_dist = get_safe_distance(ultra_back)
            c_id = color_sensor.color
            g_ang = safe_gyro_angle()
            return "SENSORS {:.1f} {:.1f} {} {:.1f}".format(f_dist, b_dist, c_id, g_ang)

        elif main_cmd == "GET_STATE":
            with lock:
                return "STATE x={:.1f} y={:.1f} heading={:.1f} L={} R={} mode={}".format(
                    robot_x, robot_y, robot_heading, current_left_speed, current_right_speed, robot_mode
                )

        elif main_cmd == "GET_OCCUPANCY":
            with lock:
                return "OCCUPANCY {}".format(occupancy_map)

        else:
            return "ERR Unknown command"

    except Exception as e:
        actions_queue.put({"type": "STOP"})
        return "ERR {}".format(e)

###############################################################################
# Main Server Loop
###############################################################################
def main():
    # Start the various worker threads.
    threading.Thread(target=ultrasonic_scanner, daemon=True).start()
    threading.Thread(target=position_tracker, daemon=True).start()
    threading.Thread(target=collision_watcher, daemon=True).start()
    threading.Thread(target=actions_runner, daemon=True).start()
    # Start the safety monitor thread.
    threading.Thread(target=safety_monitor, daemon=True).start()

    HOST = ""
    PORT = 5000
    print("[EV3] Server starting on port {}".format(PORT))
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_sock:
            server_sock.settimeout(20)
            server_sock.bind((HOST, PORT))
            server_sock.listen(1)
            print("[EV3] Waiting for connection...")
            conn, addr = server_sock.accept()
            print("[EV3] Connected by {}".format(addr))
            with conn:
                while True:
                    data = conn.recv(1024)
                    if not data:
                        break

                    update_robot_state()
                    cmd_str = data.decode("utf-8")
                    print("[EV3] Received command: {}".format(cmd_str))

                    response = handle_command(cmd_str)
                    conn.sendall(response.encode("utf-8"))
    except socket.timeout:
        print("[EV3] Connection timed out after 20 seconds. Exiting.")
    except Exception as e:
        print("[EV3] Exception in main server loop: {}".format(e))
    finally:
        motor_left.off()
        motor_right.off()
        actions_queue.put(None)
        print("[EV3] Connection closed. Exiting.")

if __name__ == "__main__":
    main()
