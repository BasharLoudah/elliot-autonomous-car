#!/usr/bin/env python3
import argparse
import logging
import threading
import time
import json
from evdev import InputDevice, categorize, ecodes
from ev3dev2.motor import LargeMotor, OUTPUT_B, OUTPUT_C, SpeedPercent
from ev3dev2.sensor.lego import GyroSensor
from ev3dev2.sound import Sound

logging.basicConfig(level=logging.DEBUG, format='[%(asctime)s] %(levelname)s: %(message)s')

# Create a global speaker instance.
speaker = Sound()

def get_robot_data():
    paths = ['../robot.json', 'robot.json']
    for p in paths:
        try:
            with open(p, 'r') as f:
                data = json.load(f)
                logging.info("Loaded robot configuration from {}".format(p))
                return data
        except FileNotFoundError:
            continue
    raise FileNotFoundError("Could not find robot.json in expected paths.")

class ControllerState:
    def __init__(self):
        self.lock = threading.Lock()
        self.forward = 0.0
        self.turn = 0.0
        self.handbrake = False
    def update(self, forward, turn, handbrake):
        with self.lock:
            self.forward = forward
            self.turn = turn
            self.handbrake = handbrake
    def get_all(self):
        with self.lock:
            return (self.forward, self.turn, self.handbrake)

class GameControllerHandler(threading.Thread):
    def __init__(self, device_path, state: ControllerState, deadzone=5, horn_file=None, mapping=None):
        threading.Thread.__init__(self)
        self.device_path = device_path
        self.controller_state = state
        self.deadzone = deadzone
        self.running = True
        self.horn_file = horn_file
        try:
            self.device = InputDevice(self.device_path)
            logging.info("Controller connected on {} (Device name: {})".format(self.device_path, self.device.name))
        except Exception as e:
            logging.error("Failed to open device {}: {}".format(self.device_path, e))
            raise

        # Default mapping: these correspond to the axes and buttons.
        # Note: "right_bumper" (RB) is added with a default value of "BTN_TR"
        default_mapping = {
            "right_trigger": ecodes.ABS_RZ,
            "left_trigger": ecodes.ABS_Z,
            "turn": ecodes.ABS_X,
            "handbrake": "BTN_TL",
            "horn": "BTN_A",
            "right_bumper": "BTN_TR"
        }
        if mapping is not None:
            # Allow mapping values as strings (e.g., "ABS_RZ") and convert them.
            for key in ["right_trigger", "left_trigger", "turn"]:
                val = mapping.get(key)
                if isinstance(val, str):
                    mapping[key] = getattr(ecodes, val)
            self.mapping = mapping
        else:
            # Auto-detect if the controller is an EvoFox Elite X.
            if "EvoFox Elite X" in self.device.name:
                logging.info("Detected EvoFox Elite X controller. Applying custom mapping.")
                self.mapping = {
                    "right_trigger": getattr(ecodes, "ABS_Z"),   # EvoFox: right trigger on ABS_Z
                    "left_trigger": getattr(ecodes, "ABS_RZ"),     # EvoFox: left trigger on ABS_RZ
                    "turn": getattr(ecodes, "ABS_X"),
                    "handbrake": "BTN_TL",
                    "horn": "BTN_A",
                    "right_bumper": "BTN_TR"  # Add right bumper mapping
                }
            else:
                self.mapping = default_mapping

        # Calibrate analog axes.
        self.abs_calibration = {}
        for key in ["right_trigger", "left_trigger", "turn"]:
            code = self.mapping[key]
            try:
                absinfo = self.device.absinfo(code)
            except AttributeError:
                abs_caps = self.device.capabilities().get(ecodes.EV_ABS, [])
                absinfo = None
                for cap in abs_caps:
                    if isinstance(cap, tuple) and cap[0] == code:
                        absinfo = cap[1]
                        break
                if absinfo is None:
                    logging.warning("No calibration info for axis {}: using default range 0-255".format(code))
                    class DefaultAbsInfo:
                        min = 0
                        max = 255
                    absinfo = DefaultAbsInfo()
            self.abs_calibration[code] = {
                'min': absinfo.min,
                'max': absinfo.max,
                'mid': (absinfo.min + absinfo.max) / 2.0,
                'range': (absinfo.max - absinfo.min) / 2.0
            }
            logging.debug("Calibration for axis {}: {}".format(code, self.abs_calibration[code]))
        # Initialize current values.
        self.rt_value = 0.0
        self.lt_value = 0.0
        self.turn_value = 0.0

    def normalize_axis(self, code, value):
        if code not in self.abs_calibration:
            return 0.0
        cal = self.abs_calibration[code]
        deviation = value - cal['mid']
        percent = (deviation / cal['range']) * 100 if cal['range'] != 0 else 0.0
        if abs(percent) < self.deadzone:
            percent = 0.0
        return max(-100, min(100, percent))

    def normalize_trigger(self, code, value):
        if code not in self.abs_calibration:
            return 0.0
        cal = self.abs_calibration[code]
        trigger_val = ((value - cal['min']) / (cal['max'] - cal['min'])) * 100
        if trigger_val < self.deadzone:
            trigger_val = 0.0
        return max(0, min(100, trigger_val))

    def run(self):
        logging.info("Starting controller event loop")
        for event in self.device.read_loop():
            if not self.running:
                break
            # If running in normal mode, process the events to control the robot.
            if event.type == ecodes.EV_ABS:
                if event.code == self.mapping["right_trigger"]:
                    self.rt_value = self.normalize_trigger(event.code, event.value)
                elif event.code == self.mapping["left_trigger"]:
                    self.lt_value = self.normalize_trigger(event.code, event.value)
                elif event.code == self.mapping["turn"]:
                    self.turn_value = self.normalize_axis(event.code, event.value)
                # Calculate forward speed (using squared response for smoother control).
                throttle = (self.rt_value/100.0)**2 * 100
                brake = (self.lt_value/100.0)**2 * 100
                forward = throttle - brake
                with self.controller_state.lock:
                    self.controller_state.forward = forward
                    self.controller_state.turn = self.turn_value
            elif event.type == ecodes.EV_KEY:
                key_event = categorize(event)
                key = key_event.keycode
                state_val = key_event.keystate
                # Handbrake button
                if (isinstance(key, list) and self.mapping["handbrake"] in key) or key == self.mapping["handbrake"]:
                    with self.controller_state.lock:
                        self.controller_state.handbrake = (state_val == 1)
                # Horn button
                elif ((isinstance(key, list) and self.mapping["horn"] in key) or key == self.mapping["horn"]) and state_val == 1:
                    logging.info("Horn button pressed, playing horn sound from {}".format(self.horn_file))
                    if self.horn_file:
                        speaker.set_volume(100)
                        speaker.play_file(self.horn_file)
                # Right bumper button
                elif (isinstance(key, list) and self.mapping.get("right_bumper") in key) or key == self.mapping.get("right_bumper"):
                    logging.info("Right bumper pressed")
                    # You can add an action here if desired.
                else:
                    logging.debug("Button event: {} State: {}".format(key, state_val))
        logging.info("Controller event loop terminated")

    def stop(self):
        self.running = False

class DriveController(threading.Thread):
    def __init__(self, state: ControllerState, update_interval=0.05, ramp_rate=2):
        threading.Thread.__init__(self)
        self.state = state
        self.update_interval = update_interval
        self.ramp_rate = ramp_rate
        self.running = True
        self.motor_left = LargeMotor(OUTPUT_C)
        self.motor_right = LargeMotor(OUTPUT_B)
        self.current_left_speed = 0.0
        self.current_right_speed = 0.0
        try:
            self.gyro = GyroSensor('in1')
            self.gyro.mode = 'GYRO-ANG'
        except Exception as e:
            logging.error("Gyro sensor error: {}".format(e))
            self.gyro = None

    def differential_drive(self, forward, turn):
        target_left = forward + turn
        target_right = forward - turn
        target_left = max(-100, min(100, target_left))
        target_right = max(-100, min(100, target_right))
        return target_left, target_right

    def ramp_speed(self, current, target):
        if current < target:
            current = min(current + self.ramp_rate, target)
        elif current > target:
            current = max(current - self.ramp_rate, target)
        return current

    def run(self):
        logging.info("Starting drive controller loop")
        while self.running:
            with self.state.lock:
                forward = self.state.forward
                turn = self.state.turn
                handbrake = self.state.handbrake
            if handbrake:
                target_left, target_right = 0.0, 0.0
            else:
                if self.gyro is not None and abs(turn) < 5:
                    error = -self.gyro.angle
                    Kp = 0.3
                    turn += Kp * error
                target_left, target_right = self.differential_drive(forward, turn)
            self.current_left_speed = self.ramp_speed(self.current_left_speed, target_left)
            self.current_right_speed = self.ramp_speed(self.current_right_speed, target_right)
            logging.debug("Setting motor speeds: left = {:.1f}%, right = {:.1f}%".format(self.current_left_speed, self.current_right_speed))
            self.motor_left.on(SpeedPercent(self.current_left_speed))
            self.motor_right.on(SpeedPercent(self.current_right_speed))
            time.sleep(self.update_interval)
        logging.info("Drive controller loop terminated")
        self.motor_left.off()
        self.motor_right.off()

    def stop(self):
        self.running = False

def input_test(device_path):
    """
    Opens the input device and prints all events.
    Use this to see which events are generated by your controller.
    """
    try:
        device = InputDevice(device_path)
        logging.info("Input Test: Device connected on {} (Name: {})".format(device_path, device.name))
    except Exception as e:
        logging.error("Failed to open device {}: {}".format(device_path, e))
        return

    logging.info("Starting input test. Press Ctrl+C to exit.")
    try:
        for event in device.read_loop():
            logging.info("Event: type={} code={} value={}".format(event.type, event.code, event.value))
    except KeyboardInterrupt:
        logging.info("Input test terminated by user.")

def main():
    parser = argparse.ArgumentParser(description="EV3 Robot Controller")
    parser.add_argument("--input-test", action="store_true",
                        help="Run in input test mode to print raw controller events.")
    args = parser.parse_args()

    # Load robot configuration.
    try:
        robot_config = get_robot_data()
    except Exception as e:
        logging.error("Error loading robot configuration: {}".format(e))
        return

    device_path = robot_config.get('xbox_device_path', '/dev/input/event2')

    # If --input-test is passed, run the input test mode.
    if args.input_test:
        input_test(device_path)
        return

    if robot_config.get('command_source', 'xbox') != 'xbox':
        logging.error("This configuration requires a local controller. Set command_source to 'xbox'.")
        return

    horn_file = robot_config.get('horn_file')
    controller_state = ControllerState()
    threads = []
    deadzone = robot_config.get('deadzone', 5)

    # Optionally, you could add a "controller_mapping" key in your JSON to customize mappings.
    controller_mapping = robot_config.get('controller_mapping')
    controller_handler = GameControllerHandler(device_path, controller_state, deadzone, horn_file=horn_file, mapping=controller_mapping)
    controller_handler.daemon = True
    controller_handler.start()
    threads.append(controller_handler)

    update_interval = robot_config.get('update_interval', 0.05)
    ramp_rate = robot_config.get('ramp_rate', 2)
    drive_controller = DriveController(controller_state, update_interval, ramp_rate)
    drive_controller.daemon = True
    drive_controller.start()
    threads.append(drive_controller)

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        logging.info("KeyboardInterrupt received, stopping threads...")
        for t in threads:
            t.stop()
        for t in threads:
            t.join()
        logging.info("Shutdown complete.")

if __name__ == '__main__':
    main()
