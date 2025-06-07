#!/usr/bin/env python3
import cv2
import time
import socket
import numpy as np
import threading, logging
import torch
from ultralytics import YOLO

#############################################################################
# CONSTANTS & SETTINGS
#############################################################################

EV3_HOST = "192.168.1.106"
EV3_PORT = 5000

# Camera feed
CAMERA_SOURCE = "http://192.168.1.102:8080/video"

# Traffic signs model (YOLO)
MODEL_TRAFFIC_SIGNS_PATH = r"C:\Users\basha\runs\detect\train13\weights\best.pt"
DEVICE = "cuda" if torch.cuda.is_available() else "cpu"

# Runtime settings
BASE_SPEED = 40
SIGN_CONF_THRESHOLD = 0.65

logging.basicConfig(level=logging.INFO, format='[%(asctime)s] %(levelname)s: %(message)s')

#############################################################################
# GLOBALS
#############################################################################

stop_threads = False
dynamic_speed = BASE_SPEED
auto_mode = True

ev3_comm = None

camera_failure_count = 0
CAMERA_FAILURE_THRESHOLD = 100  # Adjust threshold as needed

#############################################################################
# CLASSES
#############################################################################

class EV3Communicator:
    def __init__(self, host, port, timeout=5):
        self.host = host
        self.port = port
        self.timeout = timeout
        self.sock = None
        self.connect()

    def connect(self):
        attempt = 0
        while attempt < 3:
            try:
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.settimeout(self.timeout)
                self.sock.connect((self.host, self.port))
                logging.info("Connected to EV3 at {}:{}".format(self.host, self.port))
                return
            except socket.error as e:
                attempt += 1
                logging.error("EV3 connection error: {}. Attempt {}/3".format(e, attempt))
                time.sleep(2)
        logging.error("Failed to connect to EV3 after 3 attempts. Exiting run.")
        exit(1)

    def send_command(self, cmd):
        if self.sock is None:
            logging.error("No EV3 connection.")
            return "Error: No connection"
        try:
            self.sock.sendall(cmd.encode("utf-8"))
            response = self.sock.recv(1024).decode("utf-8")
            return response
        except socket.error as e:
            logging.error("Error sending command '{}' : {}".format(cmd, e))
            # Retry logic in case of failure
            for _ in range(3):
                try:
                    self.sock.sendall(cmd.encode("utf-8"))
                    response = self.sock.recv(1024).decode("utf-8")
                    return response
                except socket.error as e2:
                    logging.error("Retry failed: {}".format(e2))
                    time.sleep(1)
            if cmd.strip().upper().startswith("DRIVE"):
                try:
                    fallback_cmd = "DRIVE 10 10"
                    logging.info("Sending fallback command '{}'".format(fallback_cmd))
                    self.sock.sendall(fallback_cmd.encode("utf-8"))
                    response = self.sock.recv(1024).decode("utf-8")
                    return response
                except Exception as e2:
                    logging.error("Fallback command failed: {}".format(e2))
            return "ERROR"

    def close(self):
        if self.sock:
            self.sock.close()
            self.sock = None

class CameraCaptureThread(threading.Thread):
    def __init__(self, src):
        super().__init__()
        self.cap = cv2.VideoCapture(src)
        self.running = True
        self.latest_frame = None

    def run(self):
        global stop_threads, camera_failure_count
        while self.running and not stop_threads:
            ret, frame = self.cap.read()
            if ret:
                self.latest_frame = frame
                camera_failure_count = 0
            else:
                logging.warning("Camera read failed.")
                camera_failure_count += 1
                if camera_failure_count >= CAMERA_FAILURE_THRESHOLD:
                    logging.error("Camera read failed continuously; sending STOP command to EV3.")
                    if ev3_comm:
                        ev3_comm.send_command("STOP")
                    camera_failure_count = 0
                time.sleep(0.01)

    def stop(self):
        self.running = False
        self.cap.release()

#############################################################################
# UTILS
#############################################################################

def detect_signs(frame, model, detection_skip=False):
    """
    Runs the YOLO model on the frame if detection_skip is False,
    else returns an empty list.
    """
    if detection_skip or model is None:
        return []
    results_list = model(frame, conf=SIGN_CONF_THRESHOLD, imgsz=640)
    sign_list = []
    for result in results_list:
        boxes = result.boxes
        names = model.names
        for box in boxes:
            cls_id = int(box.cls[0].item())
            conf = float(box.conf[0].item())
            label = names[cls_id]
            if "info_parking" in label.lower():
                if conf < 0.85:
                    continue
            else:
                if conf < SIGN_CONF_THRESHOLD:
                    continue
            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
            sign_list.append((label, x1, y1, x2, y2, conf))
    return sign_list

def draw_sign_bboxes(frame, sign_list):
    for (label, x1, y1, x2, y2, conf) in sign_list:
        cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 255), 2)
        text = "{} ({:.2f})".format(label, conf)
        cv2.putText(frame, text, (x1, max(y1-5, 0)),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 2)

def handle_special_signs(label, ev3_comm, conf):
    """
    Handles one-time sign-based actions (speed changes, mandatory turns, stops, etc.)
    """
    global dynamic_speed
    lbl_lower = label.lower()

    if "red light" in lbl_lower:
        if conf >= 0.12:

            logging.info("Traffic Light: Red => STOP")
            ev3_comm.send_command("STOP")
            time.sleep(5)
        else:
            logging.info("Detected mandatory left sign, but confidence is too low: {:.2f}".format(conf))
        #ev3_comm.send_command("SIGN_ACTION STOP")

    elif "green light" in lbl_lower:
        if conf >= 0.25:

            logging.info("Traffic Light: Green => continuing")
        else:
            logging.info("Detected mandatory left sign, but confidence is too low: {:.2f}".format(conf))

    elif "mand_left" in lbl_lower:
        if conf >= 0.96:
            logging.info("Detected mandatory left => turning left gently")
            old_speed = dynamic_speed
            ev3_comm.send_command("DRIVE 10 25")
            time.sleep(3.5)
            ev3_comm.send_command(f"DRIVE {old_speed} {old_speed}")
        else:
            logging.info("Detected mandatory left sign, but confidence is too low: {:.2f}".format(conf))

    elif "mand_right" in lbl_lower:
        if conf >= 0.60:
            logging.info("Detected mandatory right => turning right gently")
            old_speed = dynamic_speed
            ev3_comm.send_command("DRIVE 25 10")
            time.sleep(3.5)
            ev3_comm.send_command(f"DRIVE {old_speed} {old_speed}")
        else:
            logging.info("Detected mandatory left sign, but confidence is too low: {:.2f}".format(conf))
    elif "mand_straight" in lbl_lower:
        if conf >= 0.89:
            logging.info("Mandatory straight => no special turn. Just keep going.")
        else:
            logging.info("Detected mandatory left sign, but confidence is too low: {:.2f}".format(conf))
    elif "speed limit" in lbl_lower:
        parts = lbl_lower.split()
        if len(parts) >= 3:
            try:
                limit_val = int(parts[2])
                dynamic_speed = limit_val
                ev3_comm.send_command(f"SIGN_ACTION SPEED_{limit_val}")
                logging.info("Set dynamic_speed=%s", limit_val)
            except ValueError:
                pass

    elif "stop" in lbl_lower:
        if conf >= 0.85:
            logging.info("Detected STOP sign => stopping")
            ev3_comm.send_command("SIGN_ACTION STOP")
        else:
            logging.info("Detected mandatory left sign, but confidence is too low: {:.2f}".format(conf))

    elif "slow" in lbl_lower:
        logging.info("Detected SLOW => partial speed")
        ev3_comm.send_command("SIGN_ACTION SLOW")

    elif "info_parking" in lbl_lower:
        if conf > 0.94:
            logging.info("Detected info_parking with high confidence => initiating parking maneuver")
            old_speed = dynamic_speed
            ev3_comm.send_command("DRIVE 5 5")
            time.sleep(1)
            ev3_comm.send_command("DRIVE -15 -15")
            time.sleep(1)
            ev3_comm.send_command("DRIVE 25 10")
            time.sleep(3)
            ev3_comm.send_command("DRIVE 10 10")
            time.sleep(3)            
            ev3_comm.send_command("DRIVE 15 15")
            time.sleep(2)
            ev3_comm.send_command("STOP")
            time.sleep(4)
            ev3_comm.send_command("DRIVE 22 10")
            time.sleep(3.5)
            ev3_comm.send_command("STOP")
            dynamic_speed = old_speed
        else:
            logging.info("Detected info_parking but confidence ({:.2f}) is too low.".format(conf))


    elif any(k in lbl_lower for k in ["danger", "prohibitory", "other"]):
        logging.info("Detected %s => performing safety maneuver", label)
        old_speed = dynamic_speed
        ev3_comm.send_command("STOP")
        time.sleep(1)
        ev3_comm.send_command("DRIVE 10 10")
        time.sleep(2)
        ev3_comm.send_command(f"DRIVE {old_speed} {old_speed}")

#############################################################################
# MAIN
#############################################################################

def main():
    global stop_threads, dynamic_speed, auto_mode, ev3_comm
    cam_thread = None
    try:
        ev3_comm = EV3Communicator(EV3_HOST, EV3_PORT)
        cam_thread = CameraCaptureThread(CAMERA_SOURCE)
        cam_thread.start()

        logging.info("Loading Unified Model (Traffic signs, Mandatory signs, Car, Person, etc.): %s",
                     MODEL_TRAFFIC_SIGNS_PATH)
        unified_model = YOLO(MODEL_TRAFFIC_SIGNS_PATH)
        unified_model.to(DEVICE)

        logging.info("Camera feed started: %s", CAMERA_SOURCE)

        # Main loop
        while True:
            frame = cam_thread.latest_frame
            if frame is None:
                time.sleep(0.01)
                continue

            sign_list = detect_signs(frame, unified_model, detection_skip=False)
            detection_frame = frame.copy()
            draw_sign_bboxes(detection_frame, sign_list)

            # Check for car/person to stop
            labels_lower = [lbl.lower() for (lbl, x1, y1, x2, y2, conf) in sign_list]
            car_person_detected = any(("car" in lbl or "person" in lbl) for lbl in labels_lower)

            if car_person_detected:
                logging.info("Car or Person detected => STOPPING")
                ev3_comm.send_command("STOP")
            else:
                for (label, x1, y1, x2, y2, conf) in sign_list:
                    handle_special_signs(label, ev3_comm, conf)
                # Drive with constant speed
                cmd = f"DRIVE {dynamic_speed} {dynamic_speed}"
                ev3_comm.send_command(cmd)

            cv2.imshow("Camera Feed", detection_frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            time.sleep(0.01)

    except KeyboardInterrupt:
        logging.info("Keyboard interrupt received. Exiting...")
    finally:
        stop_threads = True
        if cam_thread is not None:
            cam_thread.stop()
        if ev3_comm:
            ev3_comm.send_command("STOP")
            ev3_comm.close()
        cv2.destroyAllWindows()
        logging.info("Shutdown complete.")

if __name__ == "__main__":
    main()
