
#!/usr/bin/env python3
from ev3dev2.motor import LargeMotor, OUTPUT_B, OUTPUT_C, OUTPUT_A, SpeedPercent, MoveTank, MediumMotor
from ev3dev2.sensor.lego import UltrasonicSensor, LightSensor, ColorSensor
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3, INPUT_4
from ev3dev2.sound import Sound
from time import sleep
import signal
import sys

# Initialize motors, sensors, and sound
try:
    motor_b = LargeMotor(OUTPUT_B)
    motor_c = LargeMotor(OUTPUT_C)
    motor_a = MediumMotor(OUTPUT_A)  # Motor for rotating the sensor
    ultrasonic_sensor_1 = UltrasonicSensor(INPUT_2)  # Main sensor for obstacle detection
    ultrasonic_sensor_2 = UltrasonicSensor(INPUT_3)  # Secondary sensor for obstacle detection
    light_sensor = LightSensor(INPUT_4)  # Light sensor on Port 4
    color_sensor = ColorSensor(INPUT_1)  # Color sensor on Port 1
    tank_drive = MoveTank(OUTPUT_B, OUTPUT_C)
    sound = Sound()
except Exception as e:
    print("Error initializing hardware: {}".format(e))
    exit(1)

# Constants
SAFE_DISTANCE = 20  # Distance in cm to trigger obstacle avoidance for sensor 1
sec_SAFE_DISTANCE=50
FORWARD_DISTANCE = 20  # Distance in cm to trigger forward motion for sensor 2
MAX_SPEED = 50  # Maximum forward speed
MIN_SPEED = 6  # Minimum speed when near an obstacle
FORWARD_SPEED = 6  # Speed for forward motion when sensor 2 detects an obstacle
LIGHT_THRESHOLD = 10  # Light intensity percentage threshold for triggering backward movement

# Signal handler to stop the program safely
def signal_handler(sig, frame):
    print("Program interrupted. Stopping motors.")
    tank_drive.off()
    sound.speak("Program interrupted.")
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)
def parking():
    print("Finding if there parking here.")
    # Rotate sensor to -90 degrees
    motor_a.on_for_degrees(SpeedPercent(6), -90)
    sleep(1)
    if ultrasonic_sensor_2.distance_centimeters < FORWARD_DISTANCE:
        print("Stopping and turning to avoid obstacle.")
        tank_drive.off()

    # Check distance after rotating sensor to -90
    distance_1 = ultrasonic_sensor_1.distance_centimeters
    if distance_1 > SAFE_DISTANCE:
        print("Path clear at -90 degrees. Turning left.")
        sound.speak("Path clear")
        motor_a.on_for_degrees(SpeedPercent(10), 90)  # Reset sensor to initial state
        tank_drive.on(SpeedPercent(30), SpeedPercent(30))  # Turn left
        sleep(1)  # Adjust duration as needed
        tank_drive.off()
        return
    else:
        print("Path blocked at -90 degrees.")
       # Rotate sensor to 90 degrees
    tank_drive.on(SpeedPercent(-30), SpeedPercent(-30))
    sleep(2.5)
    tank_drive.off()
    
    distance_1 = ultrasonic_sensor_1.distance_centimeters
    if distance_1 > SAFE_DISTANCE:
        print("Path clear at -90 degrees. Turning left.")
        sound.speak("Path clear")
        motor_a.on_for_degrees(SpeedPercent(10), 90)  # Reset sensor to initial state
        tank_drive.on(SpeedPercent(30), SpeedPercent(30))  # Turn left
        sleep(1)  # Adjust duration as needed
        tank_drive.off()
        return
    else:
        print("Path blocked at -90 degrees again.")
        tank_drive.off()
        motor_a.on_for_degrees(SpeedPercent(10), 90)  # Reset sensor to initial state
        print("Turning left before moving forward.")
        tank_drive.on(SpeedPercent(10), SpeedPercent(10))  # forword 
        sleep(3.5)  # Adjust as needed
        tank_drive.off()

        print("Turning left before moving forward.")
        tank_drive.on(SpeedPercent(-10), SpeedPercent(10))  # Turn right
        sleep(2)  # Adjust as needed
        tank_drive.off()

        print("Moving forward slowly.")
        tank_drive.on(SpeedPercent(-15), SpeedPercent(-15))  # Move backword slowly
        sleep(3)  # Adjust as needed
        tank_drive.off()

        print("Turning right after forward motion.")
        tank_drive.on(SpeedPercent(10), SpeedPercent(-10))  # Turn left
        sleep(2)  # Adjust as needed
        tank_drive.off()
                
        tank_drive.on(SpeedPercent(10), SpeedPercent(10))   #stop 
        sleep(0.5)  # Check condition periodically
        tank_drive.off()
        sleep(7)

        #go from parking
        tank_drive.on(SpeedPercent(-10), SpeedPercent(-10))
        sleep(0.7)  # Check condition periodically
        tank_drive.off()

        print("Turning left before moving forward.")
        tank_drive.on(SpeedPercent(-10), SpeedPercent(10))  # Turn right
        sleep(2)  # Adjust as needed
        tank_drive.off()

        print("Moving forward slowly.")
        tank_drive.on(SpeedPercent(15), SpeedPercent(15))  # Move forward slowly
        sleep(5)  # Adjust as needed
        tank_drive.off()

        print("Turning right after forward motion.")
        tank_drive.on(SpeedPercent(10), SpeedPercent(-10))  # Turn left
        sleep(2)  # Adjust as needed
        tank_drive.off()
        return
 
# Function to rotate and check new paths
def find_new_path():
    
     # Rotate sensor to -90 degrees
    motor_a.on_for_degrees(SpeedPercent(6), -90)
    sleep(1)

    # Check distance after rotating sensor to -90
    distance_1 = ultrasonic_sensor_1.distance_centimeters
    distance_2 = ultrasonic_sensor_2.distance_centimeters
    if distance_1 > SAFE_DISTANCE:
        print("Path clear at -90 degrees. Turning left.")
        sound.speak("Path clear")
        motor_a.on_for_degrees(SpeedPercent(10), 90)  # Reset sensor to initial state
        tank_drive.on(SpeedPercent(20), SpeedPercent(-20))  # Turn left
        sleep(2)  # Adjust duration as needed
        tank_drive.off()
        return
    else:
        print("Path blocked at -90 degrees.")
       # Rotate sensor to 90 degrees
    motor_a.on_for_degrees(SpeedPercent(5), 90)
    sleep(1)

    # Check distance after rotating sensor to 90
    distance_1 = ultrasonic_sensor_1.distance_centimeters
    distance_2 = ultrasonic_sensor_2.distance_centimeters
    if distance_1 > sec_SAFE_DISTANCE:
        print("Path clear at 90 degrees. Turning right.")
        sound.speak("Path clear.")
        tank_drive.on(SpeedPercent(30), SpeedPercent(30))
        sleep(4)
        tank_drive.off()
        return
    else:
        print("Path blocked at 90 degrees.")
    # Rotate sensor to 180 degrees
    motor_a.on_for_degrees(SpeedPercent(5), 90)
    sleep(1)

    # Check distance after rotating sensor to 180
    distance_1 = ultrasonic_sensor_1.distance_centimeters
    distance_2 = ultrasonic_sensor_2.distance_centimeters
    if distance_1 > SAFE_DISTANCE:
        print("Path clear at 180 degrees. Turning right.")
        sound.speak("Path clear.")
        motor_a.on_for_degrees(SpeedPercent(10), -90)  # Reset sensor to initial state
        tank_drive.on(SpeedPercent(-30), SpeedPercent(30))  # Turn right
        sleep(1)  # Adjust duration as needed
        tank_drive.off()
        return
    
     
    else:
        print("Path blocked at 90 degrees. Moving backward.")
        sound.speak("Path blocked.")
        motor_a.on_for_degrees(SpeedPercent(10), -90)  # Reset sensor to initial state

        # Move backward to create space
        while True:
            if ultrasonic_sensor_2.distance_centimeters < FORWARD_DISTANCE:
                print("Stopping and turning to avoid obstacle.")
                tank_drive.off()

                print("Turning left before moving forward.")
                tank_drive.on(SpeedPercent(10), SpeedPercent(-10))  # Turn left
                sleep(2)  # Adjust as needed
                tank_drive.off()

                print("Moving forward slowly.")
                tank_drive.on(SpeedPercent(15), SpeedPercent(15))  # Move forward slowly
                sleep(7)  # Adjust as needed
                tank_drive.off()
                print("Turning right after forward motion.")
                tank_drive.on(SpeedPercent(-10), SpeedPercent(10))  # Turn right
                sleep(2)  # Adjust as needed
                tank_drive.off()
                parking()

                return
            tank_drive.on(SpeedPercent(-20), SpeedPercent(-20))
            sleep(0.5)  # Check condition periodically
        tank_drive.off()

# Main program
def main():
    print("Program started. Monitoring sensors...")
    sound.speak("started.")

    while True:
        try:
            # Measure light intensity
            light_intensity = light_sensor.reflected_light_intensity

            if light_intensity < LIGHT_THRESHOLD:
                tank_drive.off()
                while True:
                    if ultrasonic_sensor_2.distance_centimeters < FORWARD_DISTANCE:
                        print("Stopping and turning to avoid obstacle.")
                        print("Turning left before moving forward.")
                        tank_drive.on(SpeedPercent(10), SpeedPercent(-10))  # Turn left
                        sleep(2)  # Adjust as needed
                        tank_drive.off()
                    tank_drive.on(SpeedPercent(-20), SpeedPercent(-20))
                    sleep(0.5)  # Check condition periodically
                tank_drive.off()

            # Measure distances using the ultrasonic sensors
            distance_1 = ultrasonic_sensor_1.distance_centimeters
            distance_2 = ultrasonic_sensor_2.distance_centimeters

            # Measure color sensor value
            color = color_sensor.color

            if color == ColorSensor.COLOR_RED:
                tank_drive.off()
                
                while color_sensor.color == ColorSensor.COLOR_RED:
                    sleep(0.1)  # Wait while red light is detected
                sleep(2)
                tank_drive.on(SpeedPercent(10), SpeedPercent(10))  # Slow speed
               
                if color_sensor.color == ColorSensor.COLOR_YELLOW:
                    tank_drive.on(SpeedPercent(2), SpeedPercent(2))  # Slow speed
 

            elif color == ColorSensor.COLOR_YELLOW:
                print("Yellow light detected. Slowing down.")

                tank_drive.on(SpeedPercent(10), SpeedPercent(10))  # Slow speed
                while color_sensor.color == ColorSensor.COLOR_YELLOW:
                    sleep(0.1)
                sleep(2) 

            elif color == ColorSensor.COLOR_GREEN:
                print("Green light detected. Moving normally.")
                sound.speak("Green")
                tank_drive.on(SpeedPercent(40), SpeedPercent(40))  # Slow speed
                while color_sensor.color == ColorSensor.COLOR_GREEN:
                    sleep(0.1)

            print("Sensor 1 Distance: {} cm, Sensor 2 Distance: {} cm, Light Intensity: {}%".format(
                distance_1, distance_2, light_intensity))

            # Obstacle avoidance logic for sensor 1
            if distance_1 > SAFE_DISTANCE:
                speed = min(MAX_SPEED, max(MIN_SPEED, (distance_1 - SAFE_DISTANCE) * 2))
                print("Sensor 1: Moving forward with speed: {}%".format(speed))
                tank_drive.on(SpeedPercent(speed), SpeedPercent(speed))
            else:
                print("Sensor 1 detected an obstacle! Moving backward.")
                sound.speak("Sensor one")
                tank_drive.off()

                if distance_1 < SAFE_DISTANCE:
                    find_new_path()

                # Stop the robot
                tank_drive.off()

                # Find a new path
                #parking()

            # Check if distance_2 condition occurs during this process
            if distance_2 < FORWARD_DISTANCE:
                print("Stopping and turning to avoid obstacle.")
                   # Check if distance_1 > SAFE_DISTANCE within the loop
                if distance_1 < SAFE_DISTANCE:
                    print("Safe distance detected by sensor one. Stopping to reevaluate.")
                    sound.speak("distance")
                    tank_drive.off()  # Stop the robot
                    find_new_path()  
                     # Call the function to find a new path
                sound.speak("Stopping")
                tank_drive.off()  # Stop the robot
                #find_new_path()

                # Force the robot to stop and wait until the obstacle is cleared
                while distance_2 < FORWARD_DISTANCE:
                    print("Obstacle detected by sensor two. Waiting...")
                    distance_2 = ultrasonic_sensor_2.distance_centimeters  # Recheck the distance
                    sleep(1)  # Small delay to reduce polling frequency

                # Continue forward
                print("Retrying forward motion.")
                #sound.speak("Retrying forward")
                continue

            # Short delay to avoid excessive polling
            sleep(0.2)


        except Exception as e:
            print("Error during program execution: {}".format(e))
            sound.speak("Error encountered. Stopping.")
            tank_drive.off()
            break

    # Ensure motors are stopped when the program ends
    tank_drive.off()
    print("Program terminated.")
    sound.speak("Exploration ended.")

if __name__ == "__main__":
    main()




