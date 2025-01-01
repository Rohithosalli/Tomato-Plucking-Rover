import cv2
import pandas as pd
from ultralytics import YOLO
import cvzone
import serial
import time

# Configure serial communication with Arduino for arm and rover
arduino_port = "/dev/ttyUSB0"  # Update with your port
baud_rate = 9600
arduino = serial.Serial(arduino_port, baud_rate, timeout=1)
time.sleep(2)  # Allow Arduino to initialize

# Webcam configuration
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1024)  # Set width
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 768)  # Set height

# Load YOLO model
model = YOLO('best.pt')

# Load class list
with open("coco1.txt", "r") as my_file:
    class_list = my_file.read().split("\n")

# Calibration factor (pixels to cm)
arm_length_factor = 25
cm_per_pixel = arm_length_factor / 1024

# Initialize last angles for all servos
last_angles = {0: 180, 1: 180, 2: 0, 3: 0}
task_in_progress = False


def send_to_arduino_smooth(servo, target_angle, step_delay=0.025):
    """Send servo and angle command to Arduino incrementally for smooth movement."""
    global last_angles
    current_angle = last_angles[servo]
    step = 2 if target_angle > current_angle else -2

    for angle in range(current_angle, target_angle + step, step):
        command = f"{servo},{angle}\n"
        arduino.write(command.encode())
        time.sleep(step_delay)

    last_angles[servo] = target_angle


def reset_arm():
    """Reset the robotic arm to its initial position."""
    print("Resetting arm to initial position...")
    send_to_arduino_smooth(3, 0)  # Base rotation
    send_to_arduino_smooth(2, 0)  # Arm lift
    send_to_arduino_smooth(1, 180)  # Wrist control
    send_to_arduino_smooth(0, 180)  # Gripper open
    print("Arm reset complete.")


def send_rover_command(command):
    """Send movement commands to the rover."""
    arduino.write(f"{command}\n".encode())
    print(f"Sent to Rover: {command}")


try:
    reset_arm()
    send_rover_command("forward")  # Start the rover

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to capture frame")
            break

        # Correct the inverted frame
        frame = cv2.flip(frame, -1)

        # Run YOLO prediction
        results = model.predict(frame)
        a = results[0].boxes.data
        px = pd.DataFrame(a).astype("float")

        tomato_detected = False

        for index, row in px.iterrows():
            x1 = int(row[0])
            y1 = int(row[1])
            x2 = int(row[2])
            y2 = int(row[3])
            d = int(row[5])
            c = class_list[d]

            # If a tomato is detected
            if c.lower() == "tomato":
                tomato_detected = True
                send_rover_command("stop")  # Stop the rover
                task_in_progress = True

                # Draw bounding box and label
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                cvzone.putTextRect(frame, f'{c}', (x1, y1), 1, 1)

                # Calculate robot coordinates
                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                x_robot = cy * cm_per_pixel
                y_robot = cx * cm_per_pixel

                print(f"Tomato detected at: ({cx}, {cy})")

                # Move servos to pick up the tomato
                send_to_arduino_smooth(3, int(90 - x_robot))  # Base rotation
                send_to_arduino_smooth(2, int(45 + y_robot))  # Arm lift
                send_to_arduino_smooth(1, 30)                 # Wrist control
                send_to_arduino_smooth(0, 90)                 # Close gripper

                time.sleep(1)

                # Drop the tomato at the predefined position
                send_to_arduino_smooth(3, 180)  # Base rotation
                send_to_arduino_smooth(2, 45)
                send_to_arduino_smooth(1, 90)
                send_to_arduino_smooth(0, 180)  # Open gripper

                time.sleep(1)
                reset_arm()  # Reset arm
                task_in_progress = False
                send_rover_command("forward")  # Restart the rover

        if not tomato_detected and not task_in_progress:
            send_rover_command("forward")  # Continue moving forward

        cv2.imshow("Tomato Detection", frame)

        if cv2.waitKey(1) == ord('q'):
            send_rover_command("stop")  # Stop rover on quit
            break

except KeyboardInterrupt:
    print("Program terminated.")
    send_rover_command("stop")

finally:
    cap.release()
    cv2.destroyAllWindows()
    arduino.close()
