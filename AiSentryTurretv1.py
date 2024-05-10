#Code written by Ben Braham

#MIT License

#Copyright (c) 2024 BenBmakes

#Permission is hereby granted, free of charge, to any person obtaining a copy
#of this software and associated documentation files (the "Software"), to deal
#in the Software without restriction, including without limitation the rights
#to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#copies of the Software, and to permit persons to whom the Software is
#furnished to do so, subject to the following conditions:

#The above copyright notice and this permission notice shall be included in all
#copies or substantial portions of the Software.

#THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
#SOFTWARE.

import cv2
import numpy as np
import serial
import time

# Connect to Arduino via USB serial
ser = serial.Serial('COM3', 9600)  # Change 'COM3' to your Arduino's port

# Define camera resolution (change these values according to your camera)
CAMERA_WIDTH = 620
CAMERA_HEIGHT = 480

# Define deadband size (adjust as needed)
DEADBAND_SIZE = 1

# Define filtering parameters (adjust as needed)
FILTER_SIZE = 30

# Define timeout duration (adjust as needed)
TIMEOUT_DURATION = 2  # 2 seconds

# Define confidence threshold and NMS threshold
CONFIDENCE_THRESHOLD = 0.23  # Adjust as needed
NMS_THRESHOLD = 0.15  # Adjust as needed

# Initialize variables for filtering
x_filter = []
y_filter = []

# Load YOLO-Lite model
net = cv2.dnn.readNet("yolov3-tiny.weights", "yolov3-tiny.cfg")
classes = []
with open("coco.names", "r") as f:
    classes = [line.strip() for line in f.readlines()]
layer_names = net.getLayerNames()

# Manually search for output layers containing "yolo" or "YOLO"
output_layers = [layer_name for layer_name in layer_names if "yolo" in layer_name.lower()]

def calculate_servo_movements(box, scale_factor=0.8):
    global x_filter, y_filter

    # Calculate the position of the person bounding box relative to the center of the camera's view
    box_center_x = (box[0] + box[2]) / 2
    box_center_y = (box[1] + box[3]) / 2
    camera_center_x = CAMERA_WIDTH / 2
    camera_center_y = CAMERA_HEIGHT / 2
    dx = camera_center_x - box_center_x
    dy = camera_center_y - box_center_y

    # Apply deadband
    if abs(dx) < DEADBAND_SIZE:
        dx = 0
    if abs(dy) < DEADBAND_SIZE:
        dy = 0

    # Apply filtering
    x_filter.append(dx)
    y_filter.append(dy)
    if len(x_filter) > FILTER_SIZE:
        x_filter.pop(0)
    if len(y_filter) > FILTER_SIZE:
        y_filter.pop(0)
    dx_filtered = sum(x_filter) / len(x_filter)
    dy_filtered = sum(y_filter) / len(y_filter)

    # Scale down the movement
    dx_scaled = dx_filtered * scale_factor
    dy_scaled = dy_filtered * scale_factor

    # Convert the scaled position to servo movements
    servo1_movement = int(90 + dx_scaled * 90 / (CAMERA_WIDTH / 2))  # Scale the movement to servo range (0-180)
    servo2_movement = int(90 + dy_scaled * 90 / (CAMERA_HEIGHT / 2))  # Scale the movement to servo range (0-180)

    return servo1_movement, servo2_movement

def send_servo_commands(servo1_movement, servo2_movement):
    # Send servo movements to Arduino
    ser.write(f"{servo1_movement},{servo2_movement}\n".encode())

def return_to_middle():
    # Return all servos to the middle position
    send_servo_commands(90, 90)

def main():
    last_detection_time = time.time()

    # Open camera
    cap = cv2.VideoCapture(0)  # Use the default camera (index 0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)

    while True:
        # Capture frame from camera
        ret, frame = cap.read()
        if not ret:
            print("Failed to capture frame")
            break

        # Perform object detection using YOLO-Lite
        blob = cv2.dnn.blobFromImage(frame, 1/255.0, (416, 416), swapRB=True, crop=False)
        net.setInput(blob)
        outs = net.forward(output_layers)

        # Process detections
        boxes = []
        confidences = []
        class_ids = []
        for out in outs:
            for detection in out:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                if confidence > CONFIDENCE_THRESHOLD and class_id == 0:  # Adjusting the confidence threshold
                    # Object detected is a person with confidence > CONFIDENCE_THRESHOLD
                    center_x = int(detection[0] * CAMERA_WIDTH)
                    center_y = int(detection[1] * CAMERA_HEIGHT)
                    w = int(detection[2] * CAMERA_WIDTH)
                    h = int(detection[3] * CAMERA_HEIGHT)
                    x = int(center_x - w / 2)
                    y = int(center_y - h / 2)
                    boxes.append([x, y, w, h])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)

        # Non-maximum suppression to remove redundant detections
        indexes = cv2.dnn.NMSBoxes(boxes, confidences, CONFIDENCE_THRESHOLD, NMS_THRESHOLD)  # Adjusting the NMS threshold

        # Process filtered detections
        for i in range(len(boxes)):
            if i in indexes:
                x, y, w, h = boxes[i]
                cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
                person_box = (x, y, x + w, y + h)
                servo1_movement, servo2_movement = calculate_servo_movements(person_box)
                send_servo_commands(servo1_movement, servo2_movement)

        # Show frame with detections
        cv2.imshow('Frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    ser.close()

if __name__ == "__main__":
    main()
