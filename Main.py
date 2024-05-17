from cvzone.FaceDetectionModule import FaceDetector
from pyfirmata import Arduino, SERVO, PWM, OUTPUT

import cv2
import cvzone
import math
import time

class Motor:
    def __init__(self, board, ena_pin, in1_pin, in2_pin):
        self.ena = board.get_pin(f'd:{ena_pin}:p')
        self.in1 = board.get_pin(f'd:{in1_pin}:o')
        self.in2 = board.get_pin(f'd:{in2_pin}:o')
        
    def forward(self, speed):
        self.in1.write(1)
        self.in2.write(0)
        self.ena.write(speed)
        
    def backward(self, speed):
        self.in1.write(0)
        self.in2.write(1)
        self.ena.write(speed)

    def stop(self):
        self.in1.write(0)
        self.in2.write(0)
        self.ena.write(0)

# OpenCV / Board Settings
cap = cv2.VideoCapture(1)
board = Arduino('COM5')

cap.set(cv2.CAP_PROP_FPS, 20)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

detector = FaceDetector(minDetectionCon=0.3, modelSelection=0)

motor1_ena = 3
motor1_in1 = 2
motor1_in2 = 4

motor2_ena = 5
motor2_in1 = 6
motor2_in2 = 7

motor3_ena = 9
motor3_in1 = 8
motor3_in2 = 10

motor4_ena = 11
motor4_in1 = 12
motor4_in2 = 13

# Pixel Values Settings
pTime = 0

TURN_MIN_VALUE = 80
TURN_MAX_VALUE = 220

DISTANCE_ERROR_RANGE = [150, 250] # min, max
DISTANCE_PWN_RANGE = 180

PWM_SCALE = [0.50, 1.00]

motor1 = Motor(board, motor1_ena, motor1_in1, motor1_in2)
motor2 = Motor(board, motor2_ena, motor2_in1, motor2_in2)
motor3 = Motor(board, motor3_ena, motor3_in1, motor3_in2)
motor4 = Motor(board, motor4_ena, motor4_in1, motor4_in2)


def RangeCalc(In, in_max, in_min, out_max, out_min):
    # mapped_value = (x_clipped - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    x = min(max(In, in_min), in_max)
    mapped_value = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    mapped_value = round(mapped_value, 2)
    return mapped_value


while True:

    cTime = time.time()
    fps = 1 / (cTime - pTime)
    pTime = cTime

    success, img = cap.read()
    height, width, _ = img.shape
    img_center_x = width // 2
    img_center_y = height // 2
    img_center = [img_center_x, img_center_y]

    img, bboxs = detector.findFaces(img, draw=False)

    if bboxs:

        bbox = bboxs[0]

        center = bbox["center"]
        x, y, w, h = bbox['bbox']
        
        turn_direc = img_center_x - center[0]
        distance_scale = w

        # UI
        cv2.circle(img, center, 5, (255, 0, 0), cv2.FILLED) # Bbox Center
        cvzone.cornerRect(img, (x, y, w, h), 30, 3, 3, (255,0,255), (255,0,255)) # Bbox
        cv2.line(img, center, (img_center_x, img_center_y), (255 ,0, 255), 2) # Line Bbox -> Center Img

        cv2.putText(img, f"Distance :  {distance_scale}", (20, 180), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 2)
        cv2.putText(img, f"Turn Offset Value :  {turn_direc}", (20, 80), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 2)

    
        if distance_scale < DISTANCE_ERROR_RANGE[0]: # DISTANCE = FAR

            cv2.putText(img, "Action : Far", (20, 200), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 2)

            pwm = RangeCalc(distance_scale, DISTANCE_ERROR_RANGE[0], DISTANCE_ERROR_RANGE[0] - DISTANCE_PWN_RANGE, PWM_SCALE[0], PWM_SCALE[1])
            cv2.putText(img, f"PWM :  {pwm}", (20, 220), cv2.FONT_HERSHEY_PLAIN, 1, (255, 0, 255), 2)

        elif distance_scale > DISTANCE_ERROR_RANGE[1]: # DISTANCE = NEAR

            cv2.putText(img, "Action : Near", (20, 200), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 2)

            pwm = RangeCalc(distance_scale, DISTANCE_ERROR_RANGE[1] + DISTANCE_PWN_RANGE, DISTANCE_ERROR_RANGE[1], PWM_SCALE[1], PWM_SCALE[0])
            cv2.putText(img, f"PWM :  {pwm}", (20, 220), cv2.FONT_HERSHEY_PLAIN, 1, (255, 0, 255), 2)

        else: # MOTOR TURNING / DISTANCE = OPTIMAL

            if abs(turn_direc) > TURN_MIN_VALUE:

                if turn_direc < 0:
                    cv2.putText(img, "Turn Direction : Right", (20, 100), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 2)

                    pwm = RangeCalc(abs(turn_direc), TURN_MAX_VALUE, TURN_MIN_VALUE, PWM_SCALE[1], PWM_SCALE[0])
                    cv2.putText(img, f"PWM :  {pwm}", (20, 120), cv2.FONT_HERSHEY_PLAIN, 1, (255, 0, 255), 2)

                else:
                    cv2.putText(img, "Turn Direction : Left", (20, 100), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 2)

                    pwm = RangeCalc(abs(turn_direc), TURN_MAX_VALUE, TURN_MIN_VALUE, PWM_SCALE[1], PWM_SCALE[0])
                    cv2.putText(img, f"PWM :  {pwm}", (20, 120), cv2.FONT_HERSHEY_PLAIN, 1, (255, 0, 255), 2)

    
    cv2.putText(img, f'FPS : {int(fps)}', (20, 40), cv2.FONT_HERSHEY_PLAIN, 1, (255, 0, 0), 2) # FPS
    cv2.circle(img, (img_center_x, img_center_y), 5, (255, 0, 0), cv2.FILLED) # Middle Circle
    cv2.line(img, (0, img_center_y), (width, img_center_y), (0, 255, 0), 1)  # Horizontal line
    cv2.line(img, (img_center_x, 0), (img_center_x, height), (0, 255, 0), 1) # Vertical line

    cv2.imshow("Image", img)

    key = cv2.waitKey(1)
    if key == ord('q'):
        break

cap.release()
board.exit()