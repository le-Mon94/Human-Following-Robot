from cvzone.PoseModule import PoseDetector
from pyfirmata import Arduino, SERVO, PWM, OUTPUT, util

import cv2
import cvzone
import math
import time
import RPi.GPIO as GPIO

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
cap = cv2.VideoCapture('/dev/video0')
board = Arduino('/dev/ttyACM0')
GPIO.setmode(GPIO.BCM)

cap.set(cv2.CAP_PROP_FPS, 20)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

detector = PoseDetector(staticMode=False, modelComplexity=0, smoothLandmarks=True, enableSegmentation=False, smoothSegmentation=True, detectionCon=0.5, trackCon=0.5)
it = util.Iterator(board)

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

GPIO.setup(17, GPIO.OUT) # Red
GPIO.setup(27, GPIO.OUT) # Yellow
GPIO.setup(22, GPIO.OUT) # Green
GPIO.setup(16, GPIO.OUT) # Buzzer

# Pixel Values Settings
pTime = 0

TURN_MIN_VALUE = 30
TURN_MAX_VALUE = 160

DISTANCE_MIN_VALUE = 30
DISTANCE_MAX_VALUE = 120

PWM_SCALE = [0.60, 1.00]

motor1 = Motor(board, motor1_ena, motor1_in1, motor1_in2)
motor2 = Motor(board, motor2_ena, motor2_in1, motor2_in2)
motor3 = Motor(board, motor3_ena, motor3_in1, motor3_in2)
motor4 = Motor(board, motor4_ena, motor4_in1, motor4_in2)

def Buzzer():
    GPIO.output(16, GPIO.HIGH)
    time.sleep(0.5)
    GPIO.output(16, GPIO.LOW)
    time.sleep(0.2)
    GPIO.output(16, GPIO.HIGH)
    time.sleep(0.5)
    GPIO.output(16, GPIO.LOW)

def RangeCalc(In, in_max, in_min, out_max, out_min):
    # mapped_value = (x_clipped - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    x = min(max(In, in_min), in_max)
    mapped_value = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    mapped_value = round(mapped_value, 2)
    return mapped_value

it.start()
Buzzer()

while True:
    
    GPIO.output(17, GPIO.HIGH)

    cTime = time.time()
    fps = 1 / (cTime - pTime)
    pTime = cTime

    success, img = cap.read()
    height, width, _ = img.shape
    img_center_x = width // 2
    img_center_y = height // 2
    img_center = [img_center_x, img_center_y]

    img = detector.findPose(img)

    imList, bboxs = detector.findPosition(img, draw=True, bboxWithHands=False)

    if bboxs:
        
        GPIO.output(22, GPIO.HIGH)

        bbox = bboxs[0]

        center = bbox["center"]
        x, y, w, h = bbox['bbox']
        
        turn_direc = img_center_x - center[0]
        distance_scale = img_center_y - center[1]

        # UI
        cv2.circle(img, center, 5, (255, 0, 0), cv2.FILLED) # Bbox Center
        cvzone.cornerRect(img, (x, y, w, h), 30, 3, 3, (255,0,255), (255,0,255)) # Bbox
        cv2.line(img, center, (img_center_x, img_center_y), (255 ,0, 255), 2) # Line Bbox -> Center Img

        cv2.putText(img, f"Distance :  {distance_scale}", (20, 180), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 2)
        cv2.putText(img, f"Turn Offset Value :  {turn_direc}", (20, 80), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 2)

        if abs(distance_scale) > DISTANCE_MIN_VALUE:
            
            if distance_scale < 0: # DISTANCE = FAR

                cv2.putText(img, "Action : Far", (20, 200), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 2)
                
                pwm = RangeCalc(abs(distance_scale), DISTANCE_MAX_VALUE, DISTANCE_MIN_VALUE, PWM_SCALE[1], PWM_SCALE[0])
                cv2.putText(img, f"PWM :  {pwm}", (20, 220), cv2.FONT_HERSHEY_PLAIN, 1, (255, 0, 255), 2)
                
                motor1.forward(pwm)
                motor2.forward(pwm)
                motor3.forward(pwm)
                motor4.forward(pwm)
                
            else: # DISTANCE = NEAR

                cv2.putText(img, "Action : Near", (20, 200), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 2)

                pwm = RangeCalc(abs(distance_scale), DISTANCE_MAX_VALUE, DISTANCE_MIN_VALUE, PWM_SCALE[1], PWM_SCALE[0])
                cv2.putText(img, f"PWM :  {pwm}", (20, 220), cv2.FONT_HERSHEY_PLAIN, 1, (255, 0, 255), 2)
                
                motor1.backward(pwm)
                motor2.backward(pwm)
                motor3.backward(pwm)
                motor4.backward(pwm)

        else: # MOTOR TURNING / DISTANCE = OPTIMAL

            if abs(turn_direc) > TURN_MIN_VALUE:

                if turn_direc < 0:
                    cv2.putText(img, "Turn Direction : Right", (20, 100), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 2)

                    pwm = RangeCalc(abs(turn_direc), TURN_MAX_VALUE, TURN_MIN_VALUE, PWM_SCALE[1], PWM_SCALE[0])
                    cv2.putText(img, f"PWM :  {pwm}", (20, 120), cv2.FONT_HERSHEY_PLAIN, 1, (255, 0, 255), 2)

                    motor1.forward(pwm)
                    motor2.backward(pwm)
                    motor3.forward(pwm)
                    motor4.backward(pwm)
                
                else:
                    cv2.putText(img, "Turn Direction : Left", (20, 100), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 2)

                    pwm = RangeCalc(abs(turn_direc), TURN_MAX_VALUE, TURN_MIN_VALUE, PWM_SCALE[1], PWM_SCALE[0])
                    cv2.putText(img, f"PWM :  {pwm}", (20, 120), cv2.FONT_HERSHEY_PLAIN, 1, (255, 0, 255), 2)
                    
                    motor1.backward(pwm)
                    motor2.forward(pwm)
                    motor3.backward(pwm)
                    motor4.forward(pwm)
            else:
                        
                motor1.stop()
                motor2.stop()
                motor3.stop()
                motor4.stop()
    else:
        
        GPIO.output(22, GPIO.LOW)
        motor1.stop()
        motor2.stop()
        motor3.stop()
        motor4.stop()

    
    cv2.putText(img, f'FPS : {int(fps)}', (20, 40), cv2.FONT_HERSHEY_PLAIN, 1, (255, 0, 0), 2) # FPS
    cv2.circle(img, (img_center_x, img_center_y), 5, (255, 0, 0), cv2.FILLED) # Middle Circle
    cv2.line(img, (0, img_center_y), (width, img_center_y), (0, 255, 0), 1)  # Horizontal line
    cv2.line(img, (img_center_x, 0), (img_center_x, height), (0, 255, 0), 1) # Vertical line
    
    img = cv2.resize(img, (width*2, height*2))
    #cv2.imshow("Image", img)

    key = cv2.waitKey(1)
    if key == ord('q'):
        GPIO.output(17, GPIO.LOW)
        GPIO.output(22, GPIO.LOW)
        
        motor1.stop()
        motor2.stop()
        motor3.stop()
        motor4.stop()
        break

GPIO.output(17, GPIO.LOW)
GPIO.output(22, GPIO.LOW)
GPIO.cleaup()

motor1.stop()
motor2.stop()
motor3.stop()
motor4.stop()

cap.release()
board.exit()