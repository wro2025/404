import cv2
import numpy as np
import RPi.GPIO as GPIO
import time

# Setup GPIO for ultrasonic sensors
GPIO.setmode(GPIO.BCM)

# Define GPIO pins for ultrasonic sensors
RIGHT_TRIG = 16
RIGHT_ECHO = 20
LEFT_TRIG = 5
LEFT_ECHO = 6
CENTRAL_TRIG = 13
CENTRAL_ECHO = 19

# Define GPIO Motor control pins
IN1 = 17   # Motor forward
IN2 = 27   # Motor backward
ENA = 18  # PWM for speed
IN3 = 22   # Servo forward
IN4 = 23   # Servo backward
ENB = 24  # PWM for servo speed

#Setup pins
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(ENA, GPIO.OUT)
GPIO.setup(IN3, GPIO.OUT)  
GPIO.setup(IN4, GPIO.OUT)
GPIO.setup(ENB, GPIO.OUT)

GPIO.setup(RIGHT_TRIG, GPIO.OUT)
GPIO.setup(RIGHT_ECHO, GPIO.IN)
GPIO.setup(LEFT_TRIG, GPIO.OUT)
GPIO.setup(LEFT_ECHO, GPIO.IN)
GPIO.setup(CENTRAL_TRIG, GPIO.OUT)
GPIO.setup(CENTRAL_ECHO, GPIO.IN)

# PWM setup
pwm_motor = GPIO.PWM(ENA, 1000)  # 1kHz PWM
pwm_motor.start(100) # 25%
pwm_motor.ChangeDutyCycle(100)  # Set initial speed to 50%

pwm_servo = GPIO.PWM(ENB, 1000)  # 50Hz for servo
pwm_servo.start(100) # 25%
pwm_servo.ChangeDutyCycle(100)  # Set initial speed to 50%

def forward():
    GPIO.output(IN1, False)
    GPIO.output(IN2, True)
    GPIO.output(IN3, False)
    GPIO.output(IN4, False)

def left_turn():
    GPIO.output(IN3, False)
    GPIO.output(IN4, True)
    GPIO.output(IN2, True)
    GPIO.output(IN1, False) 

    pwm_servo.start(100)
    pwm_servo.ChangeDutyCycle(100)


def right_turn():
    GPIO.output(IN4, False)
    GPIO.output(IN3, True)
    GPIO.output(IN1, False)
    GPIO.output(IN2, True) 

    pwm_servo.start(100)
    pwm_servo.ChangeDutyCycle(100)


def stop():
    GPIO.output(IN3, False)  # Forward
    GPIO.output(IN4, False) 
    GPIO.output(IN1, False)
    GPIO.output(IN2, False)
    GPIO.cleanup()


def measure_distance(trig_pin, echo_pin):
    # Send 10us pulse to trigger
    GPIO.output(trig_pin, False)
    time.sleep(0.05)  # Let sensor settle
    GPIO.output(trig_pin, True)
    time.sleep(0.00001)  # 10 microseconds
    GPIO.output(trig_pin, False)

    # Wait for echo start
    timeout = time.time() + 1  # 1s timeout
    while GPIO.input(echo_pin) == 0:
        if time.time() > timeout:
            return None  # Timeout
    pulse_start = time.time()

    # Wait for echo end
    timeout = time.time() + 1
    while GPIO.input(echo_pin) == 1:
        if time.time() > timeout:
            return None
    pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start

    # Speed of sound is ~34300 cm/s
    distance_cm = pulse_duration * 17150
    return round(distance_cm, 2)

HW_MAX_LEFT = 400
HW_MAX_RIGHT = 140

servo_angle = 0

dist_left = measure_distance(LEFT_TRIG, LEFT_ECHO)
dist_right = measure_distance(RIGHT_TRIG, RIGHT_ECHO)
dist_front  = measure_distance(CENTRAL_TRIG, CENTRAL_ECHO)

delta_dist_left = 0
delta_dist_front = 0
delta_dist_right = 0

DESIRED_DIST = 30

counter = 0
MIN_DIFF = 70
aft_counter = 0
MAX_AFT_CNT = 6
MAX_AFT_DIFF = 10

TIMEOUT = 2.1
end_time = 0

turning = False
start_turn_time = 0
TURN_DURATION = 0.5
turn_orientation = ""

TURN_CD = 3.25
start_turn_cd_time = None

orientation = ""
"""
forward()
time.sleep(0.5)
while True:
  servo_angle = 0
  
  delta_dist_left = dist_left
  delta_dist_front = dist_front
  delta_dist_right = dist_right
  
  dist_left = measure_distance(LEFT_TRIG, LEFT_ECHO)
  dist_right = measure_distance(RIGHT_TRIG, RIGHT_ECHO)
  dist_front  = measure_distance(CENTRAL_TRIG, CENTRAL_ECHO)

  delta_dist_left = dist_left - delta_dist_left
  delta_dist_front = dist_front - delta_dist_front
  delta_dist_right = dist_right - delta_dist_right
  
  print(str(dist_left) + " " + str(dist_front) + " " + str(dist_right))
  print(str(delta_dist_left) + " " + str(delta_dist_front) + " " + str(delta_dist_right))

  if (dist_front <= 45) and not turning and (start_turn_cd_time is None or time.time() - start_turn_cd_time > TURN_CD):
    turning = True
    start_turn_time = time.time()
    start_turn_cd_time = None
    counter += 1
    
    if turn_orientation == "":
      if dist_left < dist_right: turn_orientation = "right"
      else: turn_orientation = "left"

  if turning:
    print("TURNING")
    # Right turn
    if turn_orientation == "right":
      # servo_angle += (1 + (60 - dist_front) / 60) * 0.5
      servo_angle += 1
    elif turn_orientation == "left":
      # servo_angle -= (1 + (60 - dist_front) / 60) * 0.5
      servo_angle -= 1
    
    if dist_front > 50 and time.time() - start_turn_time > TURN_DURATION:
      turning = False
      start_turn_time = 0
      start_turn_cd_time = time.time()
  
  else:
    if turn_orientation == "right":
      servo_angle += 1.3 * (DESIRED_DIST - dist_left) / DESIRED_DIST
    if turn_orientation == "left":
      servo_angle -= 1.3 * (DESIRED_DIST - dist_right) / DESIRED_DIST

  if counter >= 12:
    if end_time == 0: end_time = time.time()

    if time.time() - end_time > TIMEOUT:
      break

  print("SERVO ANGLE: " + str(servo_angle))

  if servo_angle > 0:
    right_turn()
  elif servo_angle < 0:
    left_turn()
  else:
    forward()
  time.sleep(0.01)  

stop()
"""
try:
  
    while True:
        distance_central=measure_distance(CENTRAL_TRIG,CENTRAL_ECHO)
        distance_left=measure_distance(LEFT_TRIG,LEFT_ECHO)
        distance_right=measure_distance(RIGHT_TRIG,RIGHT_ECHO)

        print("Lijeva distanca je: ", distance_left)
        print("Centralna distanca je: ", distance_central)
        print("Desna distanca je: ", distance_right)


        if distance_left > distance_right:
                left_turn()
        elif distance_right > distance_left:
                right_turn()
        else:
            forward()
        if distance_central<5:
            stop()
            break
        time.sleep(0.01)      
            
except KeyboardInterrupt:
    print("Interrupted")
finally:
    stop()
    pwm_motor.stop()
    pwm_servo.stop()
    GPIO.cleanup()