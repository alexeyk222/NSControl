#!/usr/bin/python3

import cv2
import numpy as np
import dlib
import RPi.GPIO as GPIO
import threading
import time
from math import hypot

GPIO.setmode(GPIO.BOARD)
inputPin = 16
inputPin1 = 23
inputPin2 = 21
inputPin3 = 18

GPIO.setup(inputPin, GPIO.IN)
GPIO.setup(inputPin1, GPIO.IN)
GPIO.setup(inputPin2, GPIO.IN)
GPIO.setup(inputPin3, GPIO.IN)

GPIO.setup(33, GPIO.OUT)
my_pwm = GPIO.PWM(33, 100)

t = 100
sec = 0
sec1 = 0
time_now = 0


volume = False
thread_lock = threading.Lock()


def gstreamer_pipeline(
    capture_width=1280,
    capture_height=720,
    display_width=1280,
    display_height=720,
    framerate=30,
    flip_method=0,
):
    return (
        "nvarguscamerasrc ! "
        "video/x-raw(memory:NVMM), "
        "width=(int)%d, height=(int)%d, "
        "format=(string)NV12, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )


cap = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
detector = dlib.get_frontal_face_detector()
predictor = dlib.shape_predictor('data/shape_predictor_68_face_landmarks.dat')


def midpoint(p1, p2):
    return int((p1.x + p2.x) / 2), int((p1.y + p2.y) / 2)


font = cv2.FONT_HERSHEY_SIMPLEX


def get_blinking_ratio(eye_points, facial_landmarks):
    left_point = (
        facial_landmarks.part(eye_points[0]).x,
        facial_landmarks.part(eye_points[0]).y,
    )
    right_point = (
        facial_landmarks.part(eye_points[3]).x,
        facial_landmarks.part(eye_points[3]).y,
    )

    center_point = midpoint(
        facial_landmarks.part(eye_points[1]), facial_landmarks.part(eye_points[2])
    )
    center_bottom = midpoint(
        facial_landmarks.part(eye_points[5]), facial_landmarks.part(eye_points[4])
    )

    eyes_line_length = hypot(
        (left_point[0] - right_point[0]), (left_point[1] - right_point[1])
    )
    vert_line_length = hypot(
        (center_point[0] - center_bottom[0]), (center_point[1] - center_bottom[1])
    )

    ratio = eyes_line_length / vert_line_length
    return ratio


def get_threshold_ratio(eye_points, facial_landmarks):
    left_eye_region = np.array(
        [
            (
                facial_landmarks.part(eye_points[0]).x,
                facial_landmarks.part(eye_points[0]).y,
            ),
            (
                facial_landmarks.part(eye_points[1]).x,
                facial_landmarks.part(eye_points[1]).y,
            ),
            (
                facial_landmarks.part(eye_points[2]).x,
                facial_landmarks.part(eye_points[2]).y,
            ),
            (
                facial_landmarks.part(eye_points[3]).x,
                facial_landmarks.part(eye_points[3]).y,
            ),
            (
                facial_landmarks.part(eye_points[4]).x,
                facial_landmarks.part(eye_points[4]).y,
            ),
            (
                facial_landmarks.part(eye_points[5]).x,
                facial_landmarks.part(eye_points[5]).y,
            ),
        ],
        np.int32,
    )

    height, width, _ = frame.shape
    mask = np.zeros((height, width), np.uint8)
    cv2.polylines(mask, [left_eye_region], True, 255, 2)
    cv2.fillPoly(mask, [left_eye_region], 255)
    eye = cv2.bitwise_and(gray, gray, mask=mask)

    min_x = np.min(left_eye_region[:, 0])
    max_x = np.max(left_eye_region[:, 0])
    min_y = np.min(left_eye_region[:, 1])
    max_y = np.max(left_eye_region[:, 1])

    gray_eye = eye[min_y: max_y, min_x: max_x]
    _, threshold_eye = cv2.threshold(gray_eye, 70, 255, cv2.THRESH_BINARY_INV)
    height, width = threshold_eye.shape

    left_side_threshold = threshold_eye[0: height, 0: int(width / 2)]
    left_side_white = cv2.countNonZero(left_side_threshold)

    right_side_threshold = threshold_eye[0: height, int(width / 2) : width]
    right_side_white = cv2.countNonZero(right_side_threshold)

    if left_side_white == 0:
        gaze_ratio = 1
    elif right_side_white == 0:
        gaze_ratio = 5
    else:
        gaze_ratio = left_side_white / right_side_white

    return gaze_ratio


def recognition():
    global gray, frame, faces, volume, sec, sec1

    first_check = True
    last_check = 0
    exit_time = 0
    time_now = 0
    last_check2 = 0
    exit_time2 = 0
    time_now2 = 0
    second_check = True
    
    
    while True:
        _, frame = cap.read()
        frame = cv2.resize(frame, (680, 460))
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = detector(gray)
        x = GPIO.input(inputPin)
        x1 = GPIO.input(inputPin1)
        x2 = GPIO.input(inputPin2)
        x3 = GPIO.input(inputPin3)
        
        if x3 == 1:
            for _ in range(3):
                my_pwm.start(t)
                time.sleep(1)
                my_pwm.start(0)
                time.sleep(5)
        
        if x == 1:
            for face in faces:
                landmarks = predictor(gray, face)
                left_eye_ratio = get_blinking_ratio([36, 37, 38, 39, 40, 41], landmarks)
                right_eye_ratio = get_blinking_ratio([42, 43, 44, 45, 46, 47], landmarks)
                blinking_ratio = (left_eye_ratio + right_eye_ratio) / 2
                #print(blinking_ratio)


                if blinking_ratio > 3.5:
                    
                    #print('blink_prepare')
                    last_check = time.time()
                    if first_check == True:
                        time_now = time.time()
                        #print(str(time_now) + ' 0')
                        first_check = False

                    
                    
                    #print((time.time() - time_now), blinking_ratio)
                    if (time.time() - time_now) > 3:
                        volume = 100
                        
                        #print('BLINK')
                        first_check = True
                elif blinking_ratio < 3.5:
                    exit_time = time.time()
                    
                    gaze_ratio_left_eye = get_threshold_ratio([36, 37, 38, 39, 40, 41], landmarks)
                    gaze_ratio_right_eye = get_threshold_ratio([42, 43, 44, 45, 46, 47], landmarks)
                    gaze_ratio = (gaze_ratio_right_eye + gaze_ratio_left_eye / 2)
                    
                    last_check2 = time.time()

                    if second_check == True:
                        time_now2 = time.time()
                        #print(str(time_now) + ' 0')
                        second_check = False

                    if gaze_ratio <= 0.7:
                        if (time.time() - time_now2) > 3:
                            volume = 100
                            #print('LEFT')
                            second_check = True

                    elif gaze_ratio >= 1.8:
                        if (time.time() - time_now2) > 3:
                            volume = 100
                            #print('RIGHT')
                            second_check = True
                    else:
                        exit_time2 = time.time()
                    
        if abs(last_check - exit_time) < 1:
                    first_check = True
                    #print('BREAK')
        if abs(last_check2 - exit_time2) < 0.4:
                    second_check_check = True
                    #print('BREAK2')
        
        
        #cv2.imshow("video6", gray)
        process_thread = threading.Thread(target=tick)
        key = cv2.waitKey(1)
        if key == 27:
            break


def tick():
    global volume
    while True:
        if volume:
            for _ in range(3):
                my_pwm.start(100)
                time.sleep(0.5)
                my_pwm.start(0)
                time.sleep(0.5)
            volume = False


process_thread2 = threading.Thread(target=recognition)
process_thread = threading.Thread(target=tick)

process_thread.start()
process_thread2.start()
