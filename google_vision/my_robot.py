#!/usr/bin/env python3

"""
This script runs continuous face detection on the Vision Bonnet and sends face parameters to Arduino robot car via 
GPIO expansion pins of Vision Bonnet.

PIN_A is used to send to Arduino the coded angle between the direction the robot faces and direction to the detected face 
PIN_B is used to send to Arduino the coded distance in inches to the detected face (coded to the interval between 0.2 and 0.9) 
      If value of 0.1 is sent it means no face is detected on the frame 
PIN_C is used to send to Arduino the coded frame number used to check if face detector is running 
PIN_D monitors if Arduino is powered up and is used for safe shutdown of Google Visin AIY kit

"""
import argparse
import os
import RPi.GPIO as GPIO

from aiy.vision.inference import CameraInference
from aiy.vision.models import face_detection
from picamera import PiCamera

from time import sleep
from math import atan2
from gpiozero import PWMLED
from gpiozero import LED
from gpiozero import Button

from aiy.leds import Leds
from aiy.pins import PIN_A
from aiy.pins import PIN_B
from aiy.pins import PIN_C
from aiy.pins import PIN_D

# Colors for LED
RED = (0xFF, 0x00, 0x00)
GREEN = (0x00, 0xFF, 0x00)
BLUE = (0x00, 0x00, 0xFF)
YELLOW = (0xFF, 0xFF, 0x00)
PURPLE = (0xFF, 0x00, 0xFF)
CYAN = (0x00, 0xFF, 0xFF)
WHITE = (0xFF, 0xFF, 0xFF)

# Initialize the GPIO pins A,B,C,D
pin_A = PWMLED(PIN_A)
pin_B = PWMLED(PIN_B)
pin_C = PWMLED(PIN_C)
pin_D = Button(PIN_D)

# Initialize LED (in the button on the top of AIY Google Vision box)
leds = Leds()

# Blink LED (color, period in seconds, n_blinks)
def blink_led(color=RED,period=1,n_blinks=3):
    for blink in range(n_blinks):
        leds.update(Leds.rgb_off())
        sleep(period/2)
        leds.update(Leds.rgb_on(color))
        sleep(period/2)
    leds.update(Leds.rgb_off())

def shut_aiy_kit_down():
    print('Terinating session...')
    blink_led(color=RED,period=0.5,n_blinks=10)
    sleep(0.5)
    GPIO.cleanup()
    sleep(3)
    os.system("sudo shutdown -h now") 

# Select the face with the largest width (others will be ignored)
def select_face(faces):
    if len(faces) > 0:
        max_width = 0
        for face in faces:
            width = face.bounding_box[2]
            if width > max_width:
                max_width = width
                face_selected = face
        return face_selected
    else:
        return None

def main():
    """Face detection camera inference example."""

    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--num_frames',
        '-n',
        type=int,
        dest='num_frames',
        default=-1,
        help='Sets the number of frames to run for, otherwise runs forever.')
    args = parser.parse_args()

    blink_led(color=BLUE,period=0.5,n_blinks=10)
    
    # Initialize variables for automatic shutdown
    shuttdown_flag = False
    min_frames_w_arduino_power = 10
    max_frames_w_no_arduino_power = 10
    counter_frames_w_power = 0
    counter_frames_w_no_power = 0

    # Focal length in pixels - found by calibration and geometry for 1640 x 1232 resolution) 
    focal_length = 1320
    camera_resolution = (1640, 1232)
    x_center = int(camera_resolution[0] / 2)
    real_face_width_inch = 11
    min_angle = atan2(-x_center,focal_length)
    max_angle = atan2(x_center,focal_length)
    min_distance = 20
    max_distance = 200
    face_detected_on_prev_frame = False  

    with PiCamera() as camera:
        # Forced sensor mode, 1640x1232, full FoV. See:
        # https://picamera.readthedocs.io/en/release-1.13/fov.html#sensor-modes
        # This is the resolution inference run on.
        camera.sensor_mode = 4

        # Scaled and cropped resolution. If different from sensor mode implied
        # resolution, inference results must be adjusted accordingly. This is
        # true in particular when camera.start_recording is used to record an
        # encoded h264 video stream as the Pi encoder can't encode all native
        # sensor resolutions, or a standard one like 1080p may be desired.
        camera.resolution = camera_resolution

        # Start the camera stream.
        camera.framerate = 30
        #camera.start_preview()

        # Calculate face data (x_mean, y_mean, width, height).
        def face_data(face):
            #face_score = int(1000. * face.face_score)
            #joy_score = int(1000. * face.joy_score)
            x, y, width, height = face.bounding_box
            x_mean = int(x + width/2)
            angle = atan2(x_mean - x_center,focal_length)
            distance = 0
            if width > 0:  
                distance = focal_length * real_face_width_inch / width
            return angle, distance 

        with CameraInference(face_detection.model()) as inference:
            for i, result in enumerate(inference.run()):
                if i == args.num_frames:
                    break
                faces = face_detection.get_faces(result)
                face = select_face(faces)
                if face:
                    if face_detected_on_prev_frame: 
                        angle, distance = face_data(face)
                        if angle < min_angle:
                            angle = min_angle
                        if angle > max_angle:
                            angle = max_angle
                        angle_to_send = (angle - min_angle)/(max_angle - min_angle)
                        pin_A.value = 0.8 * angle_to_send + 0.1
                        if distance < min_distance:
                            distance = min_distance
                        if distance > max_distance:
                            distance = max_distance
                        a = (0.9 - 0.2) / (max_distance - min_distance)
                        b = 0.9 - a * max_distance
                        pin_B.value = a * distance + b
                        leds.update(Leds.rgb_on(GREEN))
                    face_detected_on_prev_frame = True  
                else:
                    if not face_detected_on_prev_frame: 
                        pin_A.value = 0.5
                        pin_B.value = 0.1
                        leds.update(Leds.rgb_off())
                    face_detected_on_prev_frame = False  
                clock = i % 80 + 11
                pin_C.value = clock/100
                print('Iteration #', str(i), ' A=', str(pin_A.value), ' B=', str(pin_B.value), ' C=', str(pin_C.value), ' D=', str(pin_D.value))
                print('face_detected_on_prev_frame = ',face_detected_on_prev_frame) 

                if pin_D.is_pressed:
                    counter_frames_w_no_power += 1
                else:
                    counter_frames_w_power += 1
                print('released, shuttdown_flag = ',shuttdown_flag,' frames with power=',str(counter_frames_w_power))
                print('pressed,  shuttdown_flag = ',shuttdown_flag,' frames with no power=',str(counter_frames_w_no_power))
                if counter_frames_w_power >= min_frames_w_arduino_power and not shuttdown_flag:
                    shuttdown_flag = True
                    counter_frames_w_no_power = 0
                if counter_frames_w_no_power >= max_frames_w_no_arduino_power and shuttdown_flag:
                    shut_aiy_kit_down()
                sleep(0.1)

        #camera.stop_preview()

if __name__ == '__main__':
    main()
