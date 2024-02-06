# Author: Kartezyen Corp.
# Topic: Axis Control Software
# Date: June 6, 2023

import RPi.GPIO as GPIO  # Import GPIO library
import time  # Import time library
import threading
import torch
import os
import cv2
import playsound
from picamera2 import Picamera2, Preview

# Model
model = torch.hub.load(r"/usr/local/lib/python3.9/dist-packages/yolov5", 'custom',
                       path=r"/usr/local/lib/python3.9/dist-packages/yolov5/best.pt", source='local', force_reload=True, autoshape=True)

model.conf = 0.10



# Camera driver
picam2 = Picamera2()
preview_config = picam2.create_preview_configuration(main={"size": (800, 600)})
picam2.configure(preview_config)
picam2.start_preview()
picam2.start()

# Global Variables
sound_flag = 0
priority_sound_flag = 0
redirection_flag = 0
obstacle_detection_flag = 0
state = 0
lock = threading.Lock()

CAR_INDEX = 0
GREENLIGHT_INDEX = 2
GREENLIGHT_CARTON_INDEX = 1
CROSSWALK_CARTON_INDEX = 4
CROSSWALK_INDEX = 5
PERSON_INDEX = 6
REDLIGHT_INDEX = 8
REDLIGHT_CARTON_INDEX = 7

REVERSE_STATE = -1
KEEP_STATE = 0
ADVANCE_STATE = 1

# Define sound categories with numeric codes
NO_CW_DETECT = 1
CW_DETECT = 2
LITTLE_LEFT = 3
LITTLE_RIGHT = 4
CHECK_CW = 5
RED = 6
GREEN_CAR = 7
OK_PASS = 8
COMPLETE = 9
CW_NOT_WALKING_DIR = 10

cw_counter = 0

# Functions
# Audio
def play_audio_warning():

    global sound_flag
    global priority_sound_flag
    
    
    
    while True:
        print("Soundflag: ", sound_flag)
        
        if (priority_sound_flag == 0):
            # Object is about to collide move to right
            if (sound_flag == 1):
                playsound.playsound('/home/kartezyen/Desktop/audio/move_right.mp3', True)

            # Object is about to collide move to left
            elif (sound_flag == 2):
                playsound.playsound('/home/kartezyen/Desktop/audio/move_left.mp3', True)

            # Object is ahead.
            elif (sound_flag == 3):
                playsound.playsound('/home/kartezyen/Desktop/audio/object_ahead.mp3', True)

            sound_flag = 0
        else:
                        
            if (priority_sound_flag == NO_CW_DETECT):
                playsound.playsound('/home/kartezyen/Desktop/audio/no_cwdetect.mp3', True)
            elif(priority_sound_flag == CW_DETECT):
                playsound.playsound('/home/kartezyen/Desktop/audio/cw_detect.mp3', True)
            elif(priority_sound_flag == LITTLE_LEFT):
                playsound.playsound('/home/kartezyen/Desktop/audio/little_left.mp3', True)
            elif(priority_sound_flag == LITTLE_RIGHT):
                playsound.playsound('/home/kartezyen/Desktop/audio/little_right.mp3', True)
            elif(priority_sound_flag == CHECK_CW):
                playsound.playsound('/home/kartezyen/Desktop/audio/check_cw.mp3', True)
            elif(priority_sound_flag == RED):
                playsound.playsound('/home/kartezyen/Desktop/audio/red.mp3', True)
            elif(priority_sound_flag == GREEN_CAR):
                playsound.playsound('/home/kartezyen/Desktop/audio/green_car.mp3', True)
            elif(priority_sound_flag == OK_PASS):
                playsound.playsound('/home/kartezyen/Desktop/audio/ok_pass.mp3', True)
            elif(priority_sound_flag == COMPLETE):
                playsound.playsound('/home/kartezyen/Desktop/audio/complete.mp3', True)
            elif(priority_sound_flag == CW_NOT_WALKING_DIR):
                playsound.playsound('/home/kartezyen/Desktop/audio/cwnotwalingdir.mp3', True)

            priority_sound_flag = 0
            
        time.sleep(0.5)

# Redirection 
def adjust_redirection(direction):
    global redirection_flag
    redirection_flag = direction

# Detection Parser
def parse_detection (classes, xmin, xmax, ymin, ymax):

    global state   # Add this line
    global cw_counter   # Add this line
    global priority_sound_flag
    
    n1 = len(classes)
    values = range(n1)

    # Check for crosswalk.
    if (state == 0):
        cw_detected = False
        for i in values:
        # Crosswalk is detected.
            if (classes[i] == CROSSWALK_INDEX or classes[i] == CROSSWALK_CARTON_INDEX):
                cw_detected = True
                
                # Calculate crosswalk orientation
                width = xmax[i] - xmin[i]
                height = ymax[i] - ymin[i]
                if width > height:
                    orientation = "Horizontal"
                else:
                    orientation = "Vertical"
                
                # If the crosswalk is horizontal, don't proceed further.
                if orientation == "Horizontal":
                    priority_sound_flag = CW_NOT_WALKING_DIR
                    return KEEP_STATE
                break

        if (cw_detected == True):
            # Play audio for detected crosswalk and signal completion of state.  
            #playsound.playsound('/home/kartezyen/Desktop/audio/cw_detect.mp3', True)
            priority_sound_flag = CW_DETECT
            return ADVANCE_STATE
        else:
            # Play audio for missing crosswalk.
            # playsound.playsound('/home/kartezyen/Desktop/audio/no_cwdetect.mp3', True)
            priority_sound_flag = NO_CW_DETECT
            return KEEP_STATE
            
    # Adjust the user's position wrt the crosswalk.
    elif (state == 1):
        cw_detected = False
        detection_index = 0

        for i in values:
            #Crosswalk is detected.
            if (classes[i] == CROSSWALK_INDEX or classes[i] == CROSSWALK_CARTON_INDEX):
                cw_detected = True
                detection_index = i
                break

        if (cw_detected):
            # Find the center of the crosswalk's rectangle.
            center = (xmin[detection_index] + xmax[detection_index]) / 2
            print("center:", center)
            # Check the center with the screen's dimensions.
            # Crosswalk is on the left of the user.
            if (center <= 1200):
                # Play the sound to move the user to the left. And check again.
                #playsound.playsound('/home/kartezyen/Desktop/audio/little_left.mp3', True)
                priority_sound_flag = LITTLE_LEFT
                return KEEP_STATE
            
            # Crosswalk is on the right of the user.
            elif (center >= 1400):
                # Play the sound to move the user to the right. And check again.
                #playsound.playsound('/home/kartezyen/Desktop/audio/little_right.mp3', True)
                priority_sound_flag = LITTLE_RIGHT
                return KEEP_STATE
            
            # Crosswalk is in the correct position. 
            else:
                # Play the sound, signaling crosswalk is present and checking light and cars.
                #playsound.playsound('/home/kartezyen/Desktop/audio/check_cw.mp3', True)
                priority_sound_flag = CHECK_CW
                return ADVANCE_STATE

            
        #else:
            #return REVERSE_STATE
    
    # Check for traffic light and cars. 
    elif (state == 2):
        cw_detected= False
        gl_detected = False
        rl_detected = False
        car_detected = False

        car_on_path = False
        # Get crosswalk coordinates.
        for i in values:
            if (classes[i] == CROSSWALK_INDEX or classes[i] == CROSSWALK_CARTON_INDEX):
                cw_xmin = xmin[i]
                cw_xmax = xmax[i]
                cw_detected = True

            if (classes[i] == GREENLIGHT_INDEX or classes[i] == GREENLIGHT_CARTON_INDEX):
                gl_detected = True

            if (classes[i] == CAR_INDEX):
                car_xmin = xmin[i]
                car_xmax = xmax[i]
                car_detected = True

            if (classes[i] == REDLIGHT_INDEX or classes[i] == REDLIGHT_CARTON_INDEX):
                rl_detected = True

        if (gl_detected and car_detected):
            if ((car_xmax >= cw_xmin + 20) or (car_xmin <= cw_xmax - 20)):
                car_on_path = True

        if (rl_detected):
            # Play red light warning.
            # playsound.playsound('/home/kartezyen/Desktop/audio/red.mp3', True)
            priority_sound_flag = RED
            return KEEP_STATE
        
        if (gl_detected and car_on_path):
            # Play the audio for warning.
            # playsound.playsound('/home/kartezyen/Desktop/audio/green_car.mp3', True)
            priority_sound_flag = GREEN_CAR
            return KEEP_STATE
        elif (gl_detected):
            # No overlap between cars and crosswalk.
            # Play audio, telling to the user to start passing.
            # playsound.playsound('/home/kartezyen/Desktop/audio/ok_pass.mp3', True)
            priority_sound_flag = OK_PASS
            return ADVANCE_STATE
    
    # Crossing is started. Check for crosswalk position constantly and adjust obstacle detection flag wrt it. 
    elif (state == 3):
        cw_detected = False
        cw_center= 0

        global cw_counter
        # Get crosswalk coordinates.
        for i in values:
            if (classes[i] == CROSSWALK_INDEX or classes[i] == CROSSWALK_CARTON_INDEX):
                cw_detected = True
                cw_xmin = xmin[i]
                cw_xmax = xmax[i]
                cw_center = (cw_xmin + cw_xmax) / 2

                cw_counter = 0

        # No crosswalk is detected. If this happens for 5 consecutive times, assume the passing is done.
        if (cw_detected == False):
            if (cw_counter != 5):
                cw_counter += 1
                
        # If no crosswalk is detected for 5 times, warn the user about the passing is over and revert to the zeroth state.
        if (cw_counter >= 5):

            # Play audio and advance to the beginning.
            # playsound.playsound('/home/kartezyen/Desktop/audio/complete.mp3', True)
            priority_sound_flag = COMPLETE
            cw_counter = 0
            return ADVANCE_STATE
        
        # Check the coordinates and set obstacle detection flag.
        # Move left when an obstacle is in way.
        if (cw_detected):
            if (cw_center <= 1496):
                adjust_redirection("Left")
            # Move right when an obstacle is in way.
            else:
                adjust_redirection("Right")

# Image Recognition
def imageRecognition_callback(x):
    
    global obstacle_detection_flag
    global state
    global sound_flag
    
    # Prevent multiple button clicks to go through while image recognition is going on.
    if lock.acquire(False):

        while True:
            

            picam2.capture_file("test.jpg")
            # Inference
            print("before")
            results = model("test.jpg")   
            print("after")        	
            results.xyxy[0]
            coordinates = results.pandas().xyxy[0] 

            print(coordinates)

            s1 = coordinates['class']
            s2 = coordinates['xmin']
            s3 = coordinates['xmax']
            s4 = coordinates['ymin']
            s5 = coordinates['ymax']
            s_avg = (s1 + s2)/2  # Finding the center of the rectangle
            #print(s_avg)

            advance_flag = parse_detection(s1, s2, s3, s4,s5)
            # If state is completed, advance to the next state or if there is a problem go back to the previous state.
            if (advance_flag == 1):
                if (state == 3):
                    state = 0
                else:
                    state += 1
            elif (advance_flag == -1):
                if (state == 0):
                    state = 3
                else:
                    state -= 1
            
            if ( state == 3 ):
                obstacle_detection_flag = 1
            else:
                obstacle_detection_flag = 0
                sound_flag = 0

            results.pandas().xyxy[0].value_counts('name')

            s4 = coordinates['name']

            # result = results.save()
            # results.show()
            # time.sleep(1)
            # os.system ("rm result.jpg")

            # Couldn't find a crosswalk, therefore requires additional button press to search again.
            #if (state == 0):
             #   break
            # ---

            # Release the lock for the next button click.
    lock.release()

# Hardware Initializations
GPIO.setmode(GPIO.BCM)  # Set GPIO pin numbering

TRIG = 15  # Associate pin 15 to TRIG
ECHO = 14  # Associate pin 14 to Echo
BUTTON = 23  # Associate pin 23 to Button

GPIO.setup(TRIG, GPIO.OUT)  # Set pin as GPIO out
GPIO.setup(ECHO, GPIO.IN)  # Set pin as GPIO in
# Set pin as input for button
GPIO.setup(BUTTON, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Activate the interrupt
GPIO.add_event_detect(BUTTON, GPIO.FALLING,
                      callback=imageRecognition_callback, bouncetime=200)

# Activate sound thread
soundThread = threading.Thread(
    target=play_audio_warning, name="Audio Thread")
soundThread.daemon = True
soundThread.start()

length = 3
measurements = [0] * length

playsound.playsound('/home/kartezyen/Desktop/audio/starting.mp3', True)

#Main thread
while True:
    if( obstacle_detection_flag == 1 ):
        for ii in range(3):
            GPIO.output(TRIG, False)  # Set TRIG as LOW
            # print("Waiting For Sensor To Settle")
            time.sleep(0.25)  # Delay of 2 seconds

            GPIO.output(TRIG, True)  # Set TRIG as HIGH
            time.sleep(0.00001)  # Delay of 0.00001 seconds
            GPIO.output(TRIG, False)  # Set TRIG as LOW

            while GPIO.input(ECHO) == 0:  # Check if Echo is LOW
                pulse_start = time.time()  # Time of the last  LOW pulse

            while GPIO.input(ECHO) == 1:  # Check whether Echo is HIGH
                pulse_end = time.time()  # Time of the last HIGH pulse

            pulse_duration = pulse_end - pulse_start  # pulse duration

            distance = pulse_duration * 17150  # Calculate distance
            distance = round(distance, 2)
            
            if (distance > 200):
                ii -= 1
                continue;
            
            measurements[ii] = distance
            print("Measurements:",measurements[ii])

        dif_1 = abs( measurements[1] - measurements[0])
        dif_2 = abs( measurements[2] - measurements[1])
        
        print("Dif1:",dif_1)
        print("Dif2:",dif_2)
        
        if dif_1 < 85 and dif_2 < 85:
            distance = measurements[2]
                
            if distance > 80 and distance < 200:  # Is distance within the range
                distance = distance - 0.5
                print("Distance:", distance, "cm")

                # About to collide
                if distance < 150:
                    if (redirection_flag == "Right"):
                        sound_flag = 1
                    elif (redirection_flag == "Left"):
                        sound_flag = 2
                    
                # An object is ahead
                elif distance > 150 or distance < 200:
                    sound_flag = 3

                # Nothing in specified range
                else:
                    sound_flag = 0
        else:
            print("Out Of Range")  # display out of range
            sound_flag = 0
                
        measurements = [0] * length
time.sleep(0.1)
picam2.close()
