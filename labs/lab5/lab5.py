"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Lab 4B - LIDAR Wall Following
"""

########################################################################################
# Imports
########################################################################################

import sys
import cv2 as cv
import numpy as np
import math
import statistics
import matplotlib.pyplot as plt
import matplotlib.axes as ax
from enum import IntEnum
import itertools 


sys.path.insert(0, "../../library")
import racecar_core
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# Add any global variables here
MIN_CONTOUR_AREA = 30

# A crop window for the floor directly in front of the car
CROP_FLOOR = ((180,0), (rc.camera.get_height(), rc.camera.get_width()))

# Colors, stored as a pair (hsv_min, hsv_max)
BLUE = ((90, 50, 50), (120, 255, 255))
RED = ((160, 50, 170),(179,255,255))
GREEN = ((40,50,163), (80,200,255))
ORANGE = ((0,100,100),(25,255,255))
YELLOW = ((20,30,30),(30,255,255))
PURPLE = ((120,120, 122),(150,200,200))
COLORS = (GREEN, RED, BLUE, YELLOW)

class State(IntEnum):
    LineFollow = 0
    RWallFollow = 1
    LWallFollow = 2
    ConeSlalom = 3
    SafetyStop = 4
cur_state: State = State.LineFollow

speed = 0.0  # The current speed of the car
angle = 0.0  # The current angle of the car's wheels
contour_center = None  # The (pixel row, pixel column) of contour
contour_area = 0  # The area of contour
integralSum = 0

########################################################################################
# Functions
########################################################################################

def update_contour():
    """
    Finds contours in the current color image and uses them to update contour_center
    and contour_area
    """
    global contour_center
    global contour_area

    image = rc.camera.get_color_image()

    if image is None:
        contour_center = None
        contour_area = 0
    else:
        # TODO (challenge 1): Search for multiple tape colors with a priority order
        # (currently we only search for blue)

        # Crop the image to the floor directly in front of the car/
        if cur_state == State.LineFollow:
            image = rc_utils.crop(image, CROP_FLOOR[0], CROP_FLOOR[1])

        #Find all of contours of desired color and get biggest contour
        for i in COLORS:         
            currentCont = rc_utils.find_contours(image,i[0], i[1] )

            # if largestCurrColorCont is None:
            #     continue
            contour= rc_utils.get_largest_contour(currentCont, MIN_CONTOUR_AREA)

            if contour is not None:
                # Calculate contour information
                contour_center = rc_utils.get_contour_center(contour)
                contour_area = rc_utils.get_contour_area(contour)
                print("area: ", contour_area)

                # Draw contour onto the image
                rc_utils.draw_contour(image, contour)
                rc_utils.draw_circle(image, contour_center)


                break

            else:
                contour_center = None
                contour_area = 0

            # Display the image to the screen
            rc.display.show_color_image(image)
            



def start():
    """
    This function is run once every time the start button is pressed
    """
    global speed
    global angle
    global angle_time
    global lastError 
    global cur_state
     

    # Initialize variables
    speed = 0
    angle = 0
    angle_time = 0
    lastError = 0

    # Set initial driving speed and angle
    rc.drive.set_speed_angle(speed, angle)
    cur_state = State.LineFollow
    # Print start message
    print(">> Race3 - Wall, Line, Slalom, Stop")

    
def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    global cur_state
    global speed
    global angle
    global angle_time
    global lastError
    global integralSum
    #     # Search for contours in the current color image
    update_contour()

    angle = 0
    error = 0

    color_image = rc.camera.get_color_image()
    markers = rc_utils.get_ar_markers(color_image)
    if len(markers) > 0:
        id = markers[0].get_id()
    else:
        id = 1000000 #ignore just a fake val

    scan = rc.lidar.get_samples_async()
    setpoint = 65
    speed = 0.122
    kp = .3
    print("id: ", id)
    
    if id == 4:
       # rc.drive.set_speed_angle(.3, .5)
        cur_state = State.RWallFollow
    elif id == 0:
        cur_state = State.RWallFollow
        if contour_center is not None and contour_area > 1000:
            cur_state = State.LineFollow
            
    elif id == 2:
        if markers[0].detect_color(color_image, COLORS) == "Blue":
            COLORS = (BLUE)
        elif markers[0].detect_color(color_image, COLORS) == "Red":
            COLORS = (RED)
        elif markers[0].detect_color(color_image, COLORS) == "Green":
            COLORS = (GREEN)
        cur_state = State.LineFollow
    elif id == 3:
        cur_state = State.SafetyStop
    elif id == 1:
        cur_state = State.ConeSlalom
#         print(markers[0].get_orientation())
#         if markers[0].get_orientation() == "left":
#             cur_state = State.LWallFollow
#         elif markers[0].get_orientation() == "right":
        #cur_state = State.RWallFollow 

    print("state: ", cur_state)

#     if rc_utils.get_lidar_closest_point(scan)[0] < 10 and rc_utils.get_lidar_closest_point(scan)[0] > 0:
#         cur_state = State.SafetyStop

    if cur_state == State.LWallFollow:
        error = rc_utils.get_lidar_average_distance(scan, 315, 35) - setpoint
        output = error*kp
        if rc_utils.get_lidar_average_distance(scan, 10, 10) < 60:
            pass
            # angle = 1
            # speed = .8
            # fix this
        if output > 0:
            print("neg")
            angle = 2*rc_utils.remap_range(output, 0, ((900-setpoint)*kp), 0, 1) 
        else:
            print("pos")
            angle = 2*rc_utils.remap_range(output, (-setpoint*kp), 0, -.1, 0) 
    elif cur_state == State.RWallFollow and scan is not None:
        #         if rc_utils.get_lidar_average_distance(scan, 10, 10) < 30:
#             angle = -.7
#             speed = .13
            # fix this
        error = rc_utils.get_lidar_average_distance(scan, 45, 35) - setpoint
        output = error*kp
        if output > 0:
            angle = 2*rc_utils.remap_range(output, 0, ((900-setpoint)*kp), 0, 1) 
        else:
            angle = 2*rc_utils.remap_range(output, (-setpoint*kp), 0, -.1, 0) 
    
    elif cur_state == State.LineFollow:
         if contour_center is not None:
            if lastError == None:
                lastError = 0   
            kp =.17
            ki = 0
            kd = .13
            dt = rc.get_delta_time()
            width = rc.camera.get_width()
            error = contour_center[1] / width * 2 - 1 # mapping it
            integralSum += (lastError + error) / 2 * dt
            angle = error*kp + integralSum*ki + ((error-lastError)/dt)*kd
            angle = rc_utils.clamp(angle, -1, 1)
            lastError = error
    elif cur_state == State.SafetyStop:
        print("stopped")
        rc.drive.set_speed_angle(-.5, 0)
        rc.drive.stop()
    elif cur_state == State.ConeSlalom:
        if coneColor == "orange":
            if contour_center is None:
                rc.drive.set_speed_angle(0.13, -0.11)
                print("autoOrangturn")
            else:
                TURN_ANGLE = ((contour_center[1] - 70) / 320)
                TURN_ANGLE = rc_utils.clamp(TURN_ANGLE, -1, 1)
                if -0.08 < TURN_ANGLE < 0.08:
                    TURN_ANGLE = 0
                rc.drive.set_speed_angle(0.18, TURN_ANGLE)
        elif coneColor == "purple":
            if contour_center is None:
                rc.drive.set_speed_angle(0.13, 0.11)
                print("autoPurpturn")
            else:
                TURN_ANGLE = ((contour_center[1] - 570) / 320)
                TURN_ANGLE = rc_utils.clamp(TURN_ANGLE, -1, 1)
                if -0.08 < TURN_ANGLE < 0.08:
                    TURN_ANGLE = 0
                rc.drive.set_speed_angle(0.18, TURN_ANGLE)
            
        

    rc.drive.set_speed_angle(speed, angle)
    print("ang: ", angle)
    print("error: ", error)
    # TODO: Follow the wall to the right of the car without hitting anything.


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()

   
