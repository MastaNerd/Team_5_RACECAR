"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Phase 1 Challenge - Cone Slaloming
"""

########################################################################################
# Imports
########################################################################################

# Import Python libraries
import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt
import ipywidgets as widgets
import statistics
from nptyping import NDArray
from typing import Any, Tuple, List, Optional

# Import Racecar library
import sys
sys.path.append("../../library")
import racecar_core
import racecar_utils as rc_utils
from enum import IntEnum

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

MIN_CONTOUR_AREA = 50

coneColor = None

RED =((175, 160, 130), (179, 255, 250))
BLUE = ((90, 80, 80), (113, 255, 255))


clsPoint = None

CROP_FLOOR = ((100,0), (rc.camera.get_height(), rc.camera.get_width()))



# Add any global variables here
########################################################################################
# States
########################################################################################
class State(IntEnum):
    Search = 0
    redCurve = 1
    blueCurve = 2
cur_state: State = State.Search
########################################################################################
# Functions
########################################################################################


def start():
    """
    This function is run once every time the start button is pressed
    """
    global cur_state
    # Initialize variables

    cur_state = State.Search

    # Set initial driving speed and angle
    #rc.drive.set_speed_angle(speed, angle)

    # Set update_slow to refresh every half second
    rc.set_update_slow_time(0.5)
    # Have the car begin at a stop
    
    # Print start message
    print(">> Phase 1 Challenge: Cone Slaloming")

def update_color():
    """
    Finds contours in the current color image and uses them to update contour_center
    and contour_area
    """
    global cur_state
    global coneColor
    

    image = rc.camera.get_color_image()

    if image is None:
        contour_center = None
        contour_area = 0
    else:

        image = rc_utils.crop(image, CROP_FLOOR[0], CROP_FLOOR[1])

        # Find all of the red and blue contours
        redContours = rc_utils.find_contours(image, RED[0], RED[1])      
        blueContours = rc_utils.find_contours(image, BLUE[0], BLUE[1])

        # Select the largest red and blue contour
        redContour = rc_utils.get_largest_contour(redContours, MIN_CONTOUR_AREA)
        blueContour = rc_utils.get_largest_contour(blueContours, MIN_CONTOUR_AREA)

        if redContour is None and blueContour is None:
            contour = None
            coneColor = "None"
        elif redContour is None:
            contour = blueContour
            coneColor = "blue"
        elif blueContour is None:
            contour = redContour
            coneColor = "red"
        else:
            if rc_utils.get_contour_area(redContour) > rc_utils.get_contour_area(blueContour):
                contour = redContour
                coneColor = "red"
            else:
                contour = blueContour
                coneColor = "blue"


def blueCurve():
    global clsPoint
    #setting speed
    speed = 0.2
    #getting turn angle in degrees
    angleInDegrees = (clsPoint[0] - 40)%360
    print(angleInDegrees)

    #seeing what value the turn angle corresponds to
    if angleInDegrees > 270:
        angle = (angleInDegrees - 360)/90
    elif angleInDegrees < 90:
        angle = (angleInDegrees)/90
    elif angleInDegrees >= 90 and angleInDegrees < 180:
        angle = (180 - angleInDegrees)/90
    elif angleInDegrees >= 180 and angleInDegrees < 270:
        angle = -(angleInDegrees - 180)/90
    print(angle)
    rc.drive.set_speed_angle(speed,angle)

def redCurve(): 
    global clsPoint

    #setting speed
    speed = 0.2

    #getting turn angle in degrees
    angleInDegrees = (clsPoint[0] + 40)%360
    print(angleInDegrees)

    #seeing what value the turn angle corresponds to
    if angleInDegrees > 270:
        angle = (angleInDegrees - 360)/90
    elif angleInDegrees < 90:
        angle = (angleInDegrees)/90
    elif angleInDegrees >= 90 and angleInDegrees < 180:
        angle = (180 - angleInDegrees)/90
    elif angleInDegrees >= 180 and angleInDegrees < 270:
        angle = -(angleInDegrees - 180)/90

    #exeucteing drive
    print(angle)
    rc.drive.set_speed_angle(speed,angle)

def update():
    
    global cur_state
    global queue
    global coneColor
    global clsPoint

    #Getting the scan
    scan = rc.lidar.get_samples_async()

    #Displaying the scan
    rc.display.show_lidar(scan)

    #Getting the closest Cone
    clsPoint = rc_utils.get_lidar_closest_point(scan, (0, 360))

    #Sees if that cone is in front of it, and only updates color if it is
    if clsPoint[0] > 340 or clsPoint[0] < 20:
        update_color()

    #Calling States
    if coneColor == "red":
        cur_state = State.redCurve
        print(cur_state)
    elif coneColor == "blue":
        cur_state = State.blueCurve
        print(cur_state)
    
    if cur_state == State.redCurve:
        redCurve()
    elif cur_state == State.blueCurve:
        blueCurve()


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update)
    rc.go()
