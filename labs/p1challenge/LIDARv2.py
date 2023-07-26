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

MIN_CONTOUR_AREA = 1500

coneColor = None

RED =((175, 160, 130), (179, 255, 250))
BLUE = ((90, 80, 80), (113, 255, 255))


clsPoint = None

CROP_FLOOR = ((100,0), (rc.camera.get_height(), rc.camera.get_width()))

coneFront = True

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
        
        if contour is not None:
            print(rc_utils.get_contour_area(contour))

def angleMapper(angDeg):
    if angDeg > 270:
        angle = (angDeg - 360)/90
    elif angDeg < 90:
        angle = (angDeg)/90
    elif angDeg >= 90 and angDeg < 180:
        angle = (180 - angDeg)/90
    elif angDeg >= 180 and angDeg < 270:
        angle = (angDeg - 180)/90
    return angle


def blueCurve(): 
    global clsPoint

    speed = 0.3
    shiftValue = 40

    angDeg = clsPoint[0]
    if (angDeg - shiftValue) > 360:
        turnAngDeg = (angDeg - shiftValue)%360
    elif (angDeg - shiftValue) < 0:
        turnAngDeg = (angDeg - shiftValue) + 360
    else:
        turnAngDeg = (clsPoint[0] - shiftValue)

    print(angDeg)
    if angDeg > 270 or angDeg < 90:
        angle = angleMapper(turnAngDeg)
    elif angDeg >= 90 and angDeg <= 180:
        #this one is weird, want it to go straight and it goes s
        angle = 0.2
    
    rc.drive.set_speed_angle(speed,angle)




def redCurve(): 
    global clsPoint

    speed = 0.3
    shiftValue = 40

    angDeg = clsPoint[0]
    if (angDeg + shiftValue) > 360:
        turnAngDeg = (angDeg + shiftValue)%360
    elif (angDeg + shiftValue) < 0:
        turnAngDeg = (angDeg + shiftValue) + 360
    else:
        turnAngDeg = (clsPoint[0] + shiftValue)

    print(angDeg)
    if angDeg > 270 or angDeg < 90:
        angle = angleMapper(turnAngDeg)
    elif angDeg <= 270 and angDeg >= 180:
        #this one is weird, want it to go straight and it goes s
        angle = -0.2
    
    rc.drive.set_speed_angle(speed,angle)
   

def update():
    
    global cur_state
    global coneColor
    global clsPoint
    global coneFront

    #Getting the scan
    scan = rc.lidar.get_samples_async()

    #Displaying the scan
    rc.display.show_lidar(scan)

    update_color()
    print(cur_state)

    print(coneFront)
    #Getting the closest Cone in front pf car
    clsPoint = rc_utils.get_lidar_closest_point(scan, (0, 360))

    if clsPoint[0] >= 90 and clsPoint[0] <= 270 and clsPoint[1] > 37:
        clsPoint = rc_utils.get_lidar_closest_point(scan, (270, 90))

    #Calling States
    if coneColor == "red":
        cur_state = State.redCurve
    elif coneColor == "blue":
        cur_state = State.blueCurve
    

    if cur_state == State.redCurve:
        redCurve()
    elif cur_state == State.blueCurve:
        blueCurve()
    elif cur_state == State.Search:
        rc.drive.set_speed_angle(0.3,0)


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update)
    rc.go()
