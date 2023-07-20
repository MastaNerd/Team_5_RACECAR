"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Phase 1 Challenge - Cone Slaloming
"""

########################################################################################
# Imports
########################################################################################

import sys
import cv2 as cv
import numpy as np
import math

sys.path.insert(0, "../../library")
import racecar_core
import racecar_utils as rc_utils
from enum import IntEnum

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

MIN_CONTOUR_AREA = 30

global queue
queue=[]

CROP_FLOOR = ((180,0), (rc.camera.get_height(), rc.camera.get_width()))

coneColor = None

GREEN = ((50,200,174), (90,255,214))
RED =((175, 160, 130), (179, 255, 255))#actually orange
YELLOW = ((30,102,255),(30,255,153))
BLUE = ((90, 80, 80), (113, 255, 255))#actually purple

contour_center = None  # The (pixel row, pixel column) of contour
contour_area = 0  # The area of contour

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
    global speed
    global angle
    global cur_state
    global queue

    # Initialize variables
    speed = 0
    angle = 0

    cur_state = State.Search

    queue.clear()
    # Set initial driving speed and angle
    #rc.drive.set_speed_angle(speed, angle)

    # Set update_slow to refresh every half second
    rc.set_update_slow_time(0.5)
    # Have the car begin at a stop
    
    # Print start message
    print(">> Phase 1 Challenge: Cone Slaloming")

def update_contour():
    """
    Finds contours in the current color image and uses them to update contour_center
    and contour_area
    """
    global contour_center
    global contour_area
    global cur_state
    global coneColor
    global contour_center
    

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
            # Calculate contour information
            contour_center = rc_utils.get_contour_center(contour)
            contour_area = rc_utils.get_contour_area(contour)

            # Draw contour onto the image
            rc_utils.draw_contour(image, contour)
            rc_utils.draw_circle(image, contour_center)

        else:
            contour_center = None
            contour_area = 0
        # Display the image to the screen
        rc.display.show_color_image(image)

def blueCurve(contour_center, contour_area):

    if contour_area < 2500:
        rc.drive.set_speed_angle(0.2, -0.2)
    elif contour_center is None:
        rc.drive.set_speed_angle(0.2, 0.2)
    else:
        TURN_ANGLE = ((contour_center[1] - 590)/320)
        if TURN_ANGLE > 1:
            TURN_ANGLE = 1
        elif TURN_ANGLE < -1:
            TURN_ANGLE = -1
        rc.drive.set_speed_angle(0.2, TURN_ANGLE)

def redCurve(contour_center, contour_area): 

    if contour_area < 2500:
        rc.drive.set_speed_angle(0.2, 0.2)
    elif contour_center is None:
        rc.drive.set_speed_angle(0.2, -0.2)
    else:
        TURN_ANGLE = ((contour_center[1] - 50)/320)
        if TURN_ANGLE > 1:
            TURN_ANGLE = 1
        elif TURN_ANGLE < -1:
            TURN_ANGLE = -1
        rc.drive.set_speed_angle(0.2, TURN_ANGLE)

def update():
    
    global cur_state
    global queue
    global coneColor
    global contour_center
    global contour_area
    global queue

    update_contour()
    update_slow()

    if coneColor == "red":
        cur_state = State.redCurve
    elif coneColor == "blue":
        cur_state = State.blueCurve
    
    if cur_state == State.redCurve:
        print("red")
        redCurve(contour_center, contour_area)
    elif cur_state == State.blueCurve:
        print("blue")
        blueCurve(contour_center, contour_area)
    
    
def update_slow():
    """
    After start() is run, this function is run at a constant rate that is slower
    than update().  By default, update_slow() is run once per second
    """
    # Print a line of ascii text denoting the contour area and x position
    if rc.camera.get_color_image() is None:
        # If no image is found, print all X's and don't display an image
        print("X" * 10 + " (No image) " + "X" * 10)
    else:
        # If an image is found but no contour is found, print all dashes
        if contour_center is None:
            print("-" * 32 + " : area = " + str(contour_area))

        # Otherwise, print a line of dashes with a | indicating the contour x-position
        else:
            s = ["-"] * 32
            s[int(contour_center[1] / 20)] = "|"
            print("".join(s) + " : area = " + str(contour_area))


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()
