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

MIN_CONTOUR_AREA = 600

coneColor = None

ORANGE = ((0,160,160),(16,255,255))
PURPLE = ((127, 83, 100), (160, 255, 255))

contour_center = None  # The (pixel row, pixel column) of contour
contour_area = 0  # The area of contour

CROP_FLOOR = ((100,0), (rc.camera.get_height(), rc.camera.get_width()))


# Add any global variables here
########################################################################################
# States
########################################################################################
class State(IntEnum):
    Search = 0
    orangeCurve = 1
    purpleCurve = 2
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

    # Initialize variables
    speed = 0
    angle = 0

    cur_state = State.Search

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

        # Find all of the orange and purple contours
        orangeContours = rc_utils.find_contours(image, ORANGE[0], ORANGE[1])      
        purpleContours = rc_utils.find_contours(image, PURPLE[0], PURPLE[1])

        # Select the largest orange and purple contour
        orangeContour = rc_utils.get_largest_contour(orangeContours, MIN_CONTOUR_AREA)
        purpleContour = rc_utils.get_largest_contour(purpleContours, MIN_CONTOUR_AREA)
        

        if orangeContour is None and purpleContour is None:
            contour = None
        elif orangeContour is None:
            contour = purpleContour
            coneColor = "purple"
        elif purpleContour is None:
            contour = orangeContour
            coneColor = "orange"
        else:
            if rc_utils.get_contour_area(orangeContour) > rc_utils.get_contour_area(purpleContour):
                contour = orangeContour
                coneColor = "orange"
            else:
                contour = purpleContour
                coneColor = "purple"
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

def purpleCurve(contour_center, contour_area):
    if contour_center is None:
        rc.drive.set_speed_angle(0.135, 0.12)
        print("autoPurpturn")
    else:
        TURN_ANGLE = ((contour_center[1] - 570) / 320) - 0.08
        TURN_ANGLE = rc_utils.clamp(TURN_ANGLE, -1, 1)
        if -0.08 < TURN_ANGLE < 0.08:
            TURN_ANGLE = 0
        rc.drive.set_speed_angle(0.17, TURN_ANGLE)

def orangeCurve(contour_center, contour_area):
    if contour_center is None:
        rc.drive.set_speed_angle(0.135, -0.12)
        print("autoOrangturn")
    else:
        TURN_ANGLE = ((contour_center[1] - 70) / 320) + 0.08
        TURN_ANGLE = rc_utils.clamp(TURN_ANGLE, -1, 1)
        if -0.08 < TURN_ANGLE < 0.08:
            TURN_ANGLE = 0
        rc.drive.set_speed_angle(0.17, TURN_ANGLE)

def update():
    
    global cur_state
    global coneColor
    global contour_center
    global contour_area

    update_contour()
    update_slow()
    print(cur_state)

    if coneColor == "orange":
        cur_state = State.orangeCurve
    elif coneColor == "purple":
        cur_state = State.purpleCurve
    
    if cur_state == State.orangeCurve:
        orangeCurve(contour_center, contour_area)
    elif cur_state == State.purpleCurve:
        purpleCurve(contour_center, contour_area)
    elif cur_state == State.Search:
        rc.drive.set_speed_angle(0.16, 0)
    
    
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
