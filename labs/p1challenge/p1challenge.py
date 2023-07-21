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

coneColor = None

ORANGE =((175, 160, 130), (179, 255, 250))
PURPLE = ((90, 80, 80), (113, 255, 255))

contour_center = None  # The (pixel row, pixel column) of contour
contour_area = 0  # The area of contour

CROP_FLOOR = ((100,0), (rc.camera.get_height(), rc.camera.get_width()))

speed = 0
angle = 0

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

        # Find all of the orange and purple contours
        orangeContours = rc_utils.find_contours(image, ORANGE[0], ORANGE[1])      
        purpleContours = rc_utils.find_contours(image, PURPLE[0], PURPLE[1])

        # Select the largest orange and purple contour
        orangeContour = rc_utils.get_largest_contour(orangeContours, MIN_CONTOUR_AREA)
        purpleContour = rc_utils.get_largest_contour(purpleContours, MIN_CONTOUR_AREA)

        if orangeContour is None and purpleContour is None:
            contour = None
            coneColor = "None"
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

def purpleCurve():
    global queue
    if len(queue) < 1:
        queue.append([2.8,0.2,-0.3])
        queue.append([2.8,0.2,0.6])

def orangeCurve(): 
    global queue
    if len(queue) < 1:
        queue.append([2.8,0.2,0.3])
        queue.append([2.8,0.2,-0.6])
    

def update():
    
    global cur_state
    global queue
    global coneColor
    global speed
    global angle

    update_contour()
    update_slow()
    print(cur_state)

    if coneColor == "orange":
        cur_state = State.orangeCurve
    elif coneColor == "purple":
        cur_state = State.purpleCurve
    
    if cur_state == State.orangeCurve:
        orangeCurve()
    elif cur_state == State.purpleCurve:
        purpleCurve()

    if len(queue) > 0:
        speed = queue[0][1]
        angle = queue[0][2]
        queue[0][0] -= rc.get_delta_time()
        rc.drive.set_speed_angle(speed, angle)
        if queue[0][0] <= 0:
            queue.pop(0)
    
    
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
