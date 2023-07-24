"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Lab 3C - Depth Camera Wall Parking
"""

########################################################################################
# Imports
########################################################################################

# Import Python libraries
import math
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

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# Add any global variables here

MIN_CONTOUR_AREA = 30

global queue
queue=[]

coneColor = None

RED =((175, 160, 130), (179, 255, 250))
BLUE = ((90, 80, 80), (113, 255, 255))

########################################################################################
# Functions
########################################################################################


def start():
    """
    This function is run once every time the start button is pressed
    """
    # Have the car begin at a stop
    rc.drive.stop()

    # Print start message
    print(">> Lab 3C - Depth Camera Wall Parking")

    

def update():
    #global variables
    global coneColor
    #Getting the scan
    scan = rc.lidar.get_samples()

    #Displaying the scan
    rc.lidar.show_lidar(scan)

    clsPoint = rc.lidar.get_lidar_closest_point(scan, (0, 360))

    if clsPoint[0] < 100 and clsPoint > 150:
        image = rc.camera.get_color_image()

        if image is None:
            coneColor = "None"
        else:

            # Find all of the orange and purple contours
            orangeContours = rc_utils.find_contours(image, RED[0], RED[1])      
            purpleContours = rc_utils.find_contours(image, BLUE[0], BLUE[1])

            # Select the largest orange and purple contour
            orangeContour = rc_utils.get_largest_contour(orangeContours, MIN_CONTOUR_AREA)
            purpleContour = rc_utils.get_largest_contour(purpleContours, MIN_CONTOUR_AREA)
        # check which cone is closer
        if orangeContour is None and purpleContour is None:
            coneColor = "None"
        elif orangeContour is None:
            coneColor = "purple"
        elif purpleContour is None:
            coneColor = "orange"
        else:
            if rc_utils.get_contour_area(orangeContour) > rc_utils.get_contour_area(purpleContour):
                coneColor = "orange"
            else:
                coneColor = "purple"
        rc.lidar.get_lidar_average_distance(scan, 360, 20)






########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()
