"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

GRAND PRIX
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
MIN_CONTOUR_AREA = 1500

# A crop window for the floor directly in front of the car
CROP_FLOOR = ((180,0), (rc.camera.get_height(), rc.camera.get_width()))

# Colors, stored as a pair (hsv_min, hsv_max)
BLUE = ((90, 50, 50), (120, 255, 255))
RED = ((0,116, 113),(0,159,214))
GREEN = ((40,120,120), (100,255,255))
ORANGE = ((0,128,128),(25,255,255))
YELLOW = ((20,30,30),(30,255,255))
PURPLE = ((120,230, 230),(150,255,255))
COLORS = (GREEN, BLUE, YELLOW)
LANECOLORS = (PURPLE, ORANGE)

#states correlate w id num ish
class State(IntEnum):
    SafetyStop = 0
    Ramp = 1
    LineFollow = 2
    WallFollow = 3
    FastWallFollow = 4
    ConeSlalom = 5
    LaneFollow = 6
    WallFollowHazards = 7
    WallFollowBrick = 8
    # ???
    # HazardG = 7
    # HazardR = 8
    # HazardY = 9
    # HazardB = 10
    RWallFollow = 9
    LWallFollow = 10

cur_state: State = State.Ramp

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

                    # Draw contour onto the image
                    rc_utils.draw_contour(image, contour)
                    rc_utils.draw_circle(image, contour_center)


                    break

                else:
                    contour_center = None
                    contour_area = 0

        if cur_state == State.LaneFollow:
            rImage = rc_utils.crop(image, (180,320), (rc.camera.get_height(), rc.camera.get_width()))
            lImage = rc_utils.crop(image, (180,0), (rc.camera.get_height(), 320))
            
            for i in LANECOLORS:         
                rCont = rc_utils.find_contours(rImage, i[0], i[1])
                lCont = rc_utils.find_contours(lImage, i[0], i[1])

             
                rLargCont= rc_utils.get_largest_contour(rCont, 300)
                lLargCont= rc_utils.get_largest_contour(lCont, 300)
                if rLargCont is not None and lLargCont is not None:
                    rcontcenter = rc_utils.get_contour_center(rLargCont)
                    lcontcenter = rc_utils.get_contour_center(lLargCont)
               
                
                    contour_center = ((rcontcenter[0] + lcontcenter[0])/2, (rcontcenter[1] + lcontcenter[1] + 320)/2)
                    print("center is at" + str(contour_center[1]))

                    break
                elif rLargCont is not None:
                    print("right contour detected, ", rLargCont)
                elif lLargCont is not None:
                    print("left contour detected, ", lLargCont)
                else:
                    contour_center = None
            

def start():
    """
    This function is run once every time the start button is pressed
    """
    global speed
    global angle
    global angle_time
    global lastError 
    global cur_state
    global integralSum
    global time
    time = 0

    # Initialize variables
    speed = 0
    angle = 0
    angle_time = 0
    lastError = 0
    integralSum = 0

    # Set initial driving speed and angle
    rc.drive.set_speed_angle(speed, angle)
    cur_state = State.Ramp
    # Print start message
    print(">> GRAND PRIX")

def pid(kp, ki, kd, error, lastError, dt):
    global integralSum
    width = rc.camera.get_width()
    error = contour_center[1] / width * 2 - 1 # mapping it
    integralSum += (lastError + error) / 2 * dt
    angle = error*kp + integralSum*ki + ((error-lastError)/dt)*kd
    angle = rc_utils.clamp(angle, -1, 1)
    return angle
    
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
    global contour_area
    global contour_center
    global time
    # Search for contours in the current color image
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
    setpoint = 45
    speed = .12
    kp = .2
    
    if id == 1:
        cur_state = State.Ramp
    elif id == 2:
        cur_state = State.LineFollow     
    elif id == 3:
        cur_state = State.WallFollow
    elif id == 4:
        cur_state = State.FastWallFollow
    elif id == 5:
        cur_state = State.ConeSlalom
    elif id == 6:
        cur_state = State.LaneFollow
    elif id == 7:
        cur_state = State.WallFollowHazards
    elif id == 8:
        cur_state = State.WallFollow
        # cur_state = State.WallFollowBrick # code brick code if need separate

    print("closestp: ", rc_utils.get_lidar_closest_point(scan, (270, 90))[0])
    if rc_utils.get_lidar_closest_point(scan)[0] < 20 and cur_state != State.Ramp and cur_state != State.LaneFollow:
        cur_state = State.SafetyStop

    if cur_state == State.Ramp:
        if rc_utils.get_lidar_closest_point(scan, (270, 90))[0] < 100:
            time += rc.get_delta_time()
        if time < .7:
            speed = .22
        else:
            speed = .12
            cur_state = State.WallFollow
            time = 0
        angle = 0
    elif cur_state == State.LineFollow:
        if contour_center is not None:
            if lastError == None:
                lastError = 0   
            dt = rc.get_delta_time()
            angle = pid(.17, .1, .13, error, lastError, dt)
            lastError = error
    elif cur_state == State.LWallFollow or cur_state == State.WallFollow:
        error = rc_utils.get_lidar_average_distance(scan, 315, 35) - setpoint
        output = error*kp
        if rc_utils.get_lidar_average_distance(scan, 10, 10) < 60:
            pass
            # angle = 1
            # speed = .8
            # fix this
        if output > 0:
            print("neg")
            angle = -4*rc_utils.remap_range(output, 0, .1*((900-setpoint)*kp), 0, 1) 
        else:
            print("pos")
            angle = -4*rc_utils.remap_range(output, .1*(-setpoint*kp), 0, -.1, 0) 
    elif cur_state == State.RWallFollow or cur_state == State.FastWallFollow:
        error = rc_utils.get_lidar_average_distance(scan, 45, 35) - setpoint
        output = error*kp
        if rc_utils.get_lidar_average_distance(scan, 10, 10) < 60:
            angle = -1
            speed = .8
            # fix this
        elif output > 0:
            angle = 4*rc_utils.remap_range(output, 0, .1*((900-setpoint)*kp), 0, 1) 
        else:
            angle = 4*rc_utils.remap_range(output, .1*(-setpoint*kp), 0, -.1, 0) 
        speed = .2
    elif cur_state == State.LaneFollow:
        if contour_center is not None:
            #print(contour_center[1])
            if lastError == None:
                lastError = 0   
            dt = rc.get_delta_time()
            angle = pid(.17, .1, .13, error, lastError, dt)
            lastError = error
    elif cur_state == State.SafetyStop:
        print("stopped")
        angle = 0
        speed = 0
        rc.drive.stop()

    angle = rc_utils.clamp(angle,-1, 1)

    rc.drive.set_speed_angle(speed, angle)
    print("ang: ", angle)
    print("error: ", error)
    print("state: ", cur_state)
    # TODO: Follow the wall to the right of the car without hitting anything.


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()
