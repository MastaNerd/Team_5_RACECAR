import sys
import cv2 as cv
import numpy as np
from enum import IntEnum

sys.path.insert(1, "../../library")
import racecar_core
import racecar_utils as rc_utils
from enum import IntEnum

rc = racecar_core.create_racecar()

# >> Constants
# The smallest contour we will recognize as a valid contour
MIN_CONTOUR_AREA = 30

global queue
queue=[]

coneColor = None


# A crop window for the floor directly in front of the car
CROP_FLOOR_LF = ((180,0), (rc.camera.get_height(), rc.camera.get_width()))
CROP_FLOOR_CS = ((100,0), (rc.camera.get_height(), rc.camera.get_width()))


# Colors, stored as a pair (hsv_min, hsv_max)
BLUE = ((90, 50, 50), (120, 255, 255))
RED = ((0,116, 113),(0,159,214))
GREEN = ((40,120,120), (100,255,255))
YELLOW = ((20,50,50),(32,255,255))
ORANGE =((175, 160, 130), (179, 255, 250))
PURPLE = ((90, 80, 80), (113, 255, 255))
COLORS = (BLUE, YELLOW)


# >> Variables
speed = 0.0  # The current speed of the car
angle = 0.0  # The current angle of the car's wheels
contour_center = None  # The (pixel row, pixel column) of contour
contour_area = 0  # The area of contour

########################################################################################
# States
########################################################################################
class State(IntEnum):
    LineFollow = 0
    ConeSlalom = 1
    Park = 2
class StateSlalom(IntEnum):
    Search = 0
    orangeCurve = 1
    purpleCurve = 2
cur_state_slalom: StateSlalom = StateSlalom.Search
cur_state: State = State.LineFollow

########################################################################################
# Functions
########################################################################################

def start():
    """
    This function is run once every time the start button is pressed
    """
    global speed
    global angle
    global lastError 
    global cur_state
    global cur_state_slalom
    global queue
    global orangeContour

    # Initialize variables
    speed = 0
    angle = 0

    cur_state_slalom = State.Search

    queue.clear()
    # Set initial driving speed and angle
    #rc.drive.set_speed_angle(speed, angle)

    # Set update_slow to refresh every half second
    rc.set_update_slow_time(0.5)

    """
    This function is run once every time the start button is pressed
    """
    # Initialize variables
    speed = 0
    angle = 0
    lastError = 0

    # Set initial driving speed and angle
    rc.drive.set_speed_angle(speed, angle)
    # Print start message
    print(
        ">> Lab 2A - Color Image Line Following\n"
        "\n"
        "Controls:\n"
        "    Right trigger = accelerate forward\n"
        "    Left trigger = accelerate backward\n"
        "    A button = print current speed and angle\n"
        "    B button = print contour center and area"
        )

def update_contour():
    global orangeContour
    global contour_center
    global contour_area
    global cur_state
    global cur_state_slalom
    global coneColor
    global contour_center

    image = rc.camera.get_color_image()

    if image is None:
        contour_center = None
        contour_area = 0
    else:
        # Crop the image depending on which program we are running
        if cur_state == State.LineFollow:
            image = rc_utils.crop(image, CROP_FLOOR_LF[0], CROP_FLOOR_LF[1])
        elif cur_state == State.ConeSlalom:
            image = rc_utils.crop(image, CROP_FLOOR_CS[0], CROP_FLOOR_CS[1])

        orangeContour = rc_utils.find_contours(image, ORANGE[0], ORANGE[1])

        purpleContour = rc_utils.find_contours(image, PURPLE[0], PURPLE[1])
        
        if cur_state == State.ConeSlalom:
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
        elif cur_state == State.LineFollow:

        #Find all of contours of desired color and get biggest contour
            for i in COLORS:         
                currentCont = rc_utils.find_contours(image,i[0], i[1] )
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
    global cur_state_slalom
    global queue
    global coneColor
    global speed
    global angle
    global lastError
    global orangeContour

    update_contour()
    update_slow()

    if orangeContour > 100:
        cur_state = State.ConeSlalom

    #     # Choose an angle based on 
#     # If we could not find a contour, keep the previous angle
    if cur_state == State.LineFollow:
        if contour_center is not None:
            if lastError == None:
                lastError = 0   
            kp =.4
            kd = .1
            setpoint = rc.camera.get_width()/2
            error = contour_center[1] - setpoint 
            output = kp*error + kd*(error - lastError)
            lastError = error      
        
            angle = rc_utils.remap_range(output, kp*-setpoint + kd*(-2*setpoint), kp*setpoint + kd*((2*setpoint)), -1, 1)
            
        speed = .17

        rc.drive.set_speed_angle(speed, angle)
    

#     # Print the current speed and angle when the A button is held down
    if rc.controller.is_down(rc.controller.Button.A):
        print("Speed:", speed, "Angle:", angle)

#     # Print the center and area of the largest contour when B is held down
    if rc.controller.is_down(rc.controller.Button.B):
        if contour_center is None:
            print("No contour found")
        else:
            print("Center:", contour_center, "Area:", contour_area)


    if coneColor == "orange":
        cur_state_slalom = StateSlalom.orangeCurve
    elif coneColor == "purple":
        cur_state_slalom = StateSlalom.purpleCurve
    
    if cur_state_slalom == StateSlalom.orangeCurve:
        orangeCurve()
    elif cur_state_slalom == StateSlalom.purpleCurve:
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
    
