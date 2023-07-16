"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Lab 1 - Driving in Shapes
"""

########################################################################################
# Imports
########################################################################################

import sys

sys.path.insert(1, "../../library")
import racecar_core
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################
rc = racecar_core.create_racecar()

# Put any global variables here
global queue
queue=[]
########################################################################################
# Functions
########################################################################################


def start():

    """
    This function is run once every time the start button is pressed
    """
    # Begin at a full stop
    rc.drive.stop()

    queue.clear()
    # Print start message
    # TODO (main challenge): add a line explaining what the Y button does
    print(
        ">> Lab 1 - Driving in Shapes\n"
        "\n"
        "Controls:\n"
        "    Right trigger = accelerate forward\n"
        "    Left trigger = accelerate backward\n"
        "    Left joystick = turn front wheels\n"
        "    A button = drive in a circle\n"
        "    B button = drive in a square\n"
        "    X button = drive in a figure eight\n"
    ) 

def driveCircle():
    global queue

    # Tune these constants until the car completes a full circle
    CIRCLE_TIME = 6.4
    BRAKE_TIME = 0.5

    queue.clear()

    # Turn right at full speed, then reverse to quickly stop
    queue.append([CIRCLE_TIME, 1, 1])
    queue.append([BRAKE_TIME, 0, 1])


def driveTriangle():
    global queue

    Triangle_Time = 5
    Brake_Time = 0.5
    Turn_Time = 33/4

    queue.clear()

    queue.append([Triangle_Time, 0.3, 0])
    queue.append([Turn_Time, 0.2, 1])
    queue.append([Triangle_Time, 0.3, 0])
    queue.append([Turn_Time, 0.2, 1])
    queue.append([Triangle_Time, 0.3, 0])
    queue.append([Brake_Time, 0, 0])




def update():
    global queue
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    # TODO (warmup): Implement acceleration and steering
    rc.drive.set_speed_angle(0, 0)

    if rc.controller.was_pressed(rc.controller.Button.A):
        driveCircle()

    if len(queue) > 0:
        speed = queue[0][1]
        angle = queue[0][2]
        queue[0][0] -= rc.get_delta_time()
        rc.drive.set_speed_angle(speed, angle)
        if queue[0][0] <= 0:
            queue.pop(0)
            

    if rc.controller.was_pressed(rc.controller.Button.Y):
        driveTriangle()

   


    
   


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update)
    rc.go()
