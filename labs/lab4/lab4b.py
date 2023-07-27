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
import matplotlib.axes._axes as ax
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

class State(IntEnum):
    WallFollow = 0
    Stop = 1
cur_state: State = State.WallFollow

########################################################################################
# Functions
########################################################################################


def start():
    """
    This function is run once every time the start button is pressed
    """
    global cur_state
    cur_state = State.WallFollow
    # Have the car begin at a stop
    rc.drive.stop()

    # Print start message
    print(">> Lab 4B - LIDAR Wall Following")

    
def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    global cur_state

    
    scan = rc.lidar.get_samples_async()
    setpoint = 45
    error = rc_utils.get_lidar_average_distance(scan, 50, 35) - setpoint

    speed = 0.14

    kp = .4
    output = error*kp
    if output > 0:
        angle = 2*rc_utils.remap_range(output, 0, .1*((900-setpoint)*kp), 0, 1) 
    else:
        angle = 2*rc_utils.remap_range(output, .1*(-setpoint*kp), 0, -.1, 0) 

    angle = rc_utils.clamp(angle,-1, 1)
    rc.drive.set_speed_angle(speed, angle)
    print("ang: ", angle)
    print("error: ", error)


    kernel = [-3, -3, 5, -3, -3]
    stdev = np.std(scan)#calc stdev of the data 
    distances = []
    clusters = {}
    b = 1 #tune b st it splits the clusters right
    #index to identify cluster to map to angle
    #start to end of cluster can be a line or reholt etc
    # 1. plot lines 
    # CHANGE THE RANGE FOR LIDAR SO IT LOOKS FURTHER AHEAD
    b_sig = stdev*b
    C=0
    for i in range(1, len(scan)-1):
        distances.append(math.dist([scan[i]], [scan[i+1]]))
    for i in range(1, len(scan)):
        sum = 0
        for j in range(int(-(len(kernel)/2)), int((len(kernel)-1)/2)):
            index = i + j
            if i+j > len(distances)-1:
                index = len(distances)-1
            sum = sum + distances[index]*(kernel[j+int((len(kernel)+1)/2)])
        if sum > b_sig or len(clusters) == 0:
            clusters[sum] = [scan[i]]
            C = sum
        else:
            clusters[C] += [scan[i]]
       # C = sum
    for sum, points in clusters.items():

        x = [key for (key,values) in clusters.items() for _ in range(len(values))]
        y = [val for subl in clusters.values() for val in subl]

        ax.plot(x, y, 'ro')
        plt.show()

    print(clusters.keys())
    if rc_utils.get_lidar_closest_point(scan)[0] < 25:
        cur_state = State.Stop

    if cur_state == State.Stop:
        print("stopped")
        rc.drive.stop()
    # TODO: Follow the wall to the right of the car without hitting anything.


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()
