import sys
import cv2 as cv
import numpy as np
                                                                                                                #todo: add target selection, fix pid, add route completion script
sys.path.insert(1, "../../library")
import racecar_core
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()
                                                                   #   Probably edit this to be larger to decrease sensitivity

# A crop window for the floor directly in front of the car


HeightCrop = 180 #200 
CROP_FLOOR = ((HeightCrop,0), (240, 320)) #((HeightCrop,0), (rc.camera.get_height(),rc.camera.get_width()))


# Colors, stored as a pair (hsv_min, hsv_max)
RED=((160, 50, 170), (179, 255, 255))
YEL=((160, 50, 50), (45, 255, 255))
GRE=((50,70,70), (70, 255, 255))
CYA=((76, 50, 50), (105, 255, 255))
BLU=((90, 50, 50), (120, 255, 255))
MAG=((136, 50, 50), (165, 255, 255))
YEL = ((20, 20, 20), (33, 255, 255))
OUTYEL = ((23, 65, 65), (32, 255, 255))
MIDYEL = ((15, 40, 40), (33, 255, 255))
ORA = ((0, 0, 0), (70, 180, 180))
# PUR=((100, 0, 0), (179, 250, 250))
PUR = ((120, 140, 0), (150, 255, 255))
hsvTarget=((120,50,122),(150,200,200))   
prioritylist = [PUR]

def pid(Kp,Ki,Kd,target,current,dT):
    global accumulatedError
    global lastError
    error=target-current
    accumulatedError+=error*dT
    deltaError=(error-lastError)/dT
    pTerm=Kp*error
    iTerm=Ki*accumulatedError
    dTerm=Kd*deltaError
    lastError=error
    return pTerm+iTerm+dTerm

#This function returns the largest and second largest contour
def get_largest_and_second_contour(contours, MIN_CONTOUR_AREA):
    contours = list(contours)
    count = 0
    for contour in contours:
        if contour is not None:
            if cv.contourArea(contour) > MIN_CONTOUR_AREA:
                count += 1
        else:
            count += 1
    if count == 0:
        return None, None
    index = 0
    greatest = 0
    greatestcontour = None
    for i in range(len(contours)):
         if cv.contourArea(contours[i]) > greatest:
            greatest = cv.contourArea(contours[i])
            index = i
            greatestcontour = contours[i]
    contours.pop(index)
    
    greatest = 0
    secondgreatest = None
    for i in range(len(contours)):
         if cv.contourArea(contours[i]) > greatest:
            greatest = cv.contourArea(contours[i])
            secondgreatest = contours[i]
    return greatestcontour, secondgreatest

#This iterates through the priority list until it finds contours that are greater than the min contour area and gets the contour center of the largest and second              
def update_contour_two_line():
    A = 0
    while True:
        A+=1
        MIN_CONTOUR_AREA = 50
        image = rc.camera.get_color_image()
        image = rc_utils.crop(image, CROP_FLOOR[0], CROP_FLOOR[1])
        largestcontour = None
        secondcontour = None
        largestcontour_center = (0, 0)
        secondcontour_center = (0, 0)
        generalcontour_center = (0, 0) #This is the center if only one contour is seen on the screen
        for col in prioritylist:
            contours = rc_utils.find_contours(image, col[0], col[1])
            largestcontour, secondcontour = get_largest_and_second_contour(contours, MIN_CONTOUR_AREA)
            if (largestcontour is not None) and (secondcontour is not None):
                break
        if largestcontour is not None and secondcontour is not None:
            if largestcontour is not None:
                largestcontour_center = rc_utils.get_contour_center(largestcontour)
                rc_utils.draw_contour(image,largestcontour)
                if largestcontour_center is not None:
                    rc_utils.draw_circle(image,largestcontour_center)
            if secondcontour is not None:
                secondcontour_center = rc_utils.get_contour_center(secondcontour)
                rc_utils.draw_contour(image,secondcontour)
                rc_utils.draw_circle(image,secondcontour_center)
        else:
            if largestcontour is not None:
                generalcontour_center = rc_utils.get_contour_center(largestcontour)
                rc_utils.draw_contour(image,largestcontour)
            if secondcontour is not None:
                generalcontour_center = rc_utils.get_contour_center(secondcontour)
                rc_utils.draw_contour(image,secondcontour)


        rc.display.show_color_image(image)
        return largestcontour_center, secondcontour_center, generalcontour_center


def start():
    #varaibles that do not need to be copied
    global mainstate
    global timestamp
    timestamp = 0
    mainstate = "two line follow"

def update():
    #mainvariable
    global mainstate
    global timestamp
    if mainstate == "two line follow":
        timestamp = timestamp + rc.get_delta_time()
        cameraWidth = rc.camera.get_width()#320
        speed = .5 #.15
        angle = 0
        distancethreshold = 50
        largestcontour_center, secondcontour_center, generalcontour_center = update_contour_two_line()
        #print(largestcontour_center[1])
        #print(generalcontour_center[1])
        if largestcontour_center[1] != 0 and secondcontour_center[1] != 0:
            smallestx = 0
            largestx = 0
            if largestcontour_center[1] > secondcontour_center[1]:
                largestx = largestcontour_center[1]
                smallestx = secondcontour_center[1]
                # generalcontour_center = largestcontour_center
            else:
                smallestx = largestcontour_center[1]
                largestx = secondcontour_center[1]
                # generalcontour_center = secondcontour_center
            generalcontour_center_case = (largestx+smallestx)/2
            if (largestx - smallestx) > distancethreshold:
                distance = (largestx + smallestx)/2
                angle = rc_utils.remap_range(distance, 0, cameraWidth, -.25, .25)
                print(distance)
                #angle = pid(Kp,Ki,Kd,remap_range(contour_center[1], 0, int(320), -.7, .7), angle, rc.get_delta_time())
                # angle = rc_utils.remap_range(.001, smallestx, largestx, -1, 1)
            else:
                if generalcontour_center_case < (cameraWidth/2)-30:
                    angle = rc_utils.remap_range(generalcontour_center_case, 0, cameraWidth, .1, .4)
                if generalcontour_center_case > (cameraWidth/2)+30:
                    angle = rc_utils.remap_range(generalcontour_center_case, 0, cameraWidth, -.1, -.4)
        else:
            if generalcontour_center[1] < cameraWidth/2:
                angle = rc_utils.remap_range(generalcontour_center[1], 0, cameraWidth, .2, .25)
            if generalcontour_center[1] > cameraWidth/2:
                angle = rc_utils.remap_range(generalcontour_center[1], 0, cameraWidth, .2, -.25)

        angle = rc_utils.clamp(angle, -.25, .25)
        
        
        if timestamp > .2:
            speed = -.01
        if timestamp > .22:
            timestamp = 0

        rc.drive.set_speed_angle(speed, angle)

def update_slow():
    pass

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()



