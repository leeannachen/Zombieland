"""youbot_controller controller."""

from controller import Robot, Motor, Camera, Accelerometer, GPS, Gyro, LightSensor, Receiver, RangeFinder, Lidar
from controller import Supervisor

from youbot_zombie import *

#------------------CHANGE CODE BELOW HERE ONLY--------------------------
#define functions here for making decisions and using sensor inputs

#----------------- CAMERA FUNCTIONS BELOW ----------------------------------

import struct

objectColors = {

    #zombies 
    "aqua_zombie" : [[171.9, 100, 91], [176.3, 100, 32]], #unique hue
    "purple_zombie" : [[265.3, 82, 36], [284, 68, 100]],  #overlaps with tree strump
    "blue_zombie" : [[202.3, 100, 95], [215.3, 100, 42]], #unique hue, previously similar to the wall
    "green_zombie" : [[120, 100, 87],[126.9, 100, 24]],   #unique hue
    
    #berries
    "orange_berry" : [[10, 55, 30], [24.3, 56, 97]],      #conflicts with red_berries, tree, and floor; 3-sides: [21.4, 62, 87], [24.3, 56, 97], [10, 55, 30] <- problematic shaded side.
    "pink_berry" : [[295.3, 48, 31], [322,33,97]],        #unqiue hue
    "red_berry" : [[5.3, 88, 96], [359, 82, 35]],         #huge hue range [4.7, 88, 96] to [6.6, 82, 100] is for the well lit facades while [359, 82, 35] covers the darkest side
    "yellow_berry" : [[56, 100, 88], [58.5, 100, 31]],    #unique hue
    
    #world objects
    "tree" : [[0, 10, 8], [10.4, 35, 26]],                #tree conflicts with red berries ? not sure how the tree is perceived 
    "tree_stumps" : [[210, 6, 13], [240, 14, 5]],         #overlaps with purple zomblie 
    "walls" : [[225, 2, 90], [226.9, 29, 44]],            #within tree stump hues
    #"floor" : [[13.6, 39, 89], [18, 21, 92]]              #within orange berries range
}

def get_object_name(hsv):
    for key, val in objectColors:
        if hsv == val[0] or hsv == val[1]:
            return key
    return -1
    
# recursive algo that returns a dictionary of objects and their coordinate center
# def get_object_centers(coordinatesDict, imageColors, w, h, blobCounter, leftmost, rightmost):
    # for x in range(0,w):
        # for y in range(0,h):
            # if imageColors[x][y] == 0:
                # return 0
            # if imageColors[x][y] != 0:
                # if x < leftmost:
                    # leftmost = x
                # if y > rightmost:
                    # rightmost = y
                # coordinatesDict[blobCounter] = [x,y]
                # blobCounter += 1
                # imageColors[x][y] = 0
                # return 1 + get_blob_center(imageColors[x][y+1]
            # when we increment the blobCounter, basically, end of one blob, we 
            # have to reset the leftmost and rightmost counters
               
# def get_blob_center(pixel):
    # if pixel == 0:
        # return 0
    # if pixel == NULL:
        # return 0
    # else:
        
#------------------State Machine Functions Below--------------------------

        #PSEUDO CODE
        
        # if safe_berry:
            # position berry camera to the middle
             
            # if robot_info[0] < 80 OR robot_info[1] < 70:
               # go_forward(wheels, 16.0)
            # else:
                # go_forward(wheels, 10.0)
        
        # if moving forward:
            # if section2clear and section3clear:
                # go_forward(wheels, 10.0, 10.0)
            # else:
                # processSurrounding()
        
        # if processSurrounding State:
            # if berryExists with No Zombies:
                
                # switchtocase SafeBerries  
        
#----------------- ROBOT MOVEMENT FUNCTIONS BELOW -------------------------


#BASE.c FUNCTION
#SPEED = 3.0

# wheels = [fr, fl, br, bl]

def set_speeds(wheels, speeds):
  for x in range(4):
    wheels[x].setPosition(float('inf'))
    wheels[x].setVelocity(speeds[x])

def stop_wheels(wheels):
    speeds = [0.0, 0.0, 0.0, 0.0]
    set_speeds(wheels, speeds)

def go_forward(wheels, SPEED):
    speeds = [SPEED, SPEED, SPEED, SPEED]
    set_speeds(wheels, speeds)

def go_backwards(wheels, SPEED):
    speeds = [-SPEED, -SPEED, -SPEED, -SPEED]
    set_speeds(wheels, speeds)

def turn_right(wheels, SPEED):
    speeds = [-SPEED, SPEED, -SPEED, SPEED]
    set_speeds(wheels, speeds)

def turn_left(wheels, SPEED):
    speeds = [SPEED, -SPEED, SPEED, -SPEED]
    set_speeds(wheels, speeds)
    
#CAMERA RGB to HSV conversion

def rgbToHSV(r, g, b):
 
    # R, G, B values are divided by 255
    # to change the range from 0..255 to 0..1:
    r, g, b = r / 255.0, g / 255.0, b / 255.0
 
    # h, s, v = hue, saturation, value
    cmax = max(r, g, b)    # maximum of r, g, b
    cmin = min(r, g, b)    # minimum of r, g, b
    diff = cmax-cmin       # diff of cmax and cmin.
 
    # if cmax and cmax are equal then h = 0
    if cmax == cmin:
        h = 0
     
    # if cmax equal r then compute h
    elif cmax == r:
        h = (60 * ((g - b) / diff) + 360) % 360
 
    # if cmax equal g then compute h
    elif cmax == g:
        h = (60 * ((b - r) / diff) + 120) % 360
 
    # if cmax equal b then compute h
    elif cmax == b:
        h = (60 * ((r - g) / diff) + 240) % 360
 
    # if cmax equal zero
    if cmax == 0:
        s = 0
    else:
        s = (diff / cmax) * 100
 
    # compute v
    v = cmax * 100
    return h, s, v


#------------------CHANGE CODE ABOVE HERE ONLY--------------------------

def main():
    robot = Supervisor()

    # get the time step of the current world.
    timestep = int(robot.getBasicTimeStep())
    
    #health, energy, armour in that order 
    robot_info = [100,100,0]
    passive_wait(0.1, robot, timestep)
    pc = 0
    timer = 0
    
    robot_node = robot.getFromDef("Youbot")
    trans_field = robot_node.getField("translation")
    
    get_all_berry_pos(robot)
    
    robot_not_dead = 1
    
    #------------------CHANGE CODE BELOW HERE ONLY--------------------------
    
    #COMMENT OUT ALL SENSORS THAT ARE NOT USED. READ SPEC SHEET FOR MORE DETAILS
    #accelerometer = robot.getDevice("accelerometer")
    #accelerometer.enable(timestep)
    
    #gps = robot.getDevice("gps")
    #gps.enable(timestep)
    
    #compass = robot.getDevice("compass")
    #compass.enable(timestep)
    
    # camera1 = robot.getDevice("ForwardLowResBigFov")
    # camera1.enable(timestep)
    
    # camera2 = robot.getDevice("ForwardHighResSmallFov")
    # camera2.enable(timestep)
    
    # camera3 = robot.getDevice("ForwardHighRes")
    # camera3.enable(timestep)
    
    camera4 = robot.getDevice("ForwardHighResSmall")
    camera4.enable(timestep)
    
    # camera5 = robot.getDevice("BackLowRes")
    # camera5.enable(timestep)
    
    camera6 = robot.getDevice("RightLowRes")
    camera6.enable(timestep)
    
    camera7 = robot.getDevice("LeftLowRes")
    camera7.enable(timestep)
    
    # camera8 = robot.getDevice("BackHighRes")
    # camera8.enable(timestep)
    
    # gyro = robot.getDevice("gyro")
    # gyro.enable(timestep)
    
    # lightSensor = robot.getDevice("light sensor")
    # lightSensor.enable(timestep)
    
    receiver = robot.getDevice("receiver")
    receiver.enable(timestep)
    
    # rangeFinder = robot.getDevice("range-finder")
    # rangeFinder.enable(timestep)
    
    # lidar = robot.getDevice("lidar")
    # lidar.enable(timestep)
    
    fr = robot.getDevice("wheel1")
    fl = robot.getDevice("wheel2")
    br = robot.getDevice("wheel3")
    bl = robot.getDevice("wheel4")
    
    fr.setPosition(float('inf'))
    fl.setPosition(float('inf'))
    br.setPosition(float('inf'))
    bl.setPosition(float('inf'))
    
    
    i=0
    angle = 0


    #------------------CHANGE CODE ABOVE HERE ONLY--------------------------
    
    
    while(robot_not_dead == 1):
        
        if(robot_info[0] < 0):
           
            robot_not_dead = 0
            print("ROBOT IS OUT OF HEALTH")
            #if(zombieTest):
            #    print("TEST PASSED")
            #else:
            #    print("TEST FAILED")
            #robot.simulationQuit(20)
            #exit()
            
        if(timer%2==0):
            trans = trans_field.getSFVec3f()
            robot_info = check_berry_collision(robot_info, trans[0], trans[2], robot)
            robot_info = check_zombie_collision(robot_info, trans[0], trans[2], robot)
            
        if(timer%16==0):
            robot_info = update_robot(robot_info)
            timer = 0
        
        if(robot.step(timestep)==-1):
            exit()
            
            
        timer += 1
        
     #------------------CHANGE CODE BELOW HERE ONLY--------------------------   
     
     #------------------ Behavior Functions --------------------------

     # movements: go_forward, go_backwards, turn_left, turn_right
        
        # initiate wheels
        wheels = [fr, fl, br, bl] 
        
        # turn right around 90 degrees 
        # if angle > -11.8:
            # turn_right(wheels, 3.0)
        # else:
            # go_forward(wheels, 3.0)

        # angle = angle + values[2]
        
        
        #------------RangeFinder------Depth Images

        #values = rangeFinder.getRangeImage()
        
        if receiver.getQueueLength() > 0:
            message=receiver.getData()
            dataList = struct.unpack("chd" , message)
            print(dataList[0])
            

        if i % 16 == 0:
            turn_right(wheels, 3.0)
        
        go_forward(wheels, 10.0)

        i += 1

        
        #---------------------------CAMERA CODE--------------------
        
        # objectsInFOV stores the object and the pixel locations in the image
        # example of the pixel coordinates of 2 orange berries--> 
        # orange_berry : [[0, 8], [0,9], [0,10], [1, 8], [1,9], [1,10].
        #               [6, 0], [6, 1], [6, 2], [7, 0], [7, 1], [7, 2]]
        objectsInFOV = {}
        # this stores the rightmost and leftmost coordinates of each blob
        objectCoordinates = {}

        #camera samples surrounding images every 2/16 of a second
        if timer % 16 == 0:
            
            image = camera4.getImageArray()

            # image dimensions
            cameraData = camera4.getImage()
            imageWidth = camera4.getWidth()
            imageHeight = camera4.getHeight()
            
            imageColors = [[0 for x in range(imageWidth)] for y in range(imageHeight)]

            if image:
                # display the components of each pixel
                for x in range(0,camera4.getWidth()):
                    for y in range(0,camera4.getHeight()):
                        red   = image[x][y][0]
                        green = image[x][y][1]
                        blue  = image[x][y][2]
                        print('r='+str(red)+' g='+str(green)+' b='+str(blue))
                        #convert to HSV and identify object
                        hsv = rgbToHSV(red, green, blue)
                        obj = get_object_name(hsv)
                        if obj != -1:
                            if obj == "pink_berry"
                            or obj == "orange_berry"
                            or obj == "red_berry"
                            or obj == "yellow_berry":
                                pixelAngle = x/imageWidth * 100
                                if pixelAngle > 25 or pixelAngle < 75:
                                    state = MOVEFORWARD
                            
             # use the imageColor matrix to get the number of blobs and their centers
             # and populate them into the objectCoordinates dictionary
             # blobCounter = 0
             # leftmost = imageWidth
             # rightmost = 0
             # objectCoordinates = get_object_centers(objectCoordinates, imageColors, imageWidth, imageHeight, blobCounter, leftmost, rightmost)
             

         
        #------------------CHANGE CODE ABOVE HERE ONLY--------------------------
        
        
    return 0   


main()
