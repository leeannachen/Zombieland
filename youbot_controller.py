"""youbot_controller controller."""

from controller import Robot, Motor, Camera, Accelerometer, GPS, Gyro, LightSensor, Receiver, RangeFinder, Lidar
from controller import Supervisor

from youbot_zombie import *

#------------------CHANGE CODE BELOW HERE ONLY--------------------------
#define functions here for making decisions and using sensor inputs

#################### CAMERA FUNCTIONS BELOW ####################
def check_camera(camera4):
    frontBerry = 0
    frontZombie = 0
    obstacleProportion = {}
        
    image = camera4.getImageArray()
    cameraData = camera4.getImage()
    
    if image:
        imageHeight = camera4.getHeight()
        imageWidth = camera4.getWidth()
        print(imageHeight)
        print(imageWidth)
        for x in range(0,camera4.getWidth()):
            for y in range(0,camera4.getHeight()):
                red   = image[x][y][0]
                green = image[x][y][1]
                blue  = image[x][y][2]
                
                hsv = rgbToHSV(red, green, blue)
                print('h='+str(hsv[0])+' s='+str(hsv[1])+' v='+str(hsv[2]))
                obj = get_object_name(hsv)

                
                if obj != -1:
                    print(obj)
                    if obj == "pink_berry" or obj == "orange_berry" or obj == "red_berry" or obj == "yellow_berry":
                        # pixelAngle = x/imageWidth
                        # if pixelAngle > .25 or pixelAngle < .75:
                        if hsv[0] < 13.6 or hsv[0] > 18:
                            frontBerry = 1
                            frontZombie = 0
                            print("front berry detected")
                    if obj == "aqua_zombie" or obj == "blue_zombie" or obj == "purple_zombie" or obj == "green_zombie":
                        if check_receiver == 1:
                            frontBerry = 0
                            frontZombie = 1
                    if obj == "walls" or obj == "tree" or obj == "tree_stumps":
                        if obj not in obstacleProportion:
                           obstacleProportion[obj] = 1
                        else:
                            obstacleProportion[obj] += 1
                        if obstacleProportion.get(obj)/(imageWidth * imageHeight) >= .50:
                            frontObstacle = True
                            print("detected obstacle")
                        print("obstacle free")
        
        return 1                        
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
     # use the imageColor matrix to get the number of blobs and their centers
     # and populate them into the objectCoordinates dictionary
     # objectCoordinates = get_object_centers(objectCoordinates, imageColors, imageWidth, imageHeight, blobCounter, leftmost, rightmost)

#################### COLORFUNCTIONS BELOW ####################

objectColors = {

    #zombies 
    "aqua_zombie" : [[171.9, 100, 91], [176.3, 100, 32]], #unique hue
    "purple_zombie" : [[265.3, 82, 36], [284, 68, 100]],  #overlaps with tree strump
    "blue_zombie" : [[202.3, 100, 95], [215.3, 100, 42]], #unique hue, previously similar to the wall
    "green_zombie" : [[120, 100, 87],[126.9, 100, 24]],   #unique hue
    
    #berries
    "orange_berry" : [[10, 55, 30], [24.3, 56, 97]],      #conflicts with red_berries, tree, and floor; 3-sides: [21.4, 62, 87], [24.3, 56, 97], [10, 55, 30] <- problematic shaded side.
    "pink_berry" : [[295.3, 48, 31], [322,33,97]],        #unqiue hue
    "red_berry" : [[5.3, 88, 96], [6.6, 82, 100]],         #huge hue range [4.7, 88, 96] to [6.6, 82, 100] is for the well lit facades while [359, 82, 35] covers the darkest side
    "yellow_berry" : [[56, 100, 88], [58.5, 100, 31]],    #unique hue
    
    #world objects
    "tree" : [[0, 10, 8], [10.4, 35, 26]],                #tree conflicts with red berries ? not sure how the tree is perceived 
    "tree_stumps" : [[210, 6, 13], [240, 14, 5]],         #overlaps with purple zomblie 
    "walls" : [[225, 2, 90], [227, 29, 44]],            #within tree stump hues
    #"floor" : [[13.6, 39, 89], [18, 21, 92]]              #within orange berries range
}

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

def get_object_name(hsv):
    for key, val in objectColors.items():
        if hsv[0] >= val[0][0] and hsv[0] <= val[0][1]:
            return key
    return -1
        
#################### ROBOT MOVEMENT FUNCTIONS ####################

def set_speeds(wheels, speeds):
  for x in range(4):
    #wheels[x].setPosition(float('inf'))
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

#################### RECEIVER FUNCTIONS ####################

def check_receiver():
    if receiver.getQueueLength() > 0:
        # message=receiver.getData()
        # dataList = struct.unpack("chd" , message)
        return 1
    else:
        return 0
    
#################### STATE MACHINE FUNCTIONS ####################

def moveForward(wheels, speeds):
    go_forward(wheels, speeds)
    if frontObstacle:
        unstuck(wheels, speeds)
    elif frontZombie > 0:
        escapeZombie(wheels, speeds)
    elif frontBerry > 0:
        speedForward(wheels,speeds)
    else:
        wander(wheels, speeds)


def wander(wheels, speeds):
    # do a 420 degree sping
    for x in range(0, 16):
        if (frontBerry > 0):
            moveForward(wheels, speeds)
        for x in range(0, 6):
            turn_right(wheels, speeds)
        go_forward(wheels, speeds)
    moveForward( wheels, speeds)


def unstuck(wheels,speeds):
    while frontObstacle:
        for x in range(0, 6):
            turn_right(wheels, speeds)
        go_backwards(wheels, speeds)
    wander(wheels, speeds)


def escapeZombie(wheels,speeds):
    while frontObstacle or frontZombie > 0:
        for x in range(0, 6):
            turn_right(wheels, speeds)
    moveForward(wheels, speeds)

    
def speedForward(wheels,speeds):
    go_forward(wheels, 5.0)
    if frontObstacle:
        unstuck(wheels, speeds)
    elif frontZombie > 0:
        escapeZombie(wheels, speeds)
    elif frontBerry > 0:
        speedForward(wheels,speeds)
    else:
        wander(wheels, speeds)

#################### MOVEMENT FUNCTIONS ####################

def escapeObstacle(wheels,speeds):
    while frontObstacle:
        for x in range(0, 6):
            turn_right(wheels, speeds)
        go_backwards(wheels, speeds)
        
def escapeZombie(wheels,speeds):
    while frontObstacle or frontZombie > 0:
        for x in range(0, 6):
            turn_right(wheels, speeds)
    go_forward(wheels, 14.6)
    
    
def wander(wheels, speeds):
    for x in range(0, 6):
        turn_right(wheels, speeds)
    go_forward(wheels, speeds)
    
#GLOBAL VARIABLES
imageWidth = 0
imageHeight = 0
frontBerry = 0
frontZombie = 0
frontObstacle = False

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

    wheels = [fr, fl, br, bl]
    
        
    imageWidth = camera4.getWidth()
    imageHeight = camera4.getHeight()
    
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
    
        # initiate wheels    
        timeTick = 0    
        i+=1
        
        if(timer%16==0):
            check_camera(camera4)
            
            
        if(timer%32==0):     
            if frontZombie > 0:
                escapeZombie(wheels, 14.6)
            elif frontObstacle:
                escapeObstacle(wheels, 14.6)
            elif frontBerry > 0:
                go_forward(wheels, 14.6)
            else:
                if i % 10 == 0:
                    timeTick = 1
                if i %20 == 0:
                    tieTick = 0
                if timeTick == 1:
                    turn_right(wheels, 10.0)
                else:
                    go_forward(wheels, 14.0)
            
        

        #------------------CHANGE CODE ABOVE HERE ONLY--------------------------
        
        
    return 0   
    
main()
