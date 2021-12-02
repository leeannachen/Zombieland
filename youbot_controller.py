"""youbot_controller controller."""

import cv2
import numpy as np

from controller import Robot, Motor, Camera, Accelerometer, GPS, Gyro, LightSensor, Receiver, RangeFinder, Lidar
from controller import Supervisor

from youbot_zombie import *
   
#------------------CHANGE CODE BELOW HERE ONLY--------------------------
#define functions here for making decisions and using sensor inputs

#----------------- CAMERA FUNCTIONS BELOW ----------------------------------

colorsDict = {
    [0,0.7,0.9] : "aqua",
    [0.6, 0.2, 1] : "purple",
    [0, 0.5, 1] : "blue",
    [0, 0.7, 0] : "green",
    [1, 0.2, 0.1] : "red",
    [0.9, 0.5, 0.7] : "pink",
    [1, 0.9, 0] : "yellow",
    [0,0,0] : "black",
    [0.9,0.5,0.3] : "orange"
}

def get_color_name(colorArray):
    if colorArray in colorsDict:
        return colorsDict.get(colorArray)


#----------------- ROBOT MOVEMENT FUNCTIONS BELOW -------------------------


#BASE.c FUNCTION
#SPEED = 3.0

DISTANCE_TOLERANCE = 0.001
ANGLE_TOLERANCE = 0.001

#stimulus coefficients
K1 = 3.0
K2 = 1.0
K3 = 1.0


#  wheels = [fr, fl, br, bl] 


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
    
    camera5 = robot.getDevice("BackLowRes")
    camera5.enable(timestep)
    
    camera6 = robot.getDevice("RightLowRes")
    camera6.enable(timestep)
    
    camera7 = robot.getDevice("LeftLowRes")
    camera7.enable(timestep)
    
    # camera8 = robot.getDevice("BackHighRes")
    # camera8.enable(timestep)
    
    gyro = robot.getDevice("gyro")
    gyro.enable(timestep)
    
    # lightSensor = robot.getDevice("light sensor")
    # lightSensor.enable(timestep)
    
    # receiver = robot.getDevice("receiver")
    # receiver.enable(timestep)
    
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
            
            
            #camera samples surrounding images every 2/16 of a second
            cameraData = camera4.getImage()

            imageArray = camera4.getImageArray()
            image = cv2.imread(imageArray)
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            cv2.imshow("window", image)
            # indicate the lower and upper range for each color
            lower_range = np.array([110,50,50])
            upper_range = np.array([130,255,255])

            mask = cv2.inRange(hsv, lower_range, upper_range)
            cv2.imshow("Image", image)
            cv2.imshow("Mask", mask)

            if image:
                # display the components of each pixel
                for x in range(0,camera4.getWidth()):
                    for y in range(0,camera4.getHeight()):
                        red   = image[x][y][0]
                        green = image[x][y][1]
                        blue  = image[x][y][2]
                        print('r='+str(red)+' g='+str(green)+' b='+str(blue))

            
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

        values = gyro.getValues()

               
        # turn right around 90 degrees 
        if angle > -11.8:
            turn_right(wheels, 3.0)
        else:
            go_forward(wheels, 3.0)

        angle = angle + values[2]
        i += 1

        
        
        #moveForward()
        #possible pseudocode for moving forward, then doing a 90 degree left turn
        # if i < 100
        #     base_forwards() #-> can implement in Python with Webots C code (/Zombie world/libraries/youbot_control) as an example or make your own
        
        # if == 100 
        #     base_reset() 
        #     base_turn_left()  
        #     #it takes about 150 timesteps for the robot to complete the turn
                 
        # if i==300
        #     i = 0
        
        # i+=1
        
        #make decisions using inputs if you choose to do so
         
        #------------------CHANGE CODE ABOVE HERE ONLY--------------------------
        
        
    return 0   


main()
