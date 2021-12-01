"""youbot_controller controller."""

from controller import Robot, Motor, Camera, Accelerometer, GPS, Gyro, LightSensor, Receiver, RangeFinder, Lidar
from controller import Supervisor

from youbot_zombie import *
   
#------------------CHANGE CODE BELOW HERE ONLY--------------------------
#define functions here for making decisions and using sensor inputs

#BASE.c FUNCTION
SPEED = 3.0

DISTANCE_TOLERANCE = 0.001
ANGLE_TOLERANCE = 0.001

#stimulus coefficients
K1 = 3.0
K2 = 1.0
K3 = 1.0

#  wheels = [fr, fl, br, bl] 

# def defineWheels（wheelNames):
#     for x in range(3):
#         wheels[x] = wheelNames[x]


def base_set_wheel_helper(wheels, speeds):
  for x in range(4):
    wheels[x].setPosition(float('inf'))
    wheels[x].setVelocity(speeds[x])

def base_reset(wheels):
    speeds = [0.0, 0.0, 0.0, 0.0]
    base_set_wheel_helper(wheels, speeds)

def base_forwards(wheels):
    speeds = [SPEED, SPEED, SPEED, SPEED]
    base_set_wheel_helper(wheels, speeds)

def base_backwards(wheels):
    speeds = [-SPEED, -SPEED, -SPEED, -SPEED]
    base_set_wheel_helper(wheels, speeds)

def base_turn_right(wheels):
    speeds = [-SPEED, SPEED, -SPEED, SPEED]
    base_set_wheel_helper(wheels, speeds)

def base_turn_left(wheels):
    speeds = [SPEED, -SPEED, SPEED, -SPEED]
    base_set_wheel_helper(wheels, speeds)







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
    
    camera1 = robot.getDevice("ForwardLowResBigFov")
    camera1.enable(timestep)
    
    camera2 = robot.getDevice("ForwardHighResSmallFov")
    camera2.enable(timestep)
    
    camera3 = robot.getDevice("ForwardHighRes")
    camera3.enable(timestep)
    
    camera4 = robot.getDevice("ForwardHighResSmall")
    camera4.enable(timestep)
    
    camera5 = robot.getDevice("BackLowRes")
    camera5.enable(timestep)
    
    camera6 = robot.getDevice("RightLowRes")
    camera6.enable(timestep)
    
    camera7 = robot.getDevice("LeftLowRes")
    camera7.enable(timestep)
    
    camera8 = robot.getDevice("BackHighRes")
    camera8.enable(timestep)
    
    gyro = robot.getDevice("gyro")
    gyro.enable(timestep)
    
    lightSensor = robot.getDevice("light sensor")
    lightSensor.enable(timestep)
    
    receiver = robot.getDevice("receiver")
    receiver.enable(timestep)
    
    rangeFinder = robot.getDevice("range-finder")
    rangeFinder.enable(timestep)
    
    lidar = robot.getDevice("lidar")
    lidar.enable(timestep)
    
    fr = robot.getDevice("wheel1")
    fl = robot.getDevice("wheel2")
    br = robot.getDevice("wheel3")
    bl = robot.getDevice("wheel4")
    
    fr.setPosition(float('inf'))
    fl.setPosition(float('inf'))
    br.setPosition(float('inf'))
    bl.setPosition(float('inf'))
    
    
    i=0
           

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
         #called every timestep
        
        wheels = [fr, fl, br, bl] 

        # if(robot.step(timestep) < 100):
        #     base_forwards(wheels)
        
        #base_right(wheels)

       # if(robot.step(timestep) > 100 and robot.step(timestep) < 500):
        
        base_left(wheels)

        

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
