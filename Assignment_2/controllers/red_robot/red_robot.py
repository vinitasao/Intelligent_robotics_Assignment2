from controller import Motor, Supervisor
import numpy as np
import math
TIME_STEP = 64
MAX_SPEED = 2
goal = np.array([1.32,0.75])
 
robot = Supervisor()
gps = robot.getDevice('gps')
gps.enable(TIME_STEP)
object1 = robot.getFromDef('rob1')
object2 = robot.getFromDef('rob2')
# initialize devices
ps = []
psNames = [
    'ps0', 'ps1', 'ps2', 'ps3',
    'ps4', 'ps5', 'ps6', 'ps7'
]

for i in range(8):
    ps.append(robot.getDevice(psNames[i]))
    ps[i].enable(TIME_STEP)
    
object = robot.getFromDef('epuck') 
rot = object.getField("rotation")#get rotation

leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

while robot.step(TIME_STEP) != -1:
    gps_value = gps.getValues()
    current_loc = np.array(gps_value[:2])
    rotation = np.degrees(rot.getSFRotation()[3])
    rob1_location = object1.getField('translation')
    rob1_loc = np.array(rob1_location.getSFVec3f()[:2])
    # print("rob1 loc",rob1_loc)
    rob2_location = object2.getField('translation')
    rob2_loc = np.array(rob2_location.getSFVec3f()[:2])
    # print("rob2 loc",rob2_loc)
    distance_bw1 = np.linalg.norm(current_loc - rob1_loc )
    distance_bw2 = np.linalg.norm(current_loc - rob2_loc )
    reached = np.linalg.norm(current_loc - goal )
    
    
    psValues = []
    for i in range(8):
        psValues.append(ps[i].getValue())

    # detect obstacles
    thre = 80
    front_obs = (psValues[0] >thre or psValues[7] >thre)
    right_obs = (psValues[2] > thre or psValues[1] >thre) 
    left_obs = (psValues[6]> thre or psValues[5] >thre )  
            
    leftSpeed = MAX_SPEED
    rightSpeed = MAX_SPEED
    
    if right_obs or left_obs or front_obs:
        if front_obs:
            # print("Turn left")
            leftSpeed = - MAX_SPEED
            rightSpeed = MAX_SPEED
        else:
            #Turn left
            # print("Straight")
            leftSpeed = MAX_SPEED
            rightSpeed = MAX_SPEED
        
        leftMotor.setVelocity(leftSpeed)
        rightMotor.setVelocity(rightSpeed)
        
    elif distance_bw1 <= 0.3 or distance_bw2 <= 0.3:
        print("speed increased,cross the robot")
        leftSpeed = 2* MAX_SPEED
        rightSpeed = 2* MAX_SPEED
        
    else:
        def line_equation(p, q):
            a = q[1] - p[1]
            b = p[0] - q[0]
            c = a*(p[0]) + b*(p[1])
            
            angle = np.degrees(math.atan(-a/b))
            return angle
        
    
        if (line_equation(goal,current_loc) - rotation) >= 3:  
            leftSpeed = - MAX_SPEED 
            rightSpeed = MAX_SPEED
        elif (line_equation(goal,current_loc) - rotation) <= -3:
            leftSpeed = MAX_SPEED 
            rightSpeed = - MAX_SPEED
        else:
            leftSpeed = MAX_SPEED 
            rightSpeed = MAX_SPEED
            
          
    if reached <= 0.15:
        leftSpeed = 0.0 
        rightSpeed = 0.0       
            
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)
    
    
    
            
   
        
    