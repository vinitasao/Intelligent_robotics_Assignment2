from controller import Motor, Supervisor
import numpy as np
import math
TIME_STEP = 64
MAX_SPEED = 1

goal = np.array([-0.67, 0.88])
# create the Robot instance.
robot = Supervisor()
gps = robot.getDevice('gps')
gps.enable(TIME_STEP)
# get a handler to the motors and set target position to infinity (speed control)
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))

object = robot.getFromDef('rob2') 
rot = object.getField("rotation")#get rotation

# set up the motor speeds at 10% of the MAX_SPEED.
leftMotor.setVelocity(MAX_SPEED)
rightMotor.setVelocity(MAX_SPEED)

while robot.step(TIME_STEP) != -1:
        gps_value = gps.getValues()
        current_loc = np.array(gps_value[:2])
        rotation = np.degrees(rot.getSFRotation()[3])
        def line_equation(p, q):
            a = q[1] - p[1]
            b = p[0] - q[0]
            c = a*(p[0]) + b*(p[1])
            
            angle = np.degrees(math.atan(-a/b))
            return angle
        
        if -182 <= (line_equation(goal,current_loc) - rotation) <= -178:  
            leftSpeed = MAX_SPEED 
            rightSpeed = MAX_SPEED
        else:
        
            if (line_equation(goal,current_loc) - rotation) > -178:
                leftSpeed = - MAX_SPEED 
                rightSpeed =  MAX_SPEED
            else:
                leftSpeed = MAX_SPEED 
                rightSpeed = -MAX_SPEED
        leftMotor.setVelocity(leftSpeed)
        rightMotor.setVelocity(rightSpeed)