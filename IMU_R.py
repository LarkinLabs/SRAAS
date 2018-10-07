import serial
import time
import math
from Queue import Queue
## magic Kalman https://github.com/rlabbe/filterpy
import numpy as np
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise


IMU_Data = Queue()

def IMU_RX(IMU):
    timeOld =time.time()
    XtrigAccOld =0
    XtrigVel=0
    XtrigPos=0
    threshold = 0.02

      
    #    0       1        2         3     4    5    6    7    8    9    10      11      12
    # accPitch,accRoll,kalPitch,kalRoll,accX,accY,accZ,gyrX,gyrY,gyrZ,Xoffset,Yoffset,Zoffset
    raw =IMU.readline() 
    data =map(str.strip, raw.split(','))

        
       
    gyroY =0
    vel=0
    velold=0
    dt=0.002
    timeold = 0.0001
    while True:
        
        raw =IMU.readline() 
        data =map(str.strip, raw.split(','))
        
        if len(data) > 11:
            accPitch = float(data[0])
            accRoll = float(data[1])
            kalPitch = float(data[2])
            kalRoll = float(data[3])
            accX = float(data[4])
            accY = float(data[5])
            accZ = float(data[6])
            gyrX = float(data[7])
            gyrY = float(data[8])
            gyrZ = float(data[9])
            XOffset = float(data[10])
            YOffset = float(data[11])
            ZOffset = float(data[12])

            XOS = accX-XOffset
            YOS = accY-YOffset
            ZOS = accZ-ZOffset
            
            
            mag = math.sqrt(math.pow(XOS,2)+math.pow(YOS,2)+math.pow(ZOS,2))
            XG = mag - YOS+ZOS
            gyroY += gyrY * dt
            pitchOfset = math.sin(kalPitch*(3.14/180))*mag
            if ( (pitchOfset > 0.01) and (pitchOfset < -0.01) ):
                vel =0
            else:
                vel += (XOS+pitchOfset *dt)
                
            IMU_Data.put((accPitch,accRoll,kalPitch,kalRoll,accX,accY,accZ,gyrX,gyrY,gyrZ,XOffset,YOffset,ZOffset,dt))
            G = (vel-velold) / dt
            velold = vel
            
            timeOld = time.time()
            
                        
