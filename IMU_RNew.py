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
    
    time = 10/1000
      
    #    0  1    2     3   4    5    
    # accX,accY,accZ,angX,angY,angZ
    raw =IMU.readline()
    try:
        data =map(str.strip, raw.split(','))
    except:
        pass
        
       
    while True:
        try:
            raw =IMU.readline() 
            data =map(str.strip, raw.split(','))
            
            if len(data) > 4:
                accX = float(data[0])
                accY = float(data[1])
                accZ = float(data[2])
                gyrX = float(data[3])
                gyrY = float(data[4])
                gyrZ = float(data[5])
                
                IMU_Data.queue.clear()
                IMU_Data.put((accX,accY,accZ,gyrX,gyrY,gyrZ))
                
        except:
            pass
            
            
            
            
                        
