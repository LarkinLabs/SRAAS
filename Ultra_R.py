import serial
import time
import math
from Queue import Queue
## magic Kalman https://github.com/rlabbe/filterpy
import numpy as np
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
from serial.tools import list_ports


for port in list(list_ports.comports()):
    print port.description
    if port.description == "USB2.0-Serial":
        print "IMU"
        print port.name
        print "\n"
        IMU_port = str("/dev/"+port.name)
        
    if port.description == "Motor Controller":
        print "Roboteq"
        print port.name
        print "\n"
        Roboteq_port = str("/dev/"+port.name)
    if port.description == "USB Serial":
        print "Teensy"
        print port.name
        print "\n"
        Ultrasonics_port = str("/dev/"+port.name)
    if port.description == "USB-Serial Controller D":
        print "GPS"
        print port.name
        print "\n"
        GPS_port = str("/dev/"+port.name)

Ultra_Data = Queue()    
def Ultra_RX(Port):
    #    0       1        2         3     4      5
    # sensor1,sensor2,sensor3,sensor4,sensor5,sensor6
    while True:
        raw =Port.readline() 
        data =map(str.strip, raw.split(','))
        if len(data) > 4:
            S1 = float(data[0])
            S2 = float(data[1])
            S3 = float(data[2])
            S4 = float(data[3])
            S5 = float(data[4])
            S6 = float(data[5])
            Ultra_Data.queue.clear()
            Ultra_Data.put((S1,S2,S3,S4,S5,S6))
        

