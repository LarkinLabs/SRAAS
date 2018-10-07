from threading import Thread
from Queue import Queue
import serial
import re
import time
import cv2
from SerialHand import *
BatteryQ = Queue()
RawData = Queue()
LastCommand = Queue()
Ports = Queue()
MCnt1 = Queue()
MCnt2 = Queue()
com = ''
def handle_data(data):
    #print(data)
    if not LastCommand.empty():
        com = LastCommand.get()
    else:
        com =""

    if com == '?V 2\r':
        try:
            Batt = float(str(data[2]+data[3]+data[4]))/10
            #print Batt
            #print (Batt, intrn)
            BatteryQ.queue.clear()
            BatteryQ.put(Batt)
        except:
            a=0
    elif com == '?C 01\r':
        #print (LastCommand.get())
        Mcnt1 = ''
        for i in range(2,len(data)):
            if data[i] != '\r':
                Mcnt1 +=data[i]
            
        
        MCnt1.queue.clear()
        MCnt1.put(Mcnt1)

    elif com == '?C 02\r':
        #print (LastCommand.get())
        Mcnt2 = ''
        for i in range(2,len(data)):
            if data[i] != '\r':
                Mcnt2 +=data[i]
        MCnt2.queue.clear()
        MCnt2.put(Mcnt2)
            #print (MCnt1)
    

    elif data == 'W':
        a=0
    else:
        #if message is not an expected result it might be a command message
        
        LastCommand.put(data)
    RawData.queue.clear()
    RawData.put(data)
        #print "Watchdog"
   

def read_from_port(ser):
    buff =""
    Ctime = time.time()
    matcherend = re.compile('\r')
    print ser
    while True:
        #print("test")
        reading = ""
        Ctime = time.time()  
        while ((time.time() - Ctime) < .1) and not matcherend.search(reading):
            reading += ser.read(1)
        
        handle_data(bytes(reading))
                  
def bytes_to_int(bytes):
    result = 0

    for b in bytes:
        result = result * 256 + int(b)

    return result

def Write_to_port(MotorController,CMD):
    LastCommand = CMD
    MotorController.write(CMD)


def Clear_Counters(MotorController):
  # Zero the counterinfo on the Motor controller.
  MotorController.write("!c 01 0\r")
  MotorController.write("!c 02 0\r")

