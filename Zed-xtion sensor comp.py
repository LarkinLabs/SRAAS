import cv2
import time
from evdev import *
import serial
import re
import csv
import numpy
import copy
from Odometery import * # import the odometery module for the roboteq motor driver
from Serialworker import * # import the Motorcontroller module
from Primesense import * # imput the Xition module
from GPS_R import * # import the GPS module
from Logitech import * #import HID Logitech module
from IMU_RNew import * # import IMU RX
from Ultra_R import * #import Ultrasonics
from oldOpencvPIL import * #import Zed Camera

from Save import *
from threading import Thread
from Queue import Queue
import matplotlib.pyplot as plt
from serial.tools import list_ports

#connect and test all ports
IMU_port = ""
Roboteq_port = ""
UltraSonics_port = ""
GPS_port = ""
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
Ports = Queue()
Ports.put([Roboteq_port,IMU_port,GPS_port,Ultrasonics_port])


        
#Set up serial for motor controller
MotorController = serial.Serial(port=Roboteq_port,baudrate=115200,timeout=10)
try:
  MotorController.flush()
except:
    print "motor controller disconnected"
    MotorController.close()
    MotorController.flush()
from SerialHand import *
    
#Set up serial for GPS
GPS = serial.Serial(port=GPS_port,baudrate=4800,timeout=10)
try:
  GPS.flush()
except:
    print "GPS disconnected"
    GPS.close()
    GPS.flush()
    
#Set up serial for Ultrasonics
Ultra = serial.Serial(port=Ultrasonics_port,baudrate=115200,timeout=10)
try:
  Ultra.flush()
except:
    print "Ultrasonics disconnected"
    Ultra.close()
    Ultra.flush()
#Set up serial for IMU
IMU = serial.Serial(IMU_port,9600)
try:
  IMU.flush()
except:
    print "IMU disconnected"
    IMU.close()
    IMU.flush()

MA=0
MB=0
Speedmul =5
# Distance variables
WheelDiam = 126
TrackWidth = 594.202898551
Circ = WheelDiam *3.14
EncCnt = 120
DistA =0
DistB =0 
# Set varibles for the motor controller



cv2.namedWindow("right")
#Clear the counters of the motors
Clear_Counters
#mythreads = []


# thread for Logitech controller
t = Thread(target=HID_Controller, args=(Get_Controller(),))
t.deamon = True
print t.start()

# thread for MotorController RX serial
thread2 = Thread(target=read_from_port, args=(MotorController,))
thread2.deamon = True
print thread2.start()

print "start GPS"
# thread for GPS RX serial
GPS_Thread = Thread(target=GPS_RX, args=(GPS,))
GPS_Thread.deamon = True
print GPS_Thread.start()
print "started"

print "start IMU"
# thread for GPS RX serial
IMU_Thread = Thread(target=IMU_RX, args=(IMU,))
IMU_Thread.deamon = True
print IMU_Thread.start()
print "started"

print "start Ultrasonics"
# thread for Ultrasonics

Ultra_Thread = Thread(target=Ultra_RX, args=(Ultra,))
Ultra_Thread.deamon = True
print Ultra_Thread.start()
print "started"

print "start ZedCamera"
# thread for ZedCamera
ZED_Thread = Thread(target=ZedCamera, args=(1,))
ZED_Thread.deamon = True
print ZED_Thread.start()
print "started"

print "start Xtion"
# thread for Xitoin 
Xtion_Thread = Thread(target=get_depth, args=(depth_stream,))
Xtion_Thread.deamon = True
print Xtion_Thread.start()
print "started"

print "start Save"
# thread for Xitoin 
Save_Thread = Thread(target=Logger)
Save_Thread.deamon = True
print Save_Thread.start()
print "started"
Clear_Counters(MotorController)
Ctime = time.time()

#main Loop
text=""
cnt1 = 0
cnt2 = 0
RX=0

MODE =0 #Mode: 0 = Manual, 1 = Wall avoidance, 2 = recording 
dist = 450 # distance to object
Theta = 0 # initialise theta for odomtery calc.
Xpos =0
Ypos =0
imuFirst = 0
IMU_X_Pos=0
IMU_Y_Pos=0
IMU_Theta=0
#GPS Variables
GPS_Y_Pos =0
GPS_X_Pos =0
GPS_Speed =0
GPS_Heading =0
GPS_FIX = 0
# Encoder Variables
DistAold =0
DistBold =0
Odtime =0
CenterDist=0
theta=0
#Initialising data sets for threaded data
GPS_data= (0,0,0,0,0,0)
ZED_Math = (0,0,0,0,0,0,0,0,0)
d4d = (0,0,0,0,0,0,0,0,0)
IMUData = (0,0,0,0,0,0)
UltraData = (0,0,0,0,0,0)
imgL = (0,0,0,0,0,0)
XI=0
# Create a black image
img = np.zeros((512,512,3), np.uint8)
imgScale = 20
RadA=0
RadB=0
cnt1old=0
cnt2old=0
d4d =[[0,0],[1,1]]
ZED =np.zeros((680,480,3), np.uint8)+1
Batt=0
Goto = [(3000,0),(3000,3000), (0,3000),(1500,1500),(0,0)]

#Positioning Variables 
Fused_X =0
Fused_Y=0
Fused_Theta=0
GotoX = "a"
GotoY = "a"
deltaX=0
deltaY=0
GotoTheta = 0;
deltaDist =0;
CurrentObstacle =0;
deltaTheta =0;
yawthresh=12
DistanceToTargetThreshold = 50 # in mm
#Avoidance Variables
AvoidDist = 700 # how much to drive past the object  900 for xtion
AvoidDeg =12
NewTheta=0
avL = 0
avR =0
GoSpeed = 50

#ZED Camera Sensing
ZedLine =190
while 1:
    #read Queues Data
    #Motor Controller Sensors
    #Battery
    if not BatteryQ.empty():
        try:
            Batt = BatteryQ.get()
        except:
            pass
     #Get Latest Motor A counter
    if not MCnt1.empty():
        try:       
            cnt1 = int(MCnt1.get())
        except:
            cnt1 =0
        if cnt1 != 0:
            DistA = int((cnt1* Circ)/EncCnt) # calc Motor B distance
    #Get latest Motor B counter
    if not MCnt2.empty():
        try:
            cnt2 = int(MCnt2.get())
        except:
            cnt2 =0
    if cnt2 !=0:
        DistB = int((cnt2* Circ) / EncCnt) #calc Motor B distance
        
    #HID Controller Mode for Decision making ( Manual, Auto, ETC...)
    if not mode.empty():
        try:
          MODE = mode.get()
        except:
            pass
        if MODE == 0:
            MA=0
            MB=0
    # Get Latest GPS Pos.
    if not GPS_Pos.empty():
        print "GPS:"
        try:
            GPS_data = GPS_Pos.get()
            print(GPS_data)
            GPS_Y_Pos =GPS_data[1]
            GPS_X_Pos =GPS_data[0]
            GPS_Speed =GPS_data[2]
            GPS_Heading =GPS_data[3]
            GPS_Lat = GPS_data[4]
            GPS_Lon = GPS_data[5]
            GPS_FIX = GPS_data[5]
        except:
            GPS_FIX = 0
            pass
    #Get image from Asus sensor
    if not Xtion_data.empty():
        try:
            Xition = Xtion_data.get()
            dmap,d4d = Xition
        except:
            pass
    #Get image from Zed sensor
    if not Zed_Disp.empty():
        try:
            #print "ZED"
            ZedData,imgL = Zed_Disp.get()
            ZED_Math = ZedData
            #print ZED_Math.shape
        except:
            pass
    #Get Data from Ultrasonics
    if not IMU_Data.empty():
        try:
            IMUData= IMU_Data.get()
            #print "imu:"
            #print IMUData
##            print "\n"
            XI = IMUData#*(10/100)*(10/1000)
            #print XI
        except:
            pass
   #Get Data from IMU
    if not Ultra_Data.empty():
        try:
            UltraData= Ultra_Data.get()
##            print "Ultra"
##            print UltraData
##            print "\n"
        except:
            pass

    ## get Motor commands from controller
    if not inputs.empty():
        try:
            event = inputs.get()
            print(event)
            MA,MB = event
        except:
            event = (MA,MB)
    # End of getting data from threads

    
        
    if((time.time() - Odtime) > .01):
        if cnt1 != cnt1old or cnt2 != cnt2old:
            A = (cnt1-cnt1old)*Circ/EncCnt
            B = (cnt2-cnt2old)*Circ/EncCnt
            cnt1old = cnt1
            cnt2old = cnt2
          # calculate the length of the arc traveled
            CenterDist = (A + B) / 2
     
       # calculate  change in angle
            CenterAngle = (A-B) / TrackWidth/2
       # add the change in angle to the previous angle
            Theta -= CenterAngle;
       # constrain _theta to the range 0 to 2 pi
            if (Theta > 2.0 * math.pi):
                Theta -= 2.0 * math.pi
            if (Theta < 0.0):
                Theta += 2.0 * math.pi
            
            
            
         
            
       # update x and y coordinates
            Xpos += CenterDist * math.cos(Theta)
            Ypos += CenterDist * math.sin(Theta)
            
            IMU_X_Pos+=CenterDist * math.cos(IMUData[5])
            IMU_Y_Pos+=CenterDist * math.sin(IMUData[5])

            if GPS_FIX > 3:
                #GPS based fusion
                Fused_Theta = Theta*.02 + IMUData[5] *.98 #Theta Fuse
                pass
            else:
                #non GPS based fusion
                Fused_Theta = Theta*.02 + IMUData[5] *.98 #Theta Fuse

            
            if (Fused_Theta > math.pi):
                Fused_Theta -= 2.0 * math.pi
            if (Fused_Theta <= -math.pi):
                Fused_Theta += 2.0 * math.pi
            Fused_X = math.cos(Fused_Theta) * CenterDist
            Fused_Y = math.sin(Fused_Theta) * CenterDist
           # print IMU_X_Pos,IMU_Y_Pos
            img = cv2.circle(img,(int(IMU_X_Pos/imgScale+250),int(IMU_Y_Pos/imgScale+250)), 1, (0,0,255), -1) # Encoder Image
##            img = cv2.circle(img,(int((GPS_X_Pos/100)+250),int((GPS_Y_Pos/100)+250)), 1, (0,255,0), -1) #GPS Img
##            img = cv2.circle(img,(int((IMU_X_Pos/100)+250),int((IMU_Y_Pos/100)+250)), 1, (120,120,0), -1) #imu Img
            #img = cv2.circle(img,(int((Fused_X/100)+250),int((Fused_Y/100)+250)), 1, (120,120,0), -1) #imu Img
            img = cv2.rectangle(img, (0, 0), (512,30), (0, 0, 0), 30) # rectagle to hide the old text
            
            DistAold = DistA
            DistBold = DistB
            
            IMUData = (IMUData[0]*1000, IMU_X_Pos,IMU_Y_Pos,IMUData[3],IMUData[4],IMUData[5]) #add IMU X Y pos to IMU DATA
        Odtime=time.time()
        
    # Save Data for testing
    
    if MODE == 2: # mode 2 is set in the logitech module
        Data_Save.queue.clear()
        Data_Save.put((Batt,(CenterDist,Theta,Xpos,Ypos),GPS_data,IMUData,UltraData,ZED_Math,d4d))

    #print MODE
     

    #print("%s,%s,%s,%s,%s" % (DistA,DistB,int(Xpos),int(Ypos), Theta))
    text=  "Bat: %s, X= %s Y= %s FT = %s T = %s D= %s" %(Batt,int(IMU_X_Pos/10), int(IMU_Y_Pos/10), int(math.degrees(Fused_Theta)), int(math.degrees(GotoTheta)), int(math.degrees(deltaTheta)))
    #print("%s,%s,%s,%s,%s" % (DistA,DistB,int(Xpos),int(Ypos), (Theta*57.2958)))
    if(GotoX != 'a'):
        text2 = " P: %s,%s  Sensor L:%s  R:%s    T:%s " % (int(GotoX),int(GotoY), avL,avR,int(math.degrees(NewTheta)))
    else:
        text2 = " P: %s,%s  Sensor L:%s  R:%s    T:%s" % (GotoX,GotoY, avL/10,avR/10,int(math.degrees(NewTheta)))
    f= open("TestData.csv","a+")
    f.write("%s,%s,%s,%s\r" % (DistA,DistB,int(Xpos),int(Ypos)))



    # apply text to the image
    height, width, channels = d4d.shape
    font                   = cv2.FONT_HERSHEY_SIMPLEX
    bottomLeftCornerOfText = (10,10)
    fontScale              = .5
    fontColor              = (255,255,255)
    lineType               = 2

    cv2.putText(img
                ,text,bottomLeftCornerOfText,font, 
    fontScale,fontColor,lineType)
    cv2.putText(img
                ,text2,(10,25),font, 
    fontScale,fontColor,lineType)
    print 'Center pixel is {}mm away'.format(dmap[119,159])
    cv2.imshow('Xtion', d4d)
    cv2.imshow('Map',img)
    
    try:
        ZED_Math_line = copy.deepcopy(ZED_Math)/255.

        cv2.imshow('ZED', ZED_Math)
        
        cv2.imshow('ZED Left', imgL)
    except:
        pass
    
    key = cv2.waitKey(1)
    if key == 27:
        break
    
      
      #Drive robot forward one meter ############
 #   if DistA < 1000:
 #     MA = 100
 #   else:
 #     MA = 0
 #   if DistB < 1000:
 #     MB = 100
 #   else:
 #     MB =0
##
 
   
    #print ZED_Math.shape#376,672
    if MODE == 1:
       # get new waypoint from list if available
        if(GotoX == 'a' and GotoY=='a'):
            if len(Goto)>0:
                temp = Goto.pop(0)
                GotoX = temp[0]
                GotoY = temp[1]
                #add new point to the map
                img = cv2.circle(img,(int(GotoX/imgScale+250),int(GotoY/imgScale+250)), 3, (0,255,0), -1)
        
        
        #if new waypoint was loaded get new point to point yaw, / distance
                #find Delta Yaw
        if not GotoX == "a":
            GotoTheta = math.atan2(GotoY-IMU_Y_Pos,GotoX-IMU_X_Pos)
            deltaX = GotoX - IMU_X_Pos 
            deltaY = GotoY-IMU_Y_Pos
        else:
            GotoTheta = Fused_Theta
            deltaX = 0 
            deltaY = 0  
        deltaDist = math.sqrt(deltaY**2+deltaX**2)
        deltaTheta = (GotoTheta-Fused_Theta)

        if deltaTheta > math.pi:
            deltaTheta -= 2*math.pi
        if deltaTheta < -math.pi:
            deltaTheta += 2.0 * math.pi
        
        #print deltaX,deltaY,deltaDist,math.degrees(Fused_Theta),math.degrees(GotoTheta)
        
        #if delta yaw less then turning angle threshold turn the robot on the spot
        if  (deltaTheta  < -math.radians(yawthresh) ) :
            MA =GoSpeed
            MB =-GoSpeed
            print "go Left"
        elif (deltaTheta > math.radians(yawthresh) ) :
            print "Go Right"
            MA = -GoSpeed
            MB = GoSpeed
        else:
            #else go forward towards the point
            if  deltaDist >50:
                MA = GoSpeed
                MB = GoSpeed
            else:
                MA = 0
                MB = 0
                GotoX = "a"
                GotoY = "a"
            #if robot is verring off course correct.
            if  (deltaTheta  < -math.radians(1) ) :
                MA +=10
                MB -=10
                print "go Left"
            elif (deltaTheta > math.radians(1) ) :
                print "Go Right"
                MA -= 10
                MB += 10

        # distance to point is less than DistanceToTargetThreshold go forward
        if (deltaDist <= DistanceToTargetThreshold):
            
            if len(Goto)>0:
                if CurrentObstacle==1:
                    CurrentObstacle=0
                else:
                    temp = Goto.pop(0)
                    GotoX = temp[0]
                    GotoY = temp[1]
                    img = cv2.circle(img,(int(GotoX/imgScale+250),int(GotoY/imgScale+250)), 3, (0,255,0), -1)
        
            else:
                GotoX = "a"
                GotoY = "a"
            
        print len(Goto)  



#SimpleAvoidance()

#xtionSensor
##        avL = 0
##        avR =0
##        for i in range(0,200):
##            L1 = dmap[330,i]#320 was working
##
##            if L1 <200:
##                L1=dist+1
##            avL +=L1
##        for i in range(439,639):
##            R1 = dmap[330,i]#320 was working
##            if R1 <200:
##                R1=dist+1
##
##            avR +=R1
##        avL = (avL / 200)
##        avR = (avR /200)

       # ZED_Math
        avL = 0
        avR =0
        for i in range(100,300):
            L1 = ZED_Math[ZedLine,i]#320 was working
            #L2 = dmap[340,i]#320 was working
            if L1 <200:
                L1=dist+1
##            if L2 <200:
##                L2=dist+1
            avL +=L1
        for i in range(379,579):
            R1 = ZED_Math[ZedLine,i]#320 was working
            #R2 = dmap[340,i]#320 was working
            if R1 <200:
                R1=dist+1
##            if R2 <200:
##                R2=dist+1
            avR +=R1
        avL = (avL / 200)
        avR = (avR /200)
        # if object detected draw object in map
        if avR < dist or avL < dist:
            img = cv2.circle(img,(int((IMU_X_Pos+dist*math.cos(Fused_Theta))/imgScale+250),int((IMU_Y_Pos+dist*math.sin(Fused_Theta))/imgScale+250)), 2, (255,0,255), -1)

    
        if avR < dist and avL < dist:
            #create new way point to go around the object
            NewTheta = Fused_Theta - math.radians(AvoidDeg*4)
            if NewTheta > math.pi:
                NewTheta -= 2.0*math.pi
            if NewTheta <= -math.pi:
                NewTheta += 2.0*math.pi
            NewX = math.cos(NewTheta)*AvoidDist+IMU_X_Pos
            NewY = math.sin(NewTheta)*AvoidDist+IMU_Y_Pos
            if CurrentObstacle ==0:
                Goto.insert(0,(GotoX,GotoY)) # place existing point back into the list.
                CurrentObstacle =1
            
            GotoX = NewX
            GotoY = NewY
            #create new way point to go around the object
            MA = -40
            MB = -15
        elif avR < dist:           
            #create new way point to go around the object
            
            #create new way point to go around the object
            NewTheta = Fused_Theta + math.radians(AvoidDeg)
            if NewTheta > math.pi:
                NewTheta -= 2.0*math.pi
            if NewTheta <= -math.pi:
                NewTheta += 2.0*math.pi
            NewX = math.cos(NewTheta)*AvoidDist+IMU_X_Pos
            NewY = math.sin(NewTheta)*AvoidDist+IMU_Y_Pos
            if CurrentObstacle ==0:
                Goto.insert(0,(GotoX,GotoY)) # place existing point back into the list.
                CurrentObstacle =1       
            #templist.append((NewX,NewY)) # place new position in the Goto List
            GotoX = NewX
            GotoY = NewY
            MA = -30
            MB = 30
            print "Left"
        elif avL < dist:
            #create new way point to go around the object
            NewTheta = Fused_Theta - math.radians(AvoidDeg)
            if NewTheta > math.pi:
                NewTheta -= 2.0*math.pi
            if NewTheta <= -math.pi:
                NewTheta += 2.0*math.pi
            NewX = math.cos(NewTheta)*AvoidDist+IMU_X_Pos
            NewY = math.sin(NewTheta)*AvoidDist+IMU_Y_Pos
            if CurrentObstacle ==0:
                Goto.insert(0,(GotoX,GotoY)) # place existing point back into the list.
                CurrentObstacle =1
            #templist.append((NewX,NewY)) # place new position in the Goto List
            GotoX = NewX
            GotoY = NewY
            MA = 30
            MB = -30
            
        

         
    if MA >= 0 :
      if MA <10:
            MA = int(bytes(MA))
    if MB >= 0:
            MB = int(bytes(MB))
    CMD = str("!M "+ str(MA*Speedmul)+ " "+ str(MB*Speedmul)+ "\r")
    #print CMD
    if((time.time() - Ctime) > 0.001):
        MotorController.write(CMD)
        #print CMD
        #specific commands to get encoder counts and Batter voltage for Roboteq SDC2160
        MotorController.write('?C 01\r')
        MotorController.write('?V 2\r')
        MotorController.write('?C 02\r')
        Ctime = time.time()
        
cv2.destroyAllWindows()

