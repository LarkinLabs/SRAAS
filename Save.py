import time
import datetime
import numpy as np
import os
from Queue import Queue
import cv2

Data_Save = Queue()
def Logger():
    now = datetime.datetime.now()
    print str(now)
    date = str("%d-%d-%d"% (now.year,now.month,now.day))
    t = str("%d-%d" % (now.hour,now.minute))
            
    Data_Path = str("Data/" + date+"/"+t)
    Xtion_Path = str(Data_Path+"/" + "Xtion")
    Zed_Path = str(Data_Path+"/" + "Zed")
    Map_Path = str(Data_Path+"/" + "Map")
    #create folders f not exist
    if not os.path.exists(Data_Path):
        os.makedirs(Data_Path)
    if not os.path.exists(Xtion_Path):
        os.makedirs(Xtion_Path)
    if not os.path.exists(Zed_Path):
        os.makedirs(Zed_Path)
    if not os.path.exists(Map_Path):
        os.makedirs(Map_Path)
    filename = str(Data_Path+"/"+t+".csv")
    
    f =open (filename, "a+")
    f.write(",time,Batt,encDist,encTheta,encX,encY,GPSY,GPSX,GPSSpeed,GPSHeading,GPSLon, GPSLat,accX,IMU_XPOS,IMU_YPOS,angX,angY,angZ,us1,us2,us3,us4,us5,us6\n")
    f.close()
    count =0;
    while True:
        now = datetime.datetime.now()
        t = time.time()#str("%d%d%d"%(now.hour,now.minute,now.second))
        Data = ""
        if not Data_Save.empty():
            try:
                Data = Data_Save.get()
                print "got Data"
            except:
                pass
        try:
            
            Batt,Encoder,GPS_data,Imu,Ultra,ZED,Xtion,Map= Data
            encDist, encTheta,encX,encY= Encoder
            GPSY =GPS_data[1]
            GPSX =GPS_data[0]
            GPSSpeed =GPS_data[2]
            GPSHeading =GPS_data[3]
            GPSLat = GPS_data[4]
            GPSLon = GPS_data[5]
            us1 = Ultra[0]
            us2 = Ultra[1]
            us3 = Ultra[2]
            us4 = Ultra[3]
            us5 = Ultra[4]
            us6 = Ultra[5]
            accX= Imu[0]
            accY= Imu[1]
            accZ= Imu[2]
            angX= Imu[3]
            angY= Imu[4]
            angZ= Imu[5]
            f =open (filename, "a+")
            f.write(str(("",t,Batt,encDist,encTheta,encX,encY,GPSY,GPSX,GPSSpeed,GPSHeading,GPSLat,GPSLon,accX,accY,accZ,angX,angY,angZ,us1,us2,us3,us4,us5,us6,""))+"\n")
            f.close()

            now = datetime.datetime.now()
            Nowtime = str("%d%d%d"%(now.minute,now.second,now.microsecond))
            count =count+1
            Xtion_file = Xtion_Path +"/"+ str(count)
            Zed_file = Zed_Path+"/"+str(count)
            Map_file = Map_Path+"/"+str(count)
            #np.save(Xtion_file,Xtion)
            #np.save(Zed_file, ZED)
            
            cv2.imwrite(str(Xtion_file+".png"), Xtion)
            cv2.imwrite(str(Zed_file+".png"), ZED)
            cv2.imwrite(str(Map_file+".png"), Map)
            
        except:
            pass
            
            
            

            
