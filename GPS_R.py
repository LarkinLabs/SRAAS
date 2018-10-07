import serial
import math
from Queue import Queue
#http://journals.sagepub.com/doi/full/10.5772/62059

GPS_Pos = Queue()

def GPS_RX(GPS):
    print "GPS"
    lat =0
    speed =0
    heading=0
    X=0
    Y=0
    i=0
    FIX =0
    first = -1;

    while True:
        try:
            raw =  GPS.readline()
            data = map(str.strip,raw.split(','))
        except:
            print "Gps Read error"
            False
            pass
        
        #print data
        if data[0] == '$GPRMC' and not FIX <2: #get data if fix is at least 2D
            try:
                lat = float(data[3])
                lon = float(data[5])
            except:
                pass
            if not(data[7] == ''):
                speed = float(data[7]) # KM/H
                Dist = ((speed/3.6)*1000) # remove time and convert to MM  KMh = Ms*3.6 @ 1 Hz: dist = (kmh/3.6)/T  : 1.1 = (4/3.6)/1  
                if first == -1:
                    first = math.radians(float(data[8]))
                heading = math.radians(float(data[8]))+first  # offset the degree from north to be zero
                if heading > 2.0*3.14:
                    heading -=2.0*3.14 ## constrain Heading between 0 and 360
                if heading <2.0*3.14:
                    heading +=2.0*3.14
                X += Dist * math.cos(heading)
                Y += Dist * math.sin(heading)
                #Xls.append(X*10)
                #Yls.append(Y*10)
            GPS_Pos.put((X,Y,speed, heading,lat,lon,FIX)) 
        elif data[0] == '$GPGSA':
            FIX = int(data[2])
                
        
            
               
            
            
