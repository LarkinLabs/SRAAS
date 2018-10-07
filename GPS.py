import serial
import math
from Queue import Queue
#http://journals.sagepub.com/doi/full/10.5772/62059

#GPS_Pos = Queue()

def GPS_RX(GPS):
    print "GPS"
    lat =0
    speed =0
    heading=0
    X=0
    Y=0
    i=0
    while 1:
        print "GPS"
        raw =  GPS.readline()
        data = map(str.strip,raw.split(','))
        #print data
        if data[0] == '$GPRMC':
            try:
                lat = float(data[3])
                lon = float(data[5])
            except:
                pass
            if not(data[7] == ''):
                speed = float(data[7])*1.852#Knots to km/h
                heading = float(data[8])
                X += speed * math.cos(math.radians(heading))
                Y += speed * math.sin(math.radians(heading))
                #Xls.append(X*10)
                #Yls.append(Y*10)
                plt.scatter(X,Y);
                plt.draw()
                fig.canvas.draw()        
                GPS_Pos.put( X,Y,speed , heading)
            else:
                GPS_Pos.put(lat,lon,speed,heading)
            
            
