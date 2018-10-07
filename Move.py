def MoveRobot():
    # move to position
        if(GotoX == 'a' and GotoY=='a'):
            if len(Goto)>0:
                temp = Goto.pop(0)
                GotoX = temp[0]
                GotoY = temp[1]
                
                
        
        #turn the robot
        
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
        
        
        if  (deltaTheta  < -math.radians(yawthresh) ) :
            MA =GoSpeed
            MB =-GoSpeed
            print "go Left"
        elif (deltaTheta > math.radians(yawthresh) ) :
            print "Go Right"
            MA = -GoSpeed
            MB = GoSpeed
        else:
            if  deltaDist >50:
                MA = GoSpeed
                MB = GoSpeed
            else:
                MA = 0
                MB = 0
                GotoX = "a"
                GotoY = "a"
            if  (deltaTheta  < -math.radians(1) ) :
                MA +=10
                MB -=10
                print "go Left"
            elif (deltaTheta > math.radians(1) ) :
                print "Go Right"
                MA -= 10
                MB += 10

##               
        if (deltaDist <=50):
            
            if len(Goto)>0:
                if CurrentObstacle==1:
                    CurrentObstacle=0
                else:
                    temp = Goto.pop(0)
                    GotoX = temp[0]
                    GotoY = temp[1]
                
            else:
                GotoX = "a"
                GotoY = "a"
            
        print len(Goto)  

