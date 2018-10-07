import math
def WheelOdometry(DistA, DistB, x, y,theta, WheelDist):
    

 # calculate the length of the arc traveled
    CenterDist = (DistA + DistB) / 2.0
 
   # calculate  change in angle
    CenterAngle = (DistA - DistB) / WheelDist
   # add the change in angle to the previous angle
    theta += CenterAngle;
   # constrain _theta to the range 0 to 2 pi
    if (theta > 2.0 * math.pi):
        theta -= 2.0 * math.pi
    if (theta < 0.0):
        theta += 2.0 * math.pi
 
   # update Colin's x and y coordinates
    x += CenterDist * math.cos(theta)
    y += CenterDist * math.sin(theta)

    return x, y, theta
