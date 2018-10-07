import numpy as np
import cv2
from primesense import openni2#, nite2
from primesense import _openni2 as c_api
from Queue import Queue
Xtion_data = Queue()
## Path of the OpenNI redistribution OpenNI2.so or OpenNI2.dll
# Windows
#dist = 'C:\Program Files\OpenNI2\Redist\OpenNI2.dll'
# OMAP
#dist = '/home/carlos/Install/kinect/OpenNI2-Linux-ARM-2.2/Redist/'
# Linux
dist ="/home/chris/OpenNI-Linux-x64-2.2/Redist"

## Initialize openni and check
openni2.initialize(dist) #
if (openni2.is_initialized()):
    print "openNI2 initialized"
else:
    print "openNI2 not initialized"

## Register the device
dev = openni2.Device.open_any()

## Create the streams stream
depth_stream = dev.create_depth_stream()

## Configure the depth_stream -- changes automatically based on bus speed
#print 'Get b4 video mode', depth_stream.get_video_mode() # Checks depth video configuration
resX = 640
resY = 480
depth_stream.set_video_mode(c_api.OniVideoMode(pixelFormat=c_api.OniPixelFormat.ONI_PIXEL_FORMAT_DEPTH_1_MM, resolutionX=resX, resolutionY=resY, fps=30))

## Check and configure the mirroring -- default is True
# print 'Mirroring info1', depth_stream.get_mirroring_enabled()
depth_stream.set_mirroring_enabled(False)


## Start the streams
depth_stream.start()

## Use 'help' to get more info
# help(dev.set_image_registration_mode)


def get_depth(depth_stream):
    """
    Returns numpy ndarrays representing the raw and ranged depth images.
    Outputs:
        dmap:= distancemap in mm, 1L ndarray, dtype=uint16, min=0, max=2**12-1
        d4d := depth for dislay, 3L ndarray, dtype=uint8, min=0, max=255    
    Note1: 
        fromstring is faster than asarray or frombuffer
    Note2:     
        .reshape(120,160) #smaller image for faster response 
                OMAP/ARM default video configuration
        .reshape(240,320) # Used to MATCH RGB Image (OMAP/ARM)
                Requires .set_video_mode
    """
    while True:
        dmap = np.fromstring(depth_stream.read_frame().get_buffer_as_uint16(),dtype=np.uint16).reshape(resY,resX)  # Works & It's FAST
        d4d = np.uint8(dmap.astype(float) *255/ 2**12-1) # Correct the range. Depth images are 12bits
        d4d = cv2.cvtColor(d4d,cv2.COLOR_GRAY2RGB)
        # Shown unknowns in black
        d4d = 255 - d4d
        #filtering setup
        kernel1 = np.ones((2,2),np.uint8)
        kernel = np.ones((3,3),np.uint8)
        
        erode = cv2.erode(d4d,kernel1,iterations = 3)
        dilation = cv2.dilate(erode,kernel,iterations = 3)
        opening = cv2.morphologyEx(erode, cv2.MORPH_OPEN, kernel)
        closing = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel)
        Xtion_data.queue.clear()
        Xtion_data.put( (dmap, closing))#replace d4d with filtered.
#get_depth



