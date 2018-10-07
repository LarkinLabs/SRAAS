import numpy as np
import cv2
from matplotlib import pyplot as plt
import Queue
import time

Zed_Disp = Queue.Queue()

def ZedCamera(Cam):
    ## From ZedCamera conffile
    #[LEFT_CAM_VGA]
    Lfx = 350.494
    Lfy = 350.494
    Lcx = 331.039
    Lcy = 189.714
    Lk1 = -0.174488
    Lk2 = 0.027323
    Lk3=0##

    #[RIGHT_CAM_VGA]
    Rfx = 349.663
    Rfy = 349.663
    Rcx = 335.323
    Rcy = 189.551
    Rk1 = -0.175561
    Rk2 = 0.0269139
    Rk3=0##
    vc = cv2.VideoCapture(Cam)

    #stereo
    Baseline = 63
    CV = 0.00281173
    RX = 0.0031709
    RZ = -0.000340478

    R =np.array([[0.9999959891308698, 0.0003449348247654698, 0.002811181626984763],
     [-0.0003360191236988869, 0.9999949147414725, -0.00317136915914702],
     [-0.002812261247064596, 0.003170411828413505, 0.9999910197974362]])
    # wvga 1344x376 focal ~0.008mm, 2560x720
    retw = vc.set(3,1344);
    reth = vc.set(4,376);

    CameraMatrixL = np.array([[Lfx, 0, Lcx],[ 0, Lfy, Lcy],[0, 0, 1]])

    CameraMatrixR = np.array([[Rfx, 0, Rcx],[ 0, Rfy, Rcy],[0, 0, 1]])

    distCoeffsL = np.array([[Lk1], [Lk2], [0], [0], [0]])
    distCoeffsR = np.array([[Rk1], [Rk2], [0], [0], [0]])

    T = np.array([[63],[0],[RZ]])

    if vc.isOpened(): # try to get the first frame
        rval, frame = vc.read()
    else:
        rval = False
    # disparity settings
    stereo = cv2.StereoBM_create(numDisparities=16, blockSize=5)
    #stereoRectify(cameraMatrix_left, distCoeffs_left, cameraMatrix_right, distCoeffs_right, image_size, R, T,R1, R2, P1, P2, Q, cv::CALIB_ZERO_DISPARITY, 0, image_size);
    h,w = frame.shape[:2]
    size = (w/2,h)
    #stereoRectify(cameraMatrix_left, distCoeffs_left, cameraMatrix_right, distCoeffs_right, image_size, R, T,R1, R2, P1, P2, Q, cv::CALIB_ZERO_DISPARITY, 0, image_size);
    R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(CameraMatrixL,distCoeffsL,CameraMatrixR,distCoeffsR,size,R, T)
    #(cameraMatrix_left, distCoeffs_left, R1, P1, image_size)
    map_left_x, map_left_y = cv2.initUndistortRectifyMap(CameraMatrixL, distCoeffsL, R1, P1,size,cv2.CV_32F)
    #initUndistortRectifyMap(cameraMatrix_right, distCoeffs_right, R2, P2, image_size, CV_32FC1, map_right_x, map_right_y);
    map_right_x, map_right_y = cv2.initUndistortRectifyMap(CameraMatrixR, distCoeffsR, R2, P2,size,cv2.CV_32F)
    cameraMatrixL = P1
    cameraMatrixR = P2
    #print size

    ## filter
    # FILTER Parameters
    lmbda = 80000
    sigma = 1.2
    visual_multiplier = 1.0
    # SGBM Parameters -----------------
    window_size = 3                     # wsize default 3; 5; 7 for SGBM reduced size image; 15 for SGBM full size image (1300px and above); 5 Works nicely
     
    left_matcher = cv2.StereoSGBM_create(
        minDisparity=0,
        numDisparities=32,             # max_disp has to be dividable by 16 f. E. HH 192, 256
        blockSize=5,
        P1=8 * 3 * window_size ** 2,    # wsize default 3; 5; 7 for SGBM reduced size image; 15 for SGBM full size image (1300px and above); 5 Works nicely
        P2=32 * 3 * window_size ** 2,
        disp12MaxDiff=1,
        uniquenessRatio=15,
        speckleWindowSize=0,
        speckleRange=2,
        preFilterCap=63,
        mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
    )

    right_matcher = cv2.ximgproc.createRightMatcher(left_matcher)
    wls_filter = cv2.ximgproc.createDisparityWLSFilter(matcher_left=left_matcher)
    wls_filter.setLambda(lmbda)
    wls_filter.setSigmaColor(sigma)

    while rval:
        height,width, channels = frame.shape
        
        
        imgL= frame[:,0:width/2,:]
        imgR = frame[:,width/2:width,:]
        grayL = cv2.cvtColor(imgL, cv2.COLOR_BGR2GRAY)
        grayR = cv2.cvtColor(imgR, cv2.COLOR_BGR2GRAY)
        dh, dw  = grayR.shape
        center = (dw/2, dh/2)
      
        ## Remap images
     
        RectImgL =  cv2.remap(imgL,map_left_x, map_left_y, 3)#cv2.cvtColor( xxx,cv2.COLOR_BGR2GRAY)
        RectImgR =  cv2.remap(imgR,map_right_x, map_right_y, 3)#cv2.cvtColor( XXX,cv2.COLOR_BGR2GRAY)
        
        displ = left_matcher.compute(RectImgR, RectImgL)  # .astype(np.float32)/16
        dispr = right_matcher.compute(RectImgL, RectImgR)  # .astype(np.float32)/16
        displ = np.int16(displ)
        dispr = np.int16(dispr)
        filteredImg = wls_filter.filter(displ, RectImgL, None, dispr)  # important to put "imgL" here!!!
        filt = filteredImg
        filteredImg = cv2.normalize(src=filteredImg, dst=filteredImg, beta=0, alpha=255, norm_type=cv2.NORM_MINMAX);
        filteredImg = np.uint8(filteredImg)
        #cv2.imshow('Disparity Map', filteredImg)
     
        filtCorrected = filteredImg
            
        #cv2.imshow('Disparity Map', np.int8(filtCorrected))
        #print((filtCorrected[dh/2,dw/2])) #63 base distance ofcameras 0.008mm is focal lenth estimate
        #Eprint "   ,   " 
        #print((63*2.8)/ (filt[dh/2,dw/2])) #63 base distance ofcameras 0.008mm is focal lenth estimate
        Zed_Disp.queue.clear()
        Zed_Disp.put(filtCorrected)
        rval, frame = vc.read()
        
        #time.sleep(.2)
    cv2.destroyAllWindows()
    vc.release()
    

