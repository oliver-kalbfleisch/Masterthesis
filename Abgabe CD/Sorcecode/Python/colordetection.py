
from __future__ import print_function
from imutils.video.pivideostream import PiVideoStream
import numpy as np
import cv2
import imutils
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import argparse
import socket
from datetime import datetime



#HSV colorspace colors
#green= 60
#blue=120
#yellow=30
#senitivity=15 <-exmple for blue

#values for upper and lower boudnds are
#(color-senitivity,100,100) -> lower
#(color+senitivity,255,255) -> upper

#TODOS
#-> Write generic function with color value as input parameters
#Parameters for UPD Data -> TOFO make argparser or config file
UDP_IP="192.168.178.43"
UDP_PORT= 8888
#init variables
bg=None
aWeight=0.5 
numFrames=0 
def createUDPSender():
    sock= socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    return sock;
def sendTrackingData(socket,data):
    socket.sendto(bytes(data,"utf-8"),(UDP_IP,UDP_PORT))
    return;

sock=createUDPSender()
#
def calculateHSVColorValue(blue,green,red):
    
    color=np.uint8([[[blue,green,red]]])
    hsv=cv2.cvtColor(color,cv2.COLOR_BGR2HSV)
    return hsv;


def optimizeMask(mask):
    #blurr maks 
    ret_mask=cv2.GaussianBlur(mask,(9,9),3,3)
    # erode and dilate to remove high level noise
    ret_mask=cv2.erode(ret_mask,None,iterations=2)
    ret_mask=cv2.dilate(ret_mask,None,iterations=2)
    return ret_mask;
def findColorContours(mask,image):
    colorCnts=cv2.findContours(mask.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
    points="0 0 0"
    if len(colorCnts)>0:
        #find largest color occurence
        c=max(colorCnts,key=cv2.contourArea)
        rect=cv2.minAreaRect(c)
        (width,height)=rect[1]
        if width>=10 and height>=10:
            box= cv2.boxPoints(rect)
            points=cv2.boxPoints(rect)
            box=np.int0(box)
            cv2.drawContours(image,[box],0,(0,255,0),2)
    return points;
            
def detectRed(hsvImage,drawImage):
    #color Ranges for Red
    brightRedLower= (0,100,100)
    brightRedUpper= (10,255,255)
    darkRedLower=(160,100,100)
    darkRedUpper=(179,255,255)
    #generate Mask for color detection
    brightMask=cv2.inRange(hsvImage,brightRedLower,brightRedUpper)
    darkMask=cv2.inRange(hsvImage,darkRedLower,darkRedUpper)
    weightedMask=cv2.addWeighted(brightMask,1.0,darkMask,1.0,0.0)
    mask=optimizeMask(weightedMask)
    rect= findColorContours(mask,drawImage)
    return mask;
def detectGreen(hsvImage,drawImage):
    #set color space boundaries for green
    lowerGreen=(29,86,6)
    upperGreen=(64,255,255)
    #generate mask for green
    greenMask=cv2.inRange(hsvImage,lowerGreen,upperGreen)
    mask=optimizeMask(greenMask)
    #findColorContours(mask,drawImage)
    detectedRect= findColorContours(mask,drawImage)
    sendTrackingData(sock,detectedRect)
    return mask;
def detectBlue(hsvImage,drawImage):
    lowerBlue=(95,100,100)
    upperBlue=(145,255,255)
    #generate mask for blue
    blueMask=cv2.inRange(hsvImage,lowerBlue,upperBlue)
    mask=optimizeMask(blueMask)
    findColorContours(mask,drawImage)
    return mask;
def detectYellow(hsvImage,drawImage):
    #
    lowerYellow=(20,100,100)
    upperYellow=(45,255,255)
    #generate Mask for Yellow
    yellowMask=cv2.inRange(hsvImage,lowerYellow,upperYellow)
    mask=optimizeMask(yellowMask)
    findColorContours(mask,drawImage)
    return mask;
def detectOrange(hsvImage,drawImage):
    #
    lowerOrange=(0,100,100)
    upperOrange=(15,255,255)
    #generate mask for organe
    orangeMask=cv2.inRange(hsvImage,lowerOrange,upperOrange)
    mask=optimizeMask(orangeMask)
    findColorContours(mask,drawImage)
    return mask;
def calibrate(image):
    ret,corners= cv2.findChessboardCorners(cv2.cvtColor(image,cv2.COLOR_BGR2GRAY),(7,6),None)
    if ret == True:
        print("frame found")
        filename= datetime.now().strftime('%Y%m%d_%Hh%Mm%Ss%f')+'.jpg'
        cv2.imwrite('pose/sample_images/'+filename,image)
    return;

def run_avg(frame,aWeight):
    global bg
    if bg is None:
        bg=frame.copy().astype("float")
        return
    cv2.accumulateWeighted(frame,bg,aWeight)
    
def segment(frame,threshold=25):
    global bg
    #find abolute difference between background and curr frame
    diff=cv2.absdiff(bg.astype("uint8"),frame)
    #threshold diff image to get the foreground
    thresholded=cv2.threshold(diff,threshold,255,cv2.THRESH_BINARY)[1]
    #find contours from thresholded frame
    (_,cnts, _)=cv2.findContours(thresholded.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    #break condition if no contour can be found
    if len(cnts) == 0:
        return;
    else:
        #retrieve maximum contour
        segmented=max(cnts,key=cv2.contourArea)
        return(thresholded,segmented);
    
def detectHand(frame,numFrames):
    frame=imutils.resize(frame,width=700)
    frame=cv2.flip(frame,1)
    clone=frame.copy()
    (height,width)=frame.shape[:2]
    #take roi from selected frame
    roi=frame[top:bottom,right:left]
    #convert image to gray
    gray=cv2.cvtColor(roi,cv2.COLOR_BGR2GRAY)
    #blur to eliminate high frequency noise
    gray=cv2.GaussianBlur(gray,(7,7),0)
    if numFrames < 30:
        #calibrate the runnig average model for backgrounbd detection
        calc_avg(gray,aWeight)
    else:
        hand=segmentFrame(gray);
        if hand is not None:
            (thresholded,segmented)=hand
            cv2.drawContours(clone,[segemnted+(right,top)],-1,(0,0,255))
            cv2.imshow("Thresholded",thresholded)
    cv2.rectangle(clone,(left,top),(right,bottom),(0,255,0),2)
    numFrames+=1
    cv2.imshow("videoFeed",clone)
    
    return
#--------------------------------------------------------------------------------

#roi coordinates for hand detection
top,right,bottom,left= 10,150,400,590
videoStream=PiVideoStream().start()
#allow camera setup times
time.sleep(2.0)
while True:
    frame=videoStream.read()
    #camera calibration function
    #calibrate(frame)
    #hand detection function
    #detectHand(frame,numFrames)
    frame=imutils.resize(frame,width=700)
    frame=cv2.flip(frame,1)
    clone=frame.copy()
    (height,width)=frame.shape[:2]
    #take roi from selected frame
    roi=frame[top:bottom,right:left]
    #convert image to gray
    gray=cv2.cvtColor(roi,cv2.COLOR_BGR2GRAY)
    #blur to eliminate high frequency noise
    gray=cv2.GaussianBlur(gray,(7,7),0)
    if numFrames < 320:
        #calibrate the runnig average model for backgrounbd detection
        run_avg(gray,aWeight)
    else:
        hand=segment(gray);
        if hand is not None:
            (thresholded,segmented)=hand
            cv2.drawContours(clone,[segmented+(right,top)],-1,(0,0,255))
            cv2.imshow("Thresholded",thresholded)
        else:
            print("none")
    cv2.rectangle(clone,(left,top),(right,bottom),(0,255,0),2)
    numFrames+=1
    cv2.imshow("videoFeed",clone)
    #color detection 
    hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    hsv=cv2.GaussianBlur(hsv,(9,9),3,3)
    #detect red
    maskR=detectRed(hsv,frame)
    #detect blue
    #maskB=detectBlue(hsv,frame)
    #detect green
    #maskG=detectGreen(hsv,frame)
    #detect Yellow
    #maskY=detectYellow(hsv,frame)
    #detect orange
    #maskO=detectOrange(hsv,frame)
    #cv2.imshow("maskO",maskO)
    cv2.imshow("Frame",frame)
    cv2.imshow("maskR",maskR)
    #cv2.imshow("maskB",maskB)
    #cv2.imshow("maskG",maskG)
    #cv2.imshow("maskY",maskY)
    key=cv2.waitKey(1)&0xFF
    #clear the stream in preperation for the next frame
    #rawCapture.truncate(0)
    if key == ord("q"):
            break
 
cv2.destroyAllWindows()
videoStream.stop()
