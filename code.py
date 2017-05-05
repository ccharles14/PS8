# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np
import RPi.GPIO as GPIO

status='stop'#initial status of the car: 'stop' or 'move'

# initialize the generation of signals
#The 2 different modes use the different maps of Pin 
#GPIO.setmode(GPIO.BOARD) #use mode <board> 
GPIO.setmode(GPIO.BCM) #use mode <bcm> 



GPIO.setup(12, GPIO.OUT)                   # pin 12: PWM signal of speed
GPIO.setup(19, GPIO.OUT)                   # pin 19: PWM signal of direction
pv = GPIO.PWM(12, 62.5)                    #frequence is 62.5Hz
pd = GPIO.PWM(19, 62.5)

#ultrasound sensor configure 
TRIG = 23               #ultralsound trigger 
ECHO = 24               #ultralsound echo 
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

'''//?-8.59>>>>>>>pas de sortie(alimente par la voiture) 
   //?>>>>>>>>reculer a la vitesse maximale 
   //8.6>>>>>>>>avancer a la vitesse minimale /9.4 si alimente par chargeur'''


'''//11.2>>>>>>gauche 
   //8.8>>>>>>>droit 
   //7.2>>>>>>droite '''
pd.start(8.8)#direction=straight
pv.start(9.0)#speed=0
#pv.start(8)#speed=0
 
# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))
 


time.sleep(3)#wait for 3 sec before the start



 
# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        GPIO.output(TRIG, False)
        time.sleep(.00001)
        GPIO.output(TRIG, True)
        time.sleep(.00001)
        GPIO.output(TRIG, False)

        while GPIO.input(ECHO) == 0:

            pulse_start = time.time()# record the trigger time

        while GPIO.input(ECHO) == 1:

            pulse_end = time.time()#record the echo time
        pulse_duration = pulse_end - pulse_start#duration between the emission and reception
        distance = pulse_duration*17150#distance=duration/2*34300cm/s

        distance = round(distance, 2)#keep 2 number after the decimal point

        if distance>100:# if the area of 100cm in front of the car is empty, the car will analyse the traffic environment 
                
                        # grab the raw NumPy array representing the image, then initialize the timestamp
                        # and occupied/unoccupied text
                        image = frame.array
                        #white line detection
                        image0=image.copy()
                        traite=image0[400:480, 0:600]
                        gray0=cv2.cvtColor(traite,cv2.COLOR_BGR2GRAY)# convert the bgr image to gray scale
                        blur=cv2.GaussianBlur(gray0,(5,5),0)# gaussian blur
                        
                        
                        ret,thresh=cv2.threshold(blur,170,255,cv2.THRESH_BINARY)# threshold operation: keep the pixels light

                        cont,contours,hierarchy=cv2.findContours(thresh,1,cv2.CHAIN_APPROX_NONE) #find contours for the white object
                        cx=300
                        if len(contours)>0:
                            c=max(contours,key=cv2.contourArea)#find the biggest contour
                            M=cv2.moments(c)
                            if M['m00']>0:
                                cx=int(M['m10']/M['m00'])#cordinate of the biggest contour's centre geometric
                                cy=int(M['m01']/M['m00'])
                            else:
                                cx=300
                                cy=0
                            cv2.line(image0,(cx,0),(cx,480),(255,0,0),1)#draw the lines to illustrate the centre
                            cv2.line(image0,(0,cy+400),(600,cy+400),(255,0,0),1)

                            cv2.drawContours(traite,contours,-1,(0,255,0),3)
                        if status=='move':    
                                if cx>=80 and cx<=220:
                                 pd.ChangeDutyCycle(9.8)
                                elif cx>=380 and cx<=520:
                                 pd.ChangeDutyCycle(7.8)
                                elif cx>520:
                                 pd.ChangeDutyCycle(7.2)
                                elif cx<80:
                                 pd.ChangeDutyCycle(10.4)
#                                if cx<200:
#                                    pd.ChangeDutyCycle(9.8)
#                                elif cx>400:
#                                    pd.ChangeDutyCycle(7.8)


                                else:
                                    pd.ChangeDutyCycle(8.8)
                  
                            
                        #end white line detecter

                        #red traffic sign detection
                        image_red=image[0:400,200:600]#choose a part of the image to treat

                        hsv=cv2.cvtColor(image_red,cv2.COLOR_BGR2HSV)#convert the bgr to hsv
                        lower_red1=np.array([0,50,50]) #choose the range of color which will will keep 
                        upper_red1=np.array([5,255,255])
                        lower_red2=np.array([178,50,50])
                        upper_red2=np.array([180,255,255])
                        mask1=cv2.inRange(hsv,lower_red1,upper_red1)
                        mask2=cv2.inRange(hsv,lower_red2,upper_red2)
                        mask=cv2.bitwise_or(mask1,mask2)
                        res=cv2.bitwise_and(image_red,image_red,mask=mask)#get the image filtred

                        
                        gray1=cv2.cvtColor(image_red,cv2.COLOR_BGR2GRAY)
                        ret,thresh1=cv2.threshold(gray1,127,255,0)
                        cont1,contour1,hierarchy=cv2.findContours(mask,1,cv2.CHAIN_APPROX_SIMPLE)
                        if len(contour1)>0:#if some red object is found
                        
                                cmax1=max(contour1,key=cv2.contourArea)#find the biggest red object
                                (x,y),radius=cv2.minEnclosingCircle(cmax1)# calculate the smallest enclosing circle
                                center=(int(x)+200,int(y))
                                radius=int(radius)
                                cv2.circle(image0,center,radius,(0,255,0),3)#draw the enclosing circle
                        else:
                                
                                radius=0
                        if status=='stop':
                                if radius>155:

                                    time.sleep(3)
                                else:
                                    pv.ChangeDutyCycle(9.48)


                                    status='move'
                        else:
                                if radius>155:
                                    pv.ChangeDutyCycle(9)
                                    
                                    status='stop'
                                    time.sleep(3)
                                
                                    

                        # show the frame
                        cv2.imshow("Frame", image0)
                        #cv2.imshow("Frame0", image)      
                        # clear the stream in preparation for the next frame
                        rawCapture.truncate(0)
                        key = cv2.waitKey(1) & 0xFF
                        # if the `q` key was pressed, break from the loop
                        if key == ord("q"):
                                break
        else:#if some obstacles is in front of the car and the distance is less than 100cm, the car will stop for 3 seconds
                
                        rawCapture.truncate(0)
                        status='stop'
                        key = cv2.waitKey(1) & 0xFF
                        pv.ChangeDutyCycle(9)
                        cv2.imshow("Frame", image0)
                        time.sleep(3)
                        if key == ord("q"):#if button "q" is pressed, quit the loop
                                break
pv.stop()
pd.stop()
GPIO.cleanup()

