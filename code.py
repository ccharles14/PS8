
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

speed_stop=9
speed_forward=10.08


'''//?-8.59>>>>>>>pas de sortie(alimente par la voiture) 
   //?>>>>>>>>reculer a la vitesse maximale 
   //9.6>>>>>>>>avancer a la vitesse minimale, valeur a tester '''


'''//11.2>>>>>>gauche 
   //8.8>>>>>>>droit 
   //7.2>>>>>>droite '''
pd.start(8.8)#direction=straight
pv.start(speed_stop)#speed=0
 
# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))
time.sleep(2)#wait for 2 sec before the start



 
# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):#if button "q" is pressed, quit the loop
               break 
      
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

        if distance>10:# if the area of 100cm in front of the car is empty, the car will analyse the traffic environment 
                
                        # grab the raw NumPy array representing the image, then initialize the timestamp
                        # and occupied/unoccupied text
                        image = frame.array
                        #first white line detection. This part treat the line on the bottom of the image 
                        image0=image.copy()
                        treat1=image0[400:480, 0:600]
                        gray1=cv2.cvtColor(treat1,cv2.COLOR_BGR2GRAY)# convert the bgr image to gray scale
                        blur1=cv2.GaussianBlur(gray1,(5,5),0)# gaussian blur
                        ret,thresh1=cv2.threshold(blur1,160,255,cv2.THRESH_BINARY)# threshold operation: keep the pixels light
                        cont,contours1,hierarchy=cv2.findContours(thresh1,1,cv2.CHAIN_APPROX_NONE) #find contours for the white object
                        c1x=300
                        c1y=0
                        if len(contours1)>0:
                            c1=max(contours1,key=cv2.contourArea)#find the biggest contour
                            M1=cv2.moments(c1)#calculate the moment 
                            if M1['m00']>0:
                                c1x=int(M1['m10']/M1['m00'])#cordinate of the biggest contour's centre geometric
                                c1y=int(M1['m01']/M1['m00'])
                            else:
                                c1x=300
                                c1y=0
                            cv2.line(image0,(c1x,0),(c1x,480),(255,0,0),1)#draw the lines to illustrate the centre
                            cv2.line(image0,(0,c1y+400),(600,c1y+400),(255,0,0),1)
                            cv2.drawContours(treat1,contours1,-1,(0,255,0),3)

                        #second white line detection. This part treat the line which is on the top of the first part
                        treat2=image0[300:400, 0:600]
                        gray2=cv2.cvtColor(treat2,cv2.COLOR_BGR2GRAY)# convert the bgr image to gray scale
                        blur2=cv2.GaussianBlur(gray2,(5,5),0)# gaussian blur
                        ret,thresh2=cv2.threshold(blur2,165,255,cv2.THRESH_BINARY)# threshold operation: keep the pixels light
                        cont,contours2,hierarchy=cv2.findContours(thresh2,1,cv2.CHAIN_APPROX_NONE) #find contours for the white object
                        c2x=300
                        c2y=0
                        if len(contours2)>0:
                            c2=max(contours2,key=cv2.contourArea)#find the biggest contour
                            M2=cv2.moments(c2)
                            if M2['m00']>0:
                                c2x=int(M2['m10']/M2['m00'])#cordinate of the biggest contour's centre geometric
                                c2y=int(M2['m01']/M2['m00'])
                            else:
                                c2x=300
                                c2y=0
                            cv2.line(image0,(c2x,0),(c2x,480),(0,255,0),1)#draw the lines to illustrate the centre
                            cv2.line(image0,(0,c2y+300),(600,c2y+300),(0,255,0),1)
                            cv2.drawContours(treat2,contours2,-1,(0,255,0),3)


                        if status=='move':
                                #if the line far is too crooked, the car will turn around with the max angle in advance
                                if c2x>480:
                                        pd.ChangeDutyCycle(7.2)
                                elif c2x<120:
                                        pd.ChangeDutyCycle(11.2)
                                #condition of the line close: 
                                else:
                                        if c1x>=80 and c1x<=220:
                                         pd.ChangeDutyCycle(9.8)
                                        elif c1x>=380 and c1x<=520:
                                         pd.ChangeDutyCycle(7.8)
                                        elif c1x>520:
                                         pd.ChangeDutyCycle(7.2)
                                        elif c1x<80:
                                         pd.ChangeDutyCycle(11.2)
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
                        lower_red1=np.array([0,50,50]) #choose the range of color which want to retain 
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
                        if len(contour1)>0:#if some red objects are found
                        
                                cmax1=max(contour1,key=cv2.contourArea)#find the biggest red object
                                (x,y),radius=cv2.minEnclosingCircle(cmax1)# calculate the smallest enclosing circle
                                center=(int(x)+200,int(y))
                                radius=int(radius)
                                cv2.circle(image0,center,radius,(0,255,0),3)#draw the enclosing circle
                        else:
                                
                                radius=0
                        if status=='stop':
                                if radius>25:

                                    time.sleep(3)
                                else:
                                    pv.ChangeDutyCycle(speed_forward)


                                    status='move'
                        else:
                                if radius>25:
                                    pv.ChangeDutyCycle(speed_stop)
                                    
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
                        
pv.stop()
pd.stop()
GPIO.cleanup()


