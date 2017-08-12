import cv2
import numpy as np
import time
import serial
import threading

positionDate = '065065'

port = '/dev/ttyUSB0'
ser = serial.Serial(port, 9600)


def recv(serial):
    while True:
        data=ser.read(1)
        if data=="":
            continue
        while 1:
            n=ser.inWaiting()
            #print n
            if n>0:
                data+=ser.read(n)
                time.sleep(0.1)
            else:
                break
        return data

blackLower=np.array([0,0,0],dtype='uint8')
blackUpper=np.array([150,150,150],dtype='uint8')
whiteLower=np.array([220,220,220],dtype='uint8')
whiteUpper=np.array([255,255,255],dtype='uint8')
kernel=cv2.getStructuringElement(cv2.MORPH_RECT,(3,3))
kernel_1=cv2.getStructuringElement(cv2.MORPH_RECT,(5,5))
camera=cv2.VideoCapture(0)
while True:
	success,frame=camera.read()
	if not success:
		break
	black=cv2.inRange(frame,blackLower,blackUpper)
	black=cv2.GaussianBlur(black,(3,3),0)
#	black=cv2.erode(black,kernel)
#	black=cv2.dilate(black,kernel)
	black=cv2.morphologyEx(black,cv2.MORPH_OPEN,kernel_1)
	
	(_,cnts,_)=cv2.findContours(black.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
	if len(cnts)>0:
		cnt=sorted(cnts,key=cv2.contourArea,reverse=True)[0]
		rect=np.int32(cv2.boxPoints(cv2.minAreaRect(cnt)))
#		print rect
		cv2.drawContours(frame, [rect], -1, (0, 255,0), 2)
		x,y,w,h=cv2.boundingRect(cnt)
#		pts1=np.float32([[rect[1,0],rect[1,1]],[rect[2,0],rect[2,1]],[rect[0,0],rect[0,1]],[rect[3,0],rect[3,1]]])
		pts1=np.float32([[x,y],[x+w,y],[x,y+h],[x+w,y+h]])
		pts2=np.float32([[0,0],[650,0],[0,650],[650,650]])
		M=cv2.getPerspectiveTransform(pts1,pts2)
		dst=cv2.warpPerspective(frame,M,(650,650))
		white=cv2.inRange(dst,whiteLower,whiteUpper)
		white=cv2.GaussianBlur(white,(3,3),0)
		(_,contours,_)=cv2.findContours(white.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
		if len(contours) > 0:
			contour=sorted(contours,key=cv2.contourArea,reverse=True)[0]
			rect=np.int32(cv2.boxPoints(cv2.minAreaRect(contour)))
#			print rect
			if abs(rect[2,0]-rect[1,0])*abs(rect[1,1]-rect[0,1]) > 10:
				cv2.drawContours(dst, [rect], -1, (0, 255, 0), 2)
				e=rect[0,0]+abs(rect[1,0]-rect[2,0])/2.0
				f=rect[1,1]+abs(rect[0,1]-rect[1,1])/2.0
				print "detected!"
				e=int(e/0.6)
				f=int(f/0.6)
				if e<10:
					m='0'*2+'{}'.format(e)
				if e>=10 and e<100:
					m='0'+'{}'.format(e)
				if e>=100:
					m='{}'.format(e)
				if f<10:
					n='0'*2+'{}'.format(f)
				if f>=10 and f<100:
					n='0'+'{}'.format(f)
				if f>=100:
					n='{}'.format(f)
		
				positionDate ='s'+m+n	

				print positionDate
			
				ser.write(positionDate)

				print time.time()
			else:
				print "No detected!"
		else:
			print "No detected!"
		for a in range(1000):
			b=a*0.65
			cv2.line(dst,(b,0),(b,10),(0,0,255),3)
		for c in range(1000):
			d=c*0.65
			cv2.line(dst,(0,d),(10,d),(0,0,255),3)		
	else:
		print "No detect"
	if cv2.waitKey(1) & 0xFF == ord("q"):
		break
	cv2.imshow('balance_ball',frame)
	cv2.imshow('bianhuan',dst)
	cv2.imshow('thresh',black)
camera.release()
cv2.destroyAllWindows()
