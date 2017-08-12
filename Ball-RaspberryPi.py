import cv2
import numpy as np
import time
import serial
import threading
import RPi.GPIO as GPIO

# Define GPIO to LCD mapping
LCD_RS = 7
LCD_E  = 8
LCD_D4 = 25
LCD_D5 = 24
LCD_D6 = 23
LCD_D7 = 18

# Define some device constants
LCD_WIDTH = 16    # Maximum characters per line
LCD_CHR = True
LCD_CMD = False

LCD_LINE_1 = 0x80 # LCD RAM address for the 1st line
LCD_LINE_2 = 0xC0 # LCD RAM address for the 2nd line

# Timing constants
E_PULSE = 0.0005
E_DELAY = 0.0005

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)       # Use BCM GPIO numbers
GPIO.setup(LCD_E, GPIO.OUT)  # E
GPIO.setup(LCD_RS, GPIO.OUT) # RS
GPIO.setup(LCD_D4, GPIO.OUT) # DB4
GPIO.setup(LCD_D5, GPIO.OUT) # DB5
GPIO.setup(LCD_D6, GPIO.OUT) # DB6
GPIO.setup(LCD_D7, GPIO.OUT) # DB7


positionDate = '500500'

port = '/dev/ttyUSB0'
ser = serial.Serial(port, 9600)

def lcd_init():
  # Initialise display
  lcd_byte(0x33,LCD_CMD) # 110011 Initialise
  lcd_byte(0x32,LCD_CMD) # 110010 Initialise
  lcd_byte(0x06,LCD_CMD) # 000110 Cursor move direction
  lcd_byte(0x0C,LCD_CMD) # 001100 Display On,Cursor Off, Blink Off
  lcd_byte(0x28,LCD_CMD) # 101000 Data length, number of lines, font size
  lcd_byte(0x01,LCD_CMD) # 000001 Clear display
  time.sleep(E_DELAY)

def lcd_byte(bits, mode):
  # Send byte to data pins
  # bits = data
  # mode = True  for character
  #        False for command

  GPIO.output(LCD_RS, mode) # RS

  # High bits
  GPIO.output(LCD_D4, False)
  GPIO.output(LCD_D5, False)
  GPIO.output(LCD_D6, False)
  GPIO.output(LCD_D7, False)
  if bits&0x10==0x10:
    GPIO.output(LCD_D4, True)
  if bits&0x20==0x20:
    GPIO.output(LCD_D5, True)
  if bits&0x40==0x40:
    GPIO.output(LCD_D6, True)
  if bits&0x80==0x80:
    GPIO.output(LCD_D7, True)

  # Toggle 'Enable' pin
  lcd_toggle_enable()

  # Low bits
  GPIO.output(LCD_D4, False)
  GPIO.output(LCD_D5, False)
  GPIO.output(LCD_D6, False)
  GPIO.output(LCD_D7, False)
  if bits&0x01==0x01:
    GPIO.output(LCD_D4, True)
  if bits&0x02==0x02:
    GPIO.output(LCD_D5, True)
  if bits&0x04==0x04:
    GPIO.output(LCD_D6, True)
  if bits&0x08==0x08:
    GPIO.output(LCD_D7, True)

  # Toggle 'Enable' pin 
  lcd_toggle_enable()

def lcd_toggle_enable():
  # Toggle enable
  time.sleep(E_DELAY)
  GPIO.output(LCD_E, True)
  time.sleep(E_PULSE)
  GPIO.output(LCD_E, False)
  time.sleep(E_DELAY)

def lcd_string(message,line):
  # Send string to display

  message = message.ljust(LCD_WIDTH," ")

  lcd_byte(line, LCD_CMD)

  for i in range(LCD_WIDTH):
    lcd_byte(ord(message[i]),LCD_CHR)
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
	lcd_init()
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
				e=int(e/0.65)
				f=int(f/0.65)
				lcd_string('Ball.x:{}'.format(e),LCD_LINE_1)
				lcd_string('Ball.y:{}'.format(f),LCD_LINE_2)
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
	#cv2.imshow('balance_ball',frame)
	#cv2.imshow('bianhuan',dst)
	#cv2.imshow('thresh',black)
camera.release()
cv2.destroyAllWindows()
