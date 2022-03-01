import smbus
import time
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
import argparse
import cv2
from picamera.array import PiRGBArray
from picamera import PiCamera
import time

quad = 5
desired = 0
oldQuad = 0

# for RPI version 1, use “bus = smbus.SMBus(0)”
bus = smbus.SMBus(1)

# Modify this if you have a different sized Character LCD
lcd_columns = 16
lcd_rows = 2

# Initialise I2C bus.
i2c = board.I2C()  # uses board.SCL and board.SDA

# Initialise the LCD class
lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)

lcd.clear()

# Set LCD color to purple
lcd.color = [50, 0, 50]

# This is the address we setup in the Arduino Program
address = 0x04

def comVis():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Cannot open camera")
        exit()
    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()
        # if frame is read correctly ret is True
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            break
        # replace following functions with my functions

        #load the image
        imagehsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        #define the list of boundaries
        lower_blue = np.array([100,150,0])
        upper_blue = np.array([140,255,255])
   
        #find colors within the specified boundaries and apply the mask
        mask = cv2.inRange(imagehsv, lower_blue, upper_blue)
        output = cv2.bitwise_and(imagehsv, imagehsv, mask=mask)

        #cleaning up image
        kernel = np.ones((15,15),np.uint8)
        opening = cv2.morphologyEx(output, cv2.MORPH_CLOSE,kernel)

        #tape location
        imageG = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY)
        imageLL = cv2.threshold(imageG,50,255,cv2.THRESH_BINARY)
        tup=cv2.findNonZero(imageLL[1])
        tup2= cv2.mean(tup)
        print("Blue tape located at ", round(tup2[0]), round(tup2[1]))
        print(27*(((opening.shape[1])/2)-(tup2[0]))/(opening.shape[1]/2))

        if(tup2[0]==0 or tup2[1]==0):
            marker=False
            print("No marker found")
        else:
            marker=True
       
            Xmiddleofimage = opening.shape[1]/2
            Ymiddleofimage = opening.shape[0]/2

             #These if statements were used in order to assign quadrants depending on the values of the centers of the aruco marker.
            if tup2[1] > Xmiddleofimage and tup2[0] > Ymiddleofimage:
                quadrant = 4
                desired = "4.71"
                print("This is the quadrant: ", quadrant)
            if tup2[1] < Xmiddleofimage and tup2[0] > Ymiddleofimage:
                quadrant = 3
                desired = "3.14"
                print("This is the quadrant: ", quadrant)
            if tup2[1] < Xmiddleofimage and tup2[0] < Ymiddleofimage:
                quadrant = 2
                desired = "1.57"
                print("This is the quadrant: ", quadrant)
            if tup2[1] > Xmiddleofimage and tup2[0] <Ymiddleofimage:
                quadrant = 1
                desired = "0"
                print("This is the quadrant: ", quadrant)
   
        # Display the resulting frame
        cv2.imshow('imageLL', opening)
        #cv2.imshow('frame', opening)
        if cv2.waitKey(1) == ord('q'):
            break
    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()

def writePoistion(value):
    bus.write_byte(address, value)
    #bus.write_byte_data(address, 0, value)
    #bus.write_i2c_block_data(address, 0, value)
    return -1

def readPositon():
    number = bus.read_byte(address)
    #number = bus.read_byte_data(address, 0)
    #number = bus.read_i2c_block_data(address,0, datasize)
    return number

def sysInt():
    #defines global variable to be used later
    global oldQuad
    #writes quad to arduino
    writePosition(quad)
    print("RPI: Arduino, Move to position " ,quad)
    # Set LCD color to purple and words to go left to right
    lcd.color = [50, 0, 50]
    lcd.text_direction = lcd.LEFT_TO_RIGHT
    #Prints LCD Message waits a sec then clears the LCD
    lcd.message = "Move to pos. " + str(quad)
    time.sleep(2)
    lcd.clear()
    #Read in Position from the Arduino
    number = readPosition()
    #if statement to figure out what to print
    if quad == oldQuad:
        print("Already at position", quad)
        # Set LCD color to blue and words to go left to right
        lcd.color = [0, 100, 0]
        lcd.text_direction = lcd.LEFT_TO_RIGHT
        #Prints LCD Message waits a sec then clears the LCD
        lcd.message = "Already at pos.\n " + str(quad)
        time.sleep(2)
        lcd.clear()
    else:
        print("Arduino: Hey RPI, I moved to ", number)
        # Set LCD color to red and words to go left to right
        lcd.color = [100, 0, 0]
        lcd.text_direction = lcd.LEFT_TO_RIGHT
        #Prints LCD Message waits a sec then clears the LCD
        lcd.message = "Moved to pos.  " + str(quad)
        time.sleep(2)
        lcd.clear()
        lcd.color = [0,0,0]
    oldQuad = quad
   
while True:
    #runs computer vision part
    comVis()
    #to run sysint or not
    if marker == True:
        sysInt()
    else:
        print ("RPI: Marker not found.")
        lcd.color = [0,0,100]
        lcd.text_direction = lcd.LEFT_TO_RIGHT
        lcd.message = "Marker not found"
        time.sleep(2)
        lcd.clear()
        lcd.color = [0,0,0]
        
