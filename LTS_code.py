# Automatic Laser Turret system
# developed by Jeff G June 2020

from time import sleep
from SimpleCV import Camera, Display, Image
import math
import RPi.GPIO as GPIO
import pigpio

# what rasp pi pins connect to what electronic components
laserpin = 19
bottomservo = 21
topservo = 5

# sets up the rasp pi pin that control the laser
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(laserpin,GPIO.OUT)

# setup software object that enables the 2 rasp pi pins connected to the
# servos to generate pulses (pulses are how servos are controlled)
pi = pigpio.pi()

# create camera object and set resolution
cam = Camera(0,{"width":1920 , "height":1080})

#function to convert raw servo signal to 0->180 range
def Angler(theta):
    temp = ((float(100)/9)*theta) + 500
    if (temp > 2500):
        return 2500
    if (temp < 0):
        return 500
    return temp

# servo Angle/Signal calibration data
# (ensures that when I tell a servo to go to some angle, it actually goes to that angle)
BottomAngles = [-26.054 , -19.63 , -12.6 , -5.29 , 2.3 , 9.98 , 17.27 , 24.06]
BottomSignals = [139, 132.5, 125, 117.5, 108, 100.5, 91.5, 85]
TopAngles = [6.21 , 8.12 , 12.39 , 23.24 , 36.3]
TopSignals = [62.75 , 61 , 57.5 , 45 , 33]

# for use with the data above to ensure that the angle I want actually happens
def interpolate(X_pixelsX_pixels , Z_pixelsZ_pixels , X_pixels):

    status = 0 #will be zero until we find an interp

    m=0     #default vals
    b=0
    interp = 0

    for i in range(0,len(X_pixelsX_pixels)):

        #if we are to interp using ONLZ_pixels the bottom point (e.g. X_pixels is -30)
        if (X_pixels < X_pixelsX_pixels[i] and i==0):
            print "T1"
            m = (float(Z_pixelsZ_pixels[i+1]) - Z_pixelsZ_pixels[i]) / (X_pixelsX_pixels[i+1] - X_pixelsX_pixels[i])
            b = float(Z_pixelsZ_pixels[i+1]) - m*X_pixelsX_pixels[i+1]
            interp = X_pixels*m + b
            status=1 #the requested interp was in our range
            break

        #normal interpolation
        if (X_pixels < X_pixelsX_pixels[i] and i!=0):
            print "trigger"

            m = (float(Z_pixelsZ_pixels[i]) - Z_pixelsZ_pixels[i-1]) / (X_pixelsX_pixels[i] - X_pixelsX_pixels[i-1])
            b = float(Z_pixelsZ_pixels[i]) - m*X_pixelsX_pixels[i]
            interp = X_pixels*m + b
            status=1 #the requested interp was in our range
            break

        #the slim chance that our requested interp is one of the data points and no interpolation is needed
        if (X_pixels == X_pixelsX_pixels[i]):
            interp = Z_pixelsZ_pixels[i]
            status=1
            break

    #end of for loop



    #if our interp request is north of our data domain we will interp using the last 2 data pts
    if (status == 0):
        print "T3"
        m = (float(Z_pixelsZ_pixels[len(Z_pixelsZ_pixels)-1]) - Z_pixelsZ_pixels[len(Z_pixelsZ_pixels)-2]) / (X_pixelsX_pixels[len(X_pixelsX_pixels)-1] - X_pixelsX_pixels[len(X_pixelsX_pixels)-2])
        b = float(Z_pixelsZ_pixels[len(Z_pixelsZ_pixels)-1]) - m*X_pixelsX_pixels[len(X_pixelsX_pixels)-1]
        interp = X_pixels*m + b
        status=1 #we finallZ_pixels got the requested interp so we can status =1



    if (status==1):
        return interp

#END OF INTERP FUNCTION

# Here's where we begin the main operation of the system

# set both servos to their respective zero angle origins
pi.set_servo_pulsewidth(bottomservo, Angler(interpolate(BottomAngles ,BottomSignals, 45) ) )
pi.set_servo_pulsewidth(topservo, Angler(interpolate(TopAngles ,TopSignals, 45) ) )

# this while loop contains the complete operation of the system
# i.e. 1)Take Pic 2) Obtain target XYZ 3) Calc servo angles 4) Turn laser ON
# as long as I press 1 at the end, the system will run (the while loop) again
choice = 1
while (choice==1):

# take picture
    img = cam.getImage()

    img = img.flipVertical()
    img = img.flipHorizontal()
    img = img.invert()

# identify target in image
    blobs = img.findBlobs()
    for i in range (0,len(blobs)):
            actual_area = blobs[i].area()
            pred_area = blobs[i].length()*blobs[i].height()

            if (abs(blobs[i].height() - blobs[i].length()) < 20 and abs(actual_area - pred_area) < .1*pred_area ):
                targetblob = blobs[i]


# Attributes of target measured in pixels: X,Z,Length,Height
    X_pixels = targetblob.coordinates()[0]
    Z_pixels = targetblob.coordinates()[1]
    L = targetblob.length()
    H = targetblob.height()

# Linear ratio used for X and Z
    linear_ratio = 107.4 / H # mm/piX_pixels

# calc X and Z
    Z = ((img.height/2) - Z_pixels)*linear_ratio
    X = (X_pixels - (img.width/2))*linear_ratio





# warping correction
    C = targetblob.area() - .0006*X*X - 1.0945*X

# calculate Y
    Y = math.pow(C , -.491) * 151218

# Z adjustment (camera incline angle)
    Z = Z / math.sin(math.radians(90-1.23)) + Y*math.tan(math.radians(1.23))



# calculate target position relative to turret
    X_turret = X - 90.26
    Y_turret = Y + 21.26
    Z_turret = Z + 14.38

# calc the bottom servo angle and conv to degrees
    bottom_angle = math.degrees( math.atan( X_turret / Y_turret ) )

# temp variable
    h = math.sqrt( X_turret*X_turret + Y_turret*Y_turret )

# calc the top servo angle and conv to degrees
    top_angle = math.degrees( math.atan( z_turret / h ) )

# obtain corresponding signals
    bottom_signal = interpolate(BottomAngles ,BottomSignals, bottom_angle )
    top_signal = interpolate(TopAngles ,TopSignals, top_angle )

# rotate servos
    pi.set_servo_pulsewidth(bottomservo, Angler(bottom_signal) )
    pi.set_servo_pulsewidth(topservo, Angler(top_signal ) )

    sleep(1)

# fire laser
    GPIO.output(laserpin, GPIO.HIGH)
    sleep(1)
    GPIO.output(laserpin, GPIO.LOW)
    sleep(0.5)

# rotate back to resting position
    pi.set_servo_pulsewidth(bottomservo, Angler(interpolate(BottomAngles ,BottomSignals, 45) ) )
    pi.set_servo_pulsewidth(topservo, Angler(interpolate(TopAngles ,TopSignals, 45) ) )

# press 1 to run system again
    choice = input("1: Shoot Again")
