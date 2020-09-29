import sys
import smbus
import time
import math
import cv2
import pigpio

################################### Image Processing Initialization ####################################################
imageSize = (640, 480)
imageArea = imageSize[0] * imageSize[1]

thresholdStep = 8.0
minThreshold = 191.0
maxThreshold = 255.0
minRepeatability = 2
minDistBetweenBlobsProportional = 0.02
minBlobAreaProportional = 0.0001
maxBlobAreaProportional = 0.2
minBlobCircularity = 0.2
minDistBetweenBlobs = min(imageSize) * minDistBetweenBlobsProportional
minBlobArea = imageArea * minBlobAreaProportional
maxBlobArea = imageArea * maxBlobAreaProportional

detectorParams = cv2.SimpleBlobDetector_Params()

detectorParams.minDistBetweenBlobs = minDistBetweenBlobs
detectorParams.thresholdStep = thresholdStep
detectorParams.minThreshold = minThreshold
detectorParams.maxThreshold = maxThreshold
detectorParams.minRepeatability = minRepeatability
detectorParams.filterByArea = True
detectorParams.minArea = minBlobArea
detectorParams.maxArea = maxBlobArea
detectorParams.filterByColor = True
detectorParams.blobColor = 255
detectorParams.filterByCircularity = True
detectorParams.minCircularity = minBlobCircularity
detectorParams.filterByInertia = False
detectorParams.filterByConvexity = False

detector = cv2.SimpleBlobDetector_create(detectorParams)

roi_minX = int(0.4*imageSize[0])
roi_maxX = int(1.0*imageSize[0])
roi_minY = int(0.5*imageSize[1])
roi_maxY = int(0.9*imageSize[1])

lightStates = [1]*5                # 0: Not Detected & 1: Detected
dimState = True
isOverrideAutoDim = False          # For pausing Capture Loop
nightMode = True
isOverrideDayNightMode = False

capture = cv2.VideoCapture(0)

pi = pigpio.pi()
######################## Override Push Button Handling Initialisation###################################################
pi.set_mode(27, pigpio.INPUT)
pi.set_mode(23, pigpio.INPUT)
pi.set_mode(24, pigpio.INPUT)
pi.set_pull_up_down(27, pigpio.PUD_UP)
pi.set_pull_up_down(23, pigpio.PUD_UP)
pi.set_pull_up_down(24, pigpio.PUD_UP)


overrideAutoDimFlag = False
def overrideAutoDim(gpio, level, ticks):
    global overrideAutoDimFlag
    if not overrideAutoDimFlag:
        overrideAutoDimFlag = True


manualDimAndBrightFlag = False
def manualDimAndBright(gpio, level, ticks):
    global manualDimAndBrightFlag
    if not manualDimAndBrightFlag:
        manualDimAndBrightFlag = True

overrideDayNightModeFlag = False
def overrideDayNightMode(gpio, level, ticks):
    global overrideDayNightModeFlag
    if not overrideDayNightModeFlag:
        overrideDayNightModeFlag = True

callf1 = pi.callback(27, pigpio.RISING_EDGE, overrideAutoDim)
callf2 = pi.callback(23, pigpio.RISING_EDGE, manualDimAndBright)
callf3 = pi.callback(24, pigpio.RISING_EDGE, overrideDayNightMode)

########################################## Pan Tilt Initialization #####################################################
def read_word(adr):
    high = bus.read_byte_data(address, adr)
    low = bus.read_byte_data(address, adr+1)
    val = (high << 8) + low
    return val

def read_word_2c(adr):
    val = read_word(adr)
    if val >= 0x8000:
        return -((65535 - val) + 1)
    else:
        return val

def dist(a, b):
    return math.sqrt((a*a)+(b*b))

def get_y_rotation(x, y, z):
    radians = math.atan2(x, dist(y, z))
    return -math.degrees(radians)

def get_x_rotation(x, y, z):
    radians = math.atan2(y, dist(x, z))
    return math.degrees(radians)

bus = smbus.SMBus(1)
# Device Address
address = 0x68
# Power Management Registers
power_mgmt_1 = 0x6B
power_mgmt_2 = 0x6C
# Wake up 6050 as it starts in sleep mode
bus.write_byte_data(address, power_mgmt_1, 0)

angleX = 0.0
angleY = 0.0
prev_time = time.time()

while True:
    ########################################## Pan Tilt ################################################################
    try:
        # Read Gyroscope Data
        gyro_xout = read_word_2c(0x43)
        gyro_yout = read_word_2c(0x45)
        gyro_zout = read_word_2c(0x47)
        # Read Accelerometer Data
        accel_xout = read_word_2c(0x3B)
        accel_yout = read_word_2c(0x3D)
        accel_zout = read_word_2c(0x3F)
        # Accelerometer Full Scale Range = (+/-)2g, Sensitivity Scale Factor = 16384 LSB/g
        accel_xout_scaled = accel_xout/16384.0
        accel_yout_scaled = accel_yout/16384.0
        accel_zout_scaled = accel_zout/16384.0

        thetaX = get_x_rotation(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled)
        thetaY = get_y_rotation(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled)
        # Gyroscope Full Scale Range = (+/-)250 degrees/s, Sensitivity Scale Factor = 131 LSB/(degrees/s)
        wX = gyro_xout/131.0
        wY = gyro_yout/131.0

        cur_time = time.time()
        dt = cur_time - prev_time
        prev_time = cur_time
        # Complementary Filter
        angleX = 0.95*(angleX+wX*dt)+0.05*thetaX
        angleY = 0.95*(angleY+wY*dt)+0.05*thetaY
        # Angle to Servo PWM Width Conversion
        pwmX = int(10*angleX+1500)
        pwmY = int(10*angleY+1500)

        if pwmX >= 1000 and pwmX <= 2000:
            pi.set_servo_pulsewidth(15, pwmX)
        if pwmY >= 1000 and pwmY <= 2000:
            pi.set_servo_pulsewidth(17, pwmY)
    except IOError:
        print "IO Error while reading MPU6050 data"
    ########################################## Image Processing ########################################################
    if not isOverrideAutoDim:
        success, image = capture.read()
        ###################################### Day/Night Mode ##########################################################
        if not isOverrideDayNightMode:
            # Histogram
            roi_top = image[0:roi_minY, 0:roi_maxX]
            blue = cv2.calcHist([roi_top], [0], None, [256], [0, 256])
            green = cv2.calcHist([roi_top], [1], None, [256], [0, 256])
            red = cv2.calcHist([roi_top], [2], None, [256], [0, 256])
            # Weighted means of histogram
            wred = 0
            wgreen = 0
            wblue = 0
            for i in xrange(256):
                wred += (red[i] * i)
                wgreen += (green[i] * i)
                wblue += (blue[i] * i)
            wred /= sum(red)
            wgreen /= sum(green)
            wblue /= sum(blue)
            if wred >= 75 and wgreen >= 75 and wblue >= 75:
                nightMode = False
            else:
                nightMode = True
        if nightMode or isOverrideDayNightMode:
            ################################### Headlight Detection ####################################################
            grayImage = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            roi = grayImage[roi_minY:roi_maxY, roi_minX:roi_maxX]
            blobs = detector.detect(roi)
            lightStates.pop(0)
            if len(blobs) > 0:               # Detected
                lightStates.append(1)
            else:                            # Not Detected
                lightStates.append(0)
        if nightMode:
            if sum(lightStates) > 0:
                dimState = True
                pi.set_PWM_dutycycle(4, 64)
                #print "Dim"
            else:
                dimState = False
                pi.set_PWM_dutycycle(4, 255)
                #print "Bright"
        else:
            pi.set_PWM_dutycycle(4, 0)
            #print "Off"
    ####################################### Override Push Buttons Handling #############################################
    if overrideAutoDimFlag:
        if isOverrideAutoDim:
            isOverrideAutoDim = False
            #print "Auto Dim Activated"
            pi.write(22, 0)
        else:
            isOverrideAutoDim = True
            #print "Overriding Auto Dim to Manual Dim"
            pi.write(22, 1)
        overrideAutoDimFlag = False
    if manualDimAndBrightFlag:
        if dimState:
            pi.set_PWM_dutycycle(4, 255)
            dimState = False
            #print "Manual Bright"
            pi.write(10, 1)
        else:
            pi.set_PWM_dutycycle(4, 64)
            dimState = True
            #print "Manual Dim"
            pi.write(10, 0)
        manualDimAndBrightFlag = False
    if overrideDayNightModeFlag:
        if isOverrideDayNightMode:
            isOverrideDayNightMode = False
            #print "Day/Night Mode Activated"
            pi.write(25, 0)
        else:
            isOverrideDayNightMode = True
            #print "Overriding Day/Night Mode to Night Mode"
            pi.write(25, 1)
        overrideDayNightModeFlag = False

capture.release()


