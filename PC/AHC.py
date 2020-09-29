import cv2
import numpy
import math

def hueFromBGR(color):
    b, g, r = color
    hue = math.degrees(math.atan2(
        1.7320508075688772 * (g - b), 2 * r - g - b))
    if hue < 0.0:
        hue += 360.0
    return hue

def saturationFromBGR(color):
    return max(color) - min(color)

COLOR_YellowWhite = ((223, 247, 255), 'yellowish white')
##COLOR_AmberYellow = ((  0, 191, 255), 'amber or yellow')
##COLOR_Red         = ((  0,   0, 255), 'red')
##COLOR_Green       = ((128, 255, 128), 'green')
##COLOR_BlueWhite   = ((255, 231, 223), 'bluish white')
##COLOR_BluePurple  = ((255,  64,   0), 'blue or purple')
##COLOR_Pink        = ((240, 128, 255), 'pink')

thresholdStep = 8.0
minThreshold = 191.0
maxThreshold = 255.0
minRepeatability = 2
minDistBetweenBlobsProportional = 0.02
minBlobAreaProportional = 0.0001
maxBlobAreaProportional = 0.2
minBlobCircularity = 0.5
imageSize = (640, 360)
roi_minX = int(0.4*imageSize[0])
roi_maxX = int(1.0*imageSize[0])
roi_minY = int(0.5*imageSize[1])
roi_maxY = int(0.9*imageSize[1])

minDistBetweenBlobs =  min(imageSize) * minDistBetweenBlobsProportional
area = imageSize[0] * imageSize[1]
minBlobArea = area * minBlobAreaProportional
maxBlobArea = area * maxBlobAreaProportional

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

cap = cv2.VideoCapture('vid.mp4')

while True:
    ret, image = cap.read()
    if ret == False:
        break
    
    grayImage = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    roi = grayImage[roi_minY:roi_maxY, roi_minX:roi_maxX]
    blobs = detector.detect(roi)
    blobsForColors = {}
    for blob in blobs:
        centerXAsInt, centerYAsInt = (int(n) for n in blob.pt)
        radiusAsInt = int(blob.size)

        minX = max(0, centerXAsInt - radiusAsInt)
        maxX = min(imageSize[0], centerXAsInt + radiusAsInt)
        minY = max(0, centerYAsInt - radiusAsInt)
        maxY = min(imageSize[1], centerYAsInt + radiusAsInt)

        region = image[minY:maxY, minX:maxX]

        # Get the region's dimensions, which may
        # differ from the blob's diameter if the blob
        # extends past the edge of the image.
        h, w = region.shape[:2]

        meanColor = region.reshape(w * h, 3).mean(0)
        meanHue = hueFromBGR(meanColor)
        meanSaturation = saturationFromBGR(meanColor)

##        if meanHue < 22.5 or meanHue > 337.5:
##            color = COLOR_Red
##        elif meanHue < 67.5:
##            if meanSaturation < 25.0:
##                color = COLOR_YellowWhite
##            else:
##                color = COLOR_AmberYellow
##        elif meanHue < 172.5:
##            color = COLOR_Green
##        elif meanHue < 277.5:
##            if meanSaturation < 25.0:
##                color = COLOR_BlueWhite
##            else:
##                color = COLOR_BluePurple
##        else:
##            color = None

        if meanHue >= 250 and meanHue <= 275 and meanSaturation >= 1.5 and meanSaturation <= 2.5:
            color = COLOR_YellowWhite
        else:
            color = None
        if color in blobsForColors:
            blobsForColors[color] += [blob]
        else:
            blobsForColors[color] = [blob]
            

    for color in blobsForColors:
        for blob in blobsForColors[color]:
            if color is not None:
                colorBGR, colorName = color

                centerAsInts = tuple(int(n) for n in blob.pt)
                centerAsInts = (centerAsInts[0]+roi_minX,centerAsInts[1]+roi_minY)
                radiusAsInt = int(blob.size)

                cv2.circle(image, centerAsInts, 20, colorBGR, 2,cv2.LINE_AA)
                            
    #cv2.rectangle(image, (roi_minX,roi_minY), (roi_maxX, roi_maxY), (0, 0, 255), 2, cv2.LINE_AA)       
    cv2.imshow('AUTOMATIC HEADLIGHT CONTROL - AHC',image)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break;

cap.release()
cv2.destroyAllWindows()
