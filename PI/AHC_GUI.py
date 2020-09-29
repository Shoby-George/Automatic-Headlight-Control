#!/usr/bin/python

import wx
import wx.xrc
import cv2
from numpy import squeeze, asarray
import threading
import sys
import pigpio

class AHCFrame ( wx.Frame ):
    def __init__( self, parent ):
        
        self._BITMAP_NIGHT = wx.Bitmap( u"night.jpg", wx.BITMAP_TYPE_ANY )
        self._BITMAP_DAY = wx.Bitmap( u"day.jpg", wx.BITMAP_TYPE_ANY )
        self._BITMAP_DIM = wx.Bitmap( u"dim.jpg", wx.BITMAP_TYPE_ANY )
        self._BITMAP_BRIGHT = wx.Bitmap( u"bright.jpg", wx.BITMAP_TYPE_ANY )
        self._BITMAP_OFF = wx.Bitmap( u"off.jpg", wx.BITMAP_TYPE_ANY )
        self._BITMAP_NONE = wx.Bitmap( u"AHC.jpg", wx.BITMAP_TYPE_ANY)

        ###########################################################################################################################################################
        
        wx.Frame.__init__ ( self, parent, id = wx.ID_ANY, title = wx.EmptyString, pos = wx.DefaultPosition, size = wx.Size( 840,490 ), style = 0|wx.TAB_TRAVERSAL )
        
        self.SetSizeHintsSz( wx.DefaultSize, wx.DefaultSize )
        
        bsH1 = wx.BoxSizer( wx.HORIZONTAL )
        
        self._bitmapCapture = wx.StaticBitmap( self, wx.ID_ANY, self._BITMAP_NONE, wx.DefaultPosition, wx.Size( 640,480 ), 0 )
        bsH1.Add( self._bitmapCapture, 0, wx.ALL, 5 )
        
        bsV11 = wx.BoxSizer( wx.VERTICAL )

        self._bitmapLightState = wx.StaticBitmap( self, wx.ID_ANY, self._BITMAP_BRIGHT, wx.DefaultPosition, wx.DefaultSize, 0 )
        bsV11.Add( self._bitmapLightState, 0, wx.ALL|wx.ALIGN_CENTER_HORIZONTAL, 5 )
                
        self._cbOverrideAutoDim = wx.CheckBox( self, wx.ID_ANY, u"Override", wx.DefaultPosition, wx.DefaultSize, 0 )
        bsV11.Add( self._cbOverrideAutoDim, 0, wx.ALL|wx.ALIGN_CENTER_HORIZONTAL, 5 )
        
        _rboxDimChoices = [ u"DIM", u"BRIGHT" ]
        self._rboxDim = wx.RadioBox( self, wx.ID_ANY, wx.EmptyString, wx.DefaultPosition, wx.DefaultSize, _rboxDimChoices, 1, wx.RA_SPECIFY_COLS )
        self._rboxDim.SetSelection( 0 )
        self._rboxDim.Enable( False )
        bsV11.Add( self._rboxDim, 0, wx.ALL|wx.EXPAND, 5 )

        self._bitmapDayNightMode = wx.StaticBitmap( self, wx.ID_ANY, self._BITMAP_NIGHT, wx.DefaultPosition, wx.DefaultSize, 0 )
        bsV11.Add( self._bitmapDayNightMode, 0, wx.ALL|wx.ALIGN_CENTER_HORIZONTAL, 5 )

        self._cbOverrideAutoMode = wx.CheckBox( self, wx.ID_ANY, u"Override", wx.DefaultPosition, wx.DefaultSize, 0 )
        bsV11.Add( self._cbOverrideAutoMode, 0, wx.ALL|wx.ALIGN_CENTER_HORIZONTAL, 5 )

        self._buttonClose = wx.Button( self, wx.ID_ANY, u"Close", wx.DefaultPosition, wx.DefaultSize, 0 )
        bsV11.Add( self._buttonClose, 0, wx.ALL|wx.EXPAND, 5 )   
        
        bsH1.Add( bsV11, 1, wx.EXPAND, 5 )
        
        self.SetSizer( bsH1 )
        self.Layout()
        
        self.Centre( wx.BOTH )
        
        # Connect Events
        self._cbOverrideAutoDim.Bind( wx.EVT_CHECKBOX, self.OverrideAutoDim )
        self._rboxDim.Bind( wx.EVT_RADIOBOX, self.ManualDimAndBright )
        self._cbOverrideAutoMode.Bind( wx.EVT_CHECKBOX, self.OverrideAutoMode )
        self._buttonClose.Bind( wx.EVT_BUTTON, self.onClose )
    
        ###########################################################################################################################################################

        thresholdStep = 8.0
        minThreshold = 191.0
        maxThreshold = 255.0
        minRepeatability = 2
        minDistBetweenBlobsProportional = 0.02
        minBlobAreaProportional = 0.0001
        maxBlobAreaProportional = 0.2
        minBlobCircularity = 0.2
        imageSize = (640, 480)
        self._roi_minX = int(0.4*imageSize[0])
        self._roi_maxX = int(1.0*imageSize[0])
        self._roi_minY = int(0.5*imageSize[1])
        self._roi_maxY = int(0.9*imageSize[1])

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

        self._detector = cv2.SimpleBlobDetector_create(detectorParams)

        self._running = True
        self._pause = False         # For pausing Capture Loop
        self._lightStates = [1]*5   # 0: Not Detected & 1: Detected
        self._nightMode = True
        self._modeOverride = False
        self._currentBitmapLightState = 1   # 0 : OFF, 1 : DIM, 2 : BRIGHT
        self._currentBitmapDayNightMode = 0 # 0 : Night, 1 : Day
            
        self._pi = pigpio.pi()
		
        self._capture = cv2.VideoCapture(0)
        self._bitmapCapture.SetBitmap(self._BITMAP_NONE)
        self._captureThread = threading.Thread(target=self._runCaptureLoop)
        self._captureThread.start()
    
    def _runCaptureLoop(self):
        while self._running:
            if self._pause == False:
                success, image = self._capture.read()
                self._detectDayNight(image)
                if self._nightMode or self._modeOverride:
                    self._detect(image)
                wx.CallAfter(self._showImage, image)

    def _showImage(self, image):
        if self._nightMode or self._modeOverride:
            if self._currentBitmapDayNightMode != 0:
                self._bitmapDayNightMode.SetBitmap(self._BITMAP_NIGHT)
                self._currentBitmapDayNightMode = 0
            if sum(self._lightStates) > 0:
                if self._currentBitmapLightState != 1:
                    self._pi.set_PWM_dutycycle(18, 0)
                    self._bitmapLightState.SetBitmap(self._BITMAP_DIM)
                    self._currentBitmapLightState = 1
            else:
                if self._currentBitmapLightState != 2:
                    self._pi.set_PWM_dutycycle(18, 255)
                    self._bitmapLightState.SetBitmap(self._BITMAP_BRIGHT)
                    self._currentBitmapLightState = 2
        else:
            if self._currentBitmapDayNightMode != 1:
                self._bitmapDayNightMode.SetBitmap(self._BITMAP_DAY)
                self._currentBitmapDayNightMode = 1
            if self._currentBitmapLightState != 0:
                self._bitmapLightState.SetBitmap(self._BITMAP_OFF)
                self._currentBitmapLightState = 0
       
        if image is None:
            # Provide a black bitmap.
            self._bitmapCapture.SetBitmap(self._BITMAP_NONE)
        else:
            # Convert the image to bitmap format.
            #image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            h, w = image.shape[:2]
            bitmap = wx.BitmapFromBuffer(w, h, image)
            # Show the bitmap.
            self._bitmapCapture.SetBitmap(bitmap)

    def _detect(self, image):
        grayImage = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        roi = grayImage[self._roi_minY:self._roi_maxY, self._roi_minX:self._roi_maxX]
        blobs = self._detector.detect(roi)
        self._lightStates.pop(0)
        if len(blobs) > 0:  # Detected
            self._lightStates.append(1)
        else:               # Not Detected
            self._lightStates.append(0)
        for blob in blobs:
            centerAsInts = tuple(int(n) for n in blob.pt)
            centerAsInts = (centerAsInts[0]+self._roi_minX,centerAsInts[1]+self._roi_minY)
            radiusAsInt = int(blob.size)
            cv2.circle(image, centerAsInts, radiusAsInt, (0, 255, 255), 2, cv2.LINE_AA)

    def _detectDayNight(self, image):
        roi = image[0:self._roi_minY, 0:self._roi_maxX]
        hist = self.compute_histogram(roi)
        means = self.weighted_means(hist)
        if means["red"] >= 75 and means["green"] >= 75 and means["blue"] >= 75:
            self._nightMode = False
        else:
            self._nightMode = True

    def weighted_means(self, data):
        #Calcluates the weighted means for the histogram data.
        wred = 0
        wgreen = 0
        wblue = 0

        for i in xrange(256):
            wred += (data["red"][i] * i)
            wgreen += (data["green"][i] * i)
            wblue += (data["blue"][i] * i)

        wred /= sum(data["red"])
        wgreen /= sum(data["green"])
        wblue /= sum(data["blue"])

        return {"red": wred, "blue": wblue, "green": wgreen}

    def compute_histogram(self, image):
        #Computes the histogram data for the specified image.
        if image.ndim == 1:
            blue = cv2.calcHist([image], [0], None, [256], [0, 256])
            green = cv2.calcHist([image], [0], None, [256], [0, 256])
            red = cv2.calcHist([image], [0], None, [256], [0, 256])
        elif image.ndim == 3:
            blue = cv2.calcHist([image], [0], None, [256], [0, 256])
            green = cv2.calcHist([image], [1], None, [256], [0, 256])
            red = cv2.calcHist([image], [2], None, [256], [0, 256])
        else:
            #Unknown color model for image
            sys.exit(1)

        data = {
            "red": squeeze(asarray(blue.astype(int))),
            "green": squeeze(asarray(green.astype(int))),
            "blue": squeeze(asarray(red.astype(int)))
        }
        return data

    def __del__( self ):
        pass

    def OverrideAutoDim( self, event ):
        cb = event.GetEventObject() 
        if cb.GetValue():
            self._pause = True
            self._rboxDim.Enable( True )
        else:
            self._pause = False
            self._rboxDim.Enable( False )
    def OverrideAutoMode( self, event ):
        cb = event.GetEventObject() 
        if cb.GetValue():
            self._modeOverride = True
        else:
            self._modeOverride = False
    def ManualDimAndBright( self, event ):
        if self._rboxDim.GetSelection() == 0:
	    self._pi.set_PWM_dutycycle(18, 0)
            self._bitmapLightState.SetBitmap(self._BITMAP_DIM)
        else:
	    self._pi.set_PWM_dutycycle(18, 255)
            self._bitmapLightState.SetBitmap(self._BITMAP_BRIGHT)

    def onClose( self, event ):
        self._running = False
        self._captureThread.join()
        self.Destroy()

app = wx.App(False)
frame = AHCFrame(None)
frame.Show(True)
app.MainLoop()
