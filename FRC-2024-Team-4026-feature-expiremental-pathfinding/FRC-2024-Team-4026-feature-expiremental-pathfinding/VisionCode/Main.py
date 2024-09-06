from RioComms import RioComms
import cv2
from pupil_apriltags import Detector
import keyboard
import ast
import re

# Initialize RioComms and prepare vision
rioComms = RioComms("10.40.26.2")
tableName = "apriltags"

# Camera parameters for estimating apriltag pose
# cameraParams = (0, 0, 0, 0) # we need to calibrate camera to find the parameters (fx, fy, cx, cy)

# Apriltag size in meters
tagSize = 0.17

# Apriltag detector
# THESE VALUES NEED TO BE TUNED!!! ONLY FAMILY IS CORRECT
at_detector = Detector(
    families="tag36h11",
    nthreads=1,
    quad_decimate=0.3,
    quad_sigma=0.35,
    refine_edges=1,
    decode_sharpening=0.25,
    debug=0
)

# Video capture is 640 pixels by 480 pixels
cap = cv2.VideoCapture(0)

# Desired image size (should be small)
imgSizeX = 200
imgSizeY = 120

# Parameters for object detection
hueMin = 0
saturationMin = 0
valueMin = 0
hueMax = 180
saturationMax = 255
valueMax = 255

while True:
    # End program if "z" button is pressed
    if keyboard.is_pressed("z"):
        exit()

    # Read current frame from the camera
    img = cap.read()

    # Resize image to smaller size
    img = cv2.resize(img, (imgSizeX, imgSizeY))

    # Create HSV version of img (used for game piece detection)
    imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Create a grayscale version of img (used for apriltag detection)
    imgGrayscale = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Create object mask
    objectMask = cv2.inRange(imgHSV, (hueMin, saturationMin, valueMin), (hueMax, saturationMax, valueMax))
    objectMask = cv2.medianBlur(objectMask, 25)
    objectMask = cv2.erode(objectMask, 25)

    # Find object contours
    objectCnts = cv2.findContours(objectMask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Report the closest object to RioComms
    objectCntsAreas = []

    for i in objectCnts[0]:
        M = cv2.moments(i)

        objectCntsAreas.append(M["m00"])

    if len(objectCntsAreas) > 0:
        maxValue = max(objectCntsAreas)

        maxObjectCntsIndex = objectCntsAreas.index(maxValue)

        M = cv2.moments(objectCnts[0][maxObjectCntsIndex])

        cX = int((M["m10"] / M["m00"]))
        cY = int((M["m01"] / M["m00"]))

        rioComms.send("objects", "Object X", cX - (imgSizeX / 2))
        rioComms.send("objects", "Object Y", cY - (imgSizeY / 2))
        rioComms.send("objects", "Object Visible", 1)

    else:
        rioComms.send("objects", "Object X", 0)
        rioComms.send("objects", "Object Y", 0)
        rioComms.send("objects", "Object Visible", 0)

