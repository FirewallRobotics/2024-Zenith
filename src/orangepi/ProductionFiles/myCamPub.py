
#!/usr/bin/env python3
#
# This is a demo program showing CameraServer usage with OpenCV to do image
# processing. The image is acquired from the USB camera, then a rectangle
# is put on the image and sent to the dashboard. OpenCV has many methods
# for different types of processing.
#
# Warning: If you're using this with a python-based robot, do not run this
# in the same program as your robot code!
#

# example found at https://github.com/robotpy/robotpy-cscore/blob/main/examples/intermediate_cameraserver.py
# Note to Milo: I did have to update "pip install --update robotpy-cscore"
# to get past a segementation fault that was occring with the 2024.1.1 code on my
# Macbook.
# This opens camera feed ot my built in camera and posts it to the CameraPublisher table.
# Draws on the image.
# Network table innfo is updated.
# Camera feed can be accessed via webbrowser:
#   -testing locally on laptop, set RobotSimulation to localhost and keep NT Server as local host
#      *connect to http://localhost:1182 to see what the camerafeed is displaying
#   -test on robot, you will need to connect to NT tables on robot on 10.56.7.2
#      *connect to http://10.56.7.2:1182 change port to see multiple cameras ...starts at 1181
#  We aren't trying to switch cameras, we are just procesing feed for one camera in the CameraPublisher table

import cv2
import numpy as np
import ntcore
import apriltag
import imutils
from scipy.spatial.transform import Rotation

from cscore import CameraServer as CS
# import added in an attempt to get feeds on thr Shuffleboard display... not needed for browser viewing

# pulled from testAprilTagrecognition.py
def distance_to_camera(knownWidth, focalLength, perWidth):
	# compute and return the distance from the maker to the camera
	return (float(knownWidth) * float(focalLength)) / float(perWidth)

# pulled from testAprilTagrecognition.py
def find_marker(image):
	# convert the image to grayscale, blur it, and detect edges
	# gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    #image = cv2.GaussianBlur(image, (5, 5), 0)
    edged = cv2.Canny(image, 35, 125)
	# find the contours in the edged image and keep the largest one;
	# we'll assume that this is our piece of paper in the image
    cnts = cv2.findContours(edged.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    c = max(cnts, key = cv2.contourArea)
	# compute the bounding box of the of the paper region and return it
    return cv2.minAreaRect(c)

#reads the calibration data; pulled from FinalPiProgram.py
def read_from_txt_file(filename):
    try:
        with open(filename, 'r') as txt_file:
            lines = txt_file.readlines()
            if len(lines) >= 4:
                var1 = lines[0].strip()
                var2 = lines[1].strip()
                var3 = lines[2].strip()
                var4 = lines[3].strip()
                return var1, var2, var3, var4
            else:
                #print(f"File '{filename}' does not contain enough lines.")
                return None
    except FileNotFoundError:
        #print(f"File '{filename}' not found.")
        return None



def main():
    # nice part of sample code enables internal logging of the CameraServer code
    CS.enableLogging()

    camera = CS.startAutomaticCapture()

    camera.setResolution(640, 480)

    # Get a CvSink. This will capture images from the camera
    cvSink = CS.getVideo()

    # (optional) Setup a CvSource. This will send images back to the Dashboard
    outputStream = CS.putVideo("ApriltTagTest", 640, 480)

    # Allocating new images is very expensive, always try to preallocate
    img = np.zeros(shape=(480, 640, 3), dtype=np.uint8)

    #configs the detector, pulled from FinalPiProgram.py
    options = apriltag.DetectorOptions(families="tag36h11")
    detector = apriltag.Detector(options)
    FRCtagSize = float(0.17) #17cm
    fx, fy, cx, cy = read_from_txt_file("cal.txt")

    cameraParams = float(fx), float(fy), float(cx), float(cy)


    # iteration updated in status to NT tables to make sure program is still alive
    iteration= 0
    myStrPub =table.getStringTopic(str("Cam1_Status")).publish()

    myStrPub.set("status : iteration is {}".format(iteration))

    while True:
        # Tell the CvSink to grab a frame from the camera and put it
        # in the source image.  If there is an error notify the output.
        time, img = cvSink.grabFrame(img)
        img = np.rot90(np.rot90(img, 1), 1)
        if time == 0:
            # Send the output the error.
            outputStream.notifyError(cvSink.getError())
            # skip the rest of the current iteration
            continue

        # Origial example Put a rectangle on the image
        # cv2.rectangle(img, (100, 100), (400, 400), (255, 255, 255), 5)
  
        # Process image for Apriltags
        img= cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        detections = detector.detect(img)
        if not detections:
            #  Notify that no tags are detected on image
            myStrPub.set("status : iteration is {}. Nothing detected".format(iteration))
            cv2.putText(img, "Nothing Detected", (100,100), cv2.FONT_HERSHEY_SIMPLEX, 2.0, (0, 0, 255), 2)
            outputStream.putFrame(img)
            continue
        else :
            for detect in detections:
                 marker = find_marker(img)
                 distance = distance_to_camera(FRCtagSize,fx,marker[1][0])
                 myStrPub.set("status : Distance to marker is {}.".format(distance))
                 cv2.putText(img, "Distance: {}".format(distance), (200,200), cv2.FONT_HERSHEY_SIMPLEX, 2.0, (0, 0, 255), 2)

                # Processing pulled from FianlPiProgram
                 pos, e1,f1=detector.detection_pose( detect, cameraParams, FRCtagSize, z_sign=1)
                 pos = pos[:3, :3]
                 rotation = Rotation.from_matrix(pos)
                 euler_angles = rotation.as_euler('xyz')
                 pos = np.degrees(euler_angles)
                 pos_string = ("POS euler_angle: %s" % (str(pos)))
                 cv2.putText(img, pos_string, (600,600),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                 # Label corner of tags with id
                 cornerIndex=0
                 for corner in detect.corners:
                    if cornerIndex== 0:
                        org=(int(corner[0]),int(corner[1]))
                        tagId=("AprilTagId %s" % (str(detect.tag_id)))
                        cv2.putText(img, tagId, org, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    cornerIndex+=1
                 myStrPub =table.getStringTopic(str(detect.tag_id)).publish()
                 tag_dat= '{ "POS_euler": ' + '"{}",'.format(pos) + '"distance": "{}" }'.format(distance)
                 myStrPub.set(tag_dat)

        # Give the output stream a new image to display
        outputStream.putFrame(img)
        
        myStrPub = table.getStringTopic(str("Cam1_Status")).publish()

        myStrPub.set("status : iteration is {}".format(iteration))
        iteration+=1
       

if __name__ == "__main__":
    # To see messages from networktables, you must setup logging
    import logging

    logging.basicConfig(level=logging.DEBUG)

    # You should uncomment these to connect to the RoboRIO
    # import ntcore
    nt = ntcore.NetworkTableInstance.getDefault()
    # nt.setServerTeam(XXXX)
    nt.setServer("10.56.7.2")
    # nt.startClient4(__file__)
    nt.startClient4("myCamPub vision client")
    table = nt.getTable("myCamPub")
    
    myStrPub =table.getStringTopic(str("Cam1_Status")).publish()
    myStrPub.set('{"Error": "No tags detected "}' )

    main()

    # can be connected to via webbrowser to debug using http://roborio-TEAM-frc.local:1181/?action=stream increase port number for multiple cameras