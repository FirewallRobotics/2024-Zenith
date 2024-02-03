from threading import Thread
import ntcore
import cv2
import numpy as np
import apriltag
import time
import sys

class myWebcamVideoStream:
  def __init__(self, src=0):
    
    #init network tables

    global testmode
    testmode = False

    if sys.argv != None:
        testmode = True

    global myStrPub, table

    TEAM = 5607
    if testmode == False:
        ntinst = ntcore.NetworkTableInstance.getDefault()
    table = ntinst.getTable("PiDetector")
    ntinst.startClient4("pi1 vision client")
    ntinst.setServer("10.56.7.2")
    
    # initialize the video camera stream and read the 
    # first frame from the stream
    self.stream = cv2.VideoCapture(src) 
    (self.grabbed, self.frame) = self.stream.read()

    # flag to stop the thread

    self.stopped = False

  def start(self):
    # start the thread to read frames
    Thread(target=self.update, args=()).start()
    return self

  def update(self):

    while True:
       # have we been told to stop?  If so, get out of here
       if self.stopped:
           return

       # otherwise, get another frame
       (self.grabbed, self.frame) = self.stream.read()

  def read(self):
      # return the most recent frame
      return self.frame

  def stop(self):
      # signal thread to end
      self.stopped = True
      return

#reads the calibration data
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
                print(f"File '{filename}' does not contain enough lines.")
                return None
    except FileNotFoundError:
        print(f"File '{filename}' not found.")
        return None


#I think this plots the locations on the screen?
def plotPoint(image, center, color):
    center = (int(center[0]), int(center[1]))
    image = cv2.line(image,
                     (center[0] - 5, center[1]),
                     (center[0] + 5, center[1]),
                     color,
                     3)
    image = cv2.line(image,
                     (center[0], center[1] - 5),
                     (center[0], center[1] + 5),
                     color,
                     3)
    return image

#With love from ChatGPT
def denoise_image(image, kernel_size=(5, 5)):
    """
    Apply Gaussian blur to denoise the image.

    Parameters:
    - image: Input image (NumPy array).
    - kernel_size: Size of the Gaussian kernel (default is (5, 5)).

    Returns:
    - Denoised image.
    """
    denoised_image = cv2.GaussianBlur(image, kernel_size, 0)
    return denoised_image

def average_position_of_pixels(mat, threshold=128):
    # Threshold the image to get binary image
    _, thresh = cv2.threshold(mat, threshold, 255, cv2.THRESH_BINARY)

    # Find contours in the binary image
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Initialize variables to store total x and y coordinates
    total_x = 0
    total_y = 0

    # Iterate through each contour
    for contour in contours:
        # Calculate the centroid of the contour
        M = cv2.moments(contour)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])

            # Add the centroid coordinates to the total
            total_x += cx
            total_y += cy

    # Calculate the average position
    if len(contours) > 0:
        avg_x = total_x / len(contours)
        avg_y = total_y / len(contours)
        return int(avg_x), int(avg_y)
    else:
        return 0, 0


# main program
#configs the detector
if testmode == False:
    vs = myWebcamVideoStream(0).start()
    vb = myWebcamVideoStream(1).start()
options = apriltag.DetectorOptions(families="tag36h11")
detector = apriltag.Detector(options)

FRCtagSize = float(0.17) #17cm
fx, fy, cx, cy = read_from_txt_file("cal.txt")
newcameramtxstr, mtxstr, diststr, useless = read_from_txt_file("cal2.txt")
mtx = np.array(eval(mtxstr))
dist = np.array(eval(diststr))
newcameramtx = np.array(eval(newcameramtxstr))

cameraParams = float(fx), float(fy), float(cx), float(cy)

# define color the list of boundaries
boundaries = [
	([80,45,170], [100,145,255])
]

#makes sure there is a camera to stream
if not vs:
   print("no image")

iteration = 0
saved = False
#Todo: Make not timed but not stupid
while True:
   if testmode == False:
    frame = vs.read()
    frame2 = vb.read()
   else:
      frame = cv2.imread('test.jpg')
      frame2 = cv2.imread('test.jpg')

   for (lower, upper) in boundaries:
    # create NumPy arrays from the boundaries
    lower = np.array(lower, dtype = "uint8")
    upper = np.array(upper, dtype = "uint8")
    # find the colors within the specified boundaries and apply
    # the mask
    mask = cv2.inRange(frame2, lower, upper)
    output = cv2.bitwise_and(frame2, frame2, mask = mask)
    output = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY)
    # show the images
    output = denoise_image(output)
    avX, avY = average_position_of_pixels(output, 120)
    print(avX, avY)
    myStrPub =table.getStringTopic("FoundRings").publish()
    myStrPub.set('{"X": avX, "Y": avY}' )
    #cv2.imshow("images", output)
    #cv2.waitKey(5)

   #frame = cv2.undistort(img, mtx, dist, None, newcameramtx)
   grayimage = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
   #cv2.imshow('frame', frame)
   #cv2.imwrite("fulmer2.jpg",frame)

   detections = detector.detect(grayimage)
   if detections:
       #print("Nothing")
       #cv2.putText(frame, "Nothing Detected", (500,500), cv2.FONT_HERSHEY_SIMPLEX, 2.0, (0, 0, 255), 2)
       #cv2.imshow('frame', frame)
       #cv2.imwrite("fulmer.jpg",frame)
       #cv2.waitKey(1)
       #iterates over all tags detected
       for detect in detections:
           pos, e1,f1=detector.detection_pose( detect, cameraParams, FRCtagSize, z_sign=1)
           print("POSE DATA START")
           print(pos, e1, f1)
           print("POSE DATA END")

           #sends the tag data named the t(str(detect.tag_id)).publish()ag_ID myStrPub =table.getStringTopic("tag1").publish()with Center, TopLeft, BottomRight Locations
           myStrPub =table.getStringTopic(str(detect.tag_id)).publish()
           myStrPub.set('{"Center": detect.center, "TopLft": detect.corners[0], "BotRht": detect.corners[2], "POS": pos, "e1": e1, "f1", f1}' )
           print("tag_id: %s, center: %s, corners: %s, corner.top_left: %s , corner.bottom-right: %s" % (detect.tag_id, detect.center, detect.corners[0:], detect.corners[0], detect.corners[2]))
           frame=plotPoint(frame, detect.center, (255,0,255)) #purpe center
           cornerIndex=0
           for corner in detect.corners:
               if cornerIndex== 0:
                print("top left corner %s" %(corner[0]))
                frame=plotPoint(frame, corner, (0,0,255)) #red for top left corner
                #xord=int(corner[0])
                #yord=int(corner[1])
                org=(int(corner[0]),int(corner[1]))
                tagId=("AprilTagId %s" % (str(detect.tag_id)))
                cv2.putText(frame, tagId, org, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
               elif cornerIndex == 2:
                print("bottom right corner %s" %(corner[0]))
                frame=plotPoint(frame, corner, (0,255,0)) #green for bottom right corner
               else:
                frame=plotPoint(frame, corner, (0,255,255)) #yellow corner
               cornerIndex+=1
       if not saved:
           #find a apriltag save the image catches programmers looking weird
           #cv2.imwrite("fulmer.jpg",frame)
           saved = True
           print("Saved!")
   #cv2.imshow('frame', frame)
   #cv2.waitKey(1)
   iteration = iteration + 1
   time.sleep(0.1)


version =ntcore.ConnectionInfo.protocol_version
print(" Remote ip: %s" % ntcore.ConnectionInfo.remote_ip)

#Closes everything out
vs.stop()
#cv2.destroyAllWindows()
