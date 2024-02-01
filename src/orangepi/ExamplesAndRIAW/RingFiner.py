# import the necessary packages
import numpy as np
import argparse
import cv2
from threading import Thread

class myWebcamVideoStream:
  def __init__(self, src=0):
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

#function that writes calibration output to json file 
def write_to_txt_file(filename, var1, var2, var3, var4):
    with open(filename, 'w') as txt_file:
        txt_file.write(f"{var1}\n{var2}\n{var3}\n{var4}\n")


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
vs = myWebcamVideoStream(0).start() 


# define the list of boundaries
boundaries = [
	([80,45,170], [100,145,255])
]

while True:
  # load the image
  image = vs.read()
  # loop over the boundaries
  for (lower, upper) in boundaries:
    # create NumPy arrays from the boundaries
    lower = np.array(lower, dtype = "uint8")
    upper = np.array(upper, dtype = "uint8")
    # find the colors within the specified boundaries and apply
    # the mask
    mask = cv2.inRange(image, lower, upper)
    output = cv2.bitwise_and(image, image, mask = mask)
    output = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY)
    # show the images
    avX, avY = average_position_of_pixels(output, 120)
    print(avX, avY)
    cv2.imshow("images", output)
    cv2.waitKey(5)