import ntcore
import cv2
from threading import Thread

class myWebcamVideoStream:
  def __init__(self, src=1):

    global table

    ntinst = ntcore.NetworkTableInstance.getDefault()
    table = ntinst.getTable("PiDetector")
    ntinst.startClient4("pi1 vision client")
    ntinst.setServer("10.56.7.2")

    # initialize the video camera stream and read the 
    # first frame from the stream
    self.stream = cv2.VideoCapture(0) 
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

vs = myWebcamVideoStream(0).start()

while True:
   frame = vs.read()
   myStrPub = table.getStringTopic("RAWCamera").publish()
   myStrPub.set('{"Cam": frame}' )