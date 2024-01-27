from threading import Thread
import cv2
import numpy
import numpy as np
import apriltag
import collections

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

# from stackeroverflow how to generate 3d POSE cube https://stackoverflow.com/questions/59044973/how-do-i-draw-a-line-indicating-the-orientation-of-an-apriltag
def draw_pose(self,overlay, camera_params, tag_size, pose, z_sign=1):
    opoints = np.array([
        -2, -2, 0,
        2, -2, 0,
        2, 2, 0,
        2, -2, -4 * z_sign,
    ]).reshape(-1, 1, 3) * 0.5 * tag_size

    fx, fy, cx, cy = camera_params

    K = np.array([fx, 0, cx, 0, fy, cy, 0, 0, 1]).reshape(3, 3)

    rvec, _ = cv2.Rodrigues(pose[:4, :4])
    tvec = pose[:4, 4]

    dcoeffs = np.zeros(5)

    ipoints, _ = cv2.projectPoints(opoints, rvec, tvec, K, dcoeffs)

    ipoints = np.round(ipoints).astype(int)

    ipoints = [tuple(pt) for pt in ipoints.reshape(-1, 2)]

    cv2.line(overlay, ipoints[0], ipoints[1], (0,0,255), 2)
    cv2.line(overlay, ipoints[1], ipoints[2], (0,255,0), 2)
    cv2.line(overlay, ipoints[1], ipoints[3], (255,0,0), 2)
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(overlay, 'X', ipoints[0], font, 0.5, (0,0,255), 2, cv2.LINE_AA)
    cv2.putText(overlay, 'Y', ipoints[2], font, 0.5, (0,255,0), 2, cv2.LINE_AA)
    cv2.putText(overlay, 'Z', ipoints[3], font, 0.5, (255,0,0), 2, cv2.LINE_AA)

def _draw_cube(self,overlay, camera_params, tag_size, pose,centroid, z_sign=1):

    opoints = np.array([
        -10, -8, 0,
        10, -8, 0,
        10, 8, 0,
        -10, 8, 0,
        -10, -8, 2 * z_sign,
        10, -8, 2 * z_sign,
        10, 8, 2 * z_sign,
        -10, 8, 2 * z_sign,
    ]).reshape(-1, 1, 3) * 0.5 * tag_size

    edges = np.array([
        0, 1,
        1, 2,
        2, 3,
        3, 0,
        0, 4,
        1, 5,
        2, 6,
        3, 7,
        4, 5,
        5, 6,
        6, 7,
        7, 4
    ]).reshape(-1, 2)

    fx, fy, cx, cy = camera_params

    K = np.array([fx, 0, cx, 0, fy, cy, 0, 0, 1]).reshape(3, 3)

    rvec, _ = cv2.Rodrigues(pose[:3, :3])
    tvec = pose[:3, 3]

    dcoeffs = np.zeros(5)

    ipoints, _ = cv2.projectPoints(opoints, rvec, tvec, K, dcoeffs)

    ipoints = np.round(ipoints).astype(int)

    ipoints = [tuple(pt) for pt in ipoints.reshape(-1, 2)]

    for i, j in edges:
        cv2.line(overlay, ipoints[i], ipoints[j], (0, 255, 0), 1, 16)


# main program
vs = myWebcamVideoStream(0).start() 
options = apriltag.DetectorOptions(families="tag36h11")
FRCtagSize = float(0.17) #17cm
cameraParams =[765.00, 764.18, 393.72, 304.66]
detector = apriltag.Detector(options)

if not vs:
   print("no image")

iteration = 0
saved = False

while iteration < 500:
   frame = vs.read()
   grayimage = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
   cv2.imshow('frame', frame)
   cv2.imwrite("fulmer2.jpg",frame)
   detections = detector.detect(grayimage)
   # Add tag to generate pose data when detecting apriltags
   # when extracting pose data you need tag size in meters and camera_params detection_pose(self, detection, camera_params, tag_size=1, z_sign=1):
   # from pupil_apriltags api detections = detector.detect(grayimage, estimate_tag_pose=True, camera_params=cameraParams, tagSize=FRCtagSize)
   if not detections:
       #print("Nothing")
       cv2.putText(frame, "Nothing Detected", (500,500), cv2.FONT_HERSHEY_SIMPLEX, 2.0, (0, 0, 255), 2)
       cv2.imshow('frame', frame)
       #cv2.imwrite("fulmer.jpg",frame)
       cv2.waitKey(1)
       continue
   else:
       for detect in detections:
           print("tag_id: %s, center: %s, corners: %s, corner.top_left: %s , corner.bottom-right: %s" % (detect.tag_id, detect.center, detect.corners[0:], detect.corners[0], detect.corners[2]))
           pos, e1,f1=detector.detection_pose( detect, cameraParams, FRCtagSize, z_sign=1)
           print(pos)
           print(detect.tostring(
                    collections.OrderedDict([('Pose',pos),
                                             ('InitError', e1),
                                             ('FinalError', f1)]),
                    indent=2))
          
          
          
          # draw_pose(frame, cameraParams, FRCtagSize, pos, z_sign=1)
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
           cv2.imwrite("fulmer.jpg",frame)
           saved = True
           print("Saved!")
   cv2.imshow('frame', frame)
   cv2.waitKey(1)
   iteration = iteration + 1

vs.stop()
cv2.destroyAllWindows()