#!/usr/bin/env python3
# USAGE
# python3 ROITrackerFullFrames.py --video videos/Test2.avi --tracker csrt

#################################################################################################
# EXPERIMENT SPECIFIC! (ALTER THIS AS NECESSARY FOR EACH USE)                                   #
numberOfChildrenPlusRobot = 8 # the number of children in the play group + 1 for the robot     #
#################################################################################################

# IMPORTS 
#####################################
from imutils.video import FileVideoStream
from imutils.video import FPS
from imutils.video import VideoStream
import argparse
import imutils
import time
import cv2
import csv
import numpy as np
import pandas as pd
import collections
import time

# supresses warning about data type comparison between list and np.array 
import warnings
import numpy as np
warnings.simplefilter(action='ignore', category=FutureWarning)

# DEFINTIONS 
######################################
# function to check if something is empty 
def is_empty(any_structure):
    if any_structure:
        return False
    else:
        return True

# PRE-LOOP 
######################################
# construct the argument parser and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video", type=str,
    help="path to input video file")
ap.add_argument("-t", "--tracker", type=str, default="csrt",
    help="OpenCV object tracker type")
args = vars(ap.parse_args())

# initialize a dictionary that maps strings to their corresponding
# OpenCV object tracker implementations
OPENCV_OBJECT_TRACKERS = {
    "csrt": cv2.TrackerCSRT_create,
    "kcf": cv2.TrackerKCF_create,
    "boosting": cv2.TrackerBoosting_create,
    "mil": cv2.TrackerMIL_create,
    "tld": cv2.TrackerTLD_create,
    "medianflow": cv2.TrackerMedianFlow_create,
    "mosse": cv2.TrackerMOSSE_create
}

# if a video path was not supplied, grab the reference to the web cam
if not args.get("video", False):
    print("[INFO] starting video stream...")
    vs = VideoStream(src=0).start()
    time.sleep(1.0)

# otherwise, grab a reference to the video file
else:
    vs = cv2.VideoCapture(args["video"])
 

# initializing variables 
frameTrack = [] # tracks frame 
box = [] # iterates through each box in boxes 
box_title = [] # holds all titles that label the dataframe 
dist = [] # holds the distance for interaction classification
playarea = [] # holds the box of the playarea 
scale = [] # holds the scale, fills by pressing 's'
displayText = [None]*2 # holds text displayed on image

# dataframes for exported information
interactions = pd.DataFrame()
data = pd.DataFrame()
velocities = pd.DataFrame()

# placeholders 
trackers = [None]*numberOfChildrenPlusRobot # placeholder for trackers                          
placeholder = ["NR"]*numberOfChildrenPlusRobot # placeholder for centroid spaces 

# fills box_titles 
for count in range(numberOfChildrenPlusRobot):
    trackers[count] = cv2.MultiTracker_create()
    # header for the future DataFrame
    # create a list of all box titles with a space for the structure of the DataFrame
    if count == 0:
        box_title.append("Robot")
    else:
        box_title.append("Child " + str(count))
    # initializing boxes list
    boxes = [None]*numberOfChildrenPlusRobot

#initialize time variables
lastTime = 0 

#initialize counts 
frameCount = 0 # count to skip frames 
macroCount = 0 # total count 
childCount = 0 # child status count 

# constants for distance calculations 
dirSocialInteraction = 1 # ft
socialPlay = 3 # ft 
pix2ft = 50 # default set based on "Test2.avi" select new factor by pressing "s"

# Note: For all box indexes the format is as follows box = (x,y,w,h)
# where x and y are the coordinates of the top left corner of the box
# relative to the origin as the top left corner of the frame 
# w and h are the width and height of the box respectively  

# VIDEO LOOP 
######################################
# loop over frames from the video stream
while True:
    key = []
    # frame ID
    #frameId = int(round(vs.get(1))
    #    if frameID % multiplier == 0:
    #    if args.get("video", True):
    if not args.get("video", False):
        time_track = time.time() 
    else:
        time_track = vs.get(cv2.CAP_PROP_POS_MSEC)/1000    

    # grab the current frame, then handle if we are using a
    # VideoStream or VideoCapture object
    frame = vs.read()
    frame = frame[1] if args.get("video", False) else frame
    #increment global frame count 
    macroCount = macroCount + 1 

    # check to see if we have reached the end of the stream
    if frame is None:
        break

    # resize the frame (so we can process it faster)
    frame = imutils.resize(frame, width=1000)

    # grab the updated bounding box coordinates (if any) for each
    # object that is being tracked
    for count, tracker in enumerate(trackers):
        (success, box) = tracker.update(frame)
        if box != ():
            box = box[0]
            (x, y, w, h) = [int(v) for v in box]
            boxes[count] = box 
        else: 
            boxes[count] = 'NR'

    # defining playarea box 
    if is_empty(playarea) == False:
        # defining the top left and bottom right points of the bb to 
        # pass to rectangle method later
        startpoint = ((playarea[0]), (playarea[1]))
        endpoint = ((playarea[0]+playarea[2]), (playarea[1]+playarea[3]))
        # box characteristics
        color = [152, 154, 156] # light gray 
        thickness = 1 # random number 

        # creating the playarea rectangle on each frame loop 
        cv2.rectangle(frame, startpoint, endpoint, color, thickness)
        displayText[0] = 'Boundary: Engaged'
    else: 
        displayText[0] = 'Boundary: Not Engaged'

    # checks if scale has been established 
    if is_empty(scale) == False:
        displayText[1] = 'Scale: ' + str(round(pix2ft,3))
    else: 
        displayText[1] = 'Scale Not Chosen'

    for count, text in enumerate(displayText):
        if count == 0:
            # draw text to acknowledge boundary has been put into place 
            cv2.putText(frame, "{}".format(text), (10, 20),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        else:
            # draw text to acknowledge boundary has been put into place 
            cv2.putText(frame, "{}".format(text), (10, 40),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)


    # only enter if the play area has been defined 
    if is_empty(playarea) == False:
        centroid = []
        centroids = []
        # looping through boxes to determine where the children are relative to eachother and if they have
        # left the play area 
        for count_out, box in enumerate(boxes):
            if box != 'NR':
                # calculate the centroid of the box for all boxes that are registered 
                centroid = [box[0]+box[2]/2, box[1]+box[3]/2] # (x + 1/2 width, y + 1/2 height)
                
                # If all centroids of 'boxes' are in the play area this loop will append their centroids to a
                # list and draw their rectangles. Otherwise it will remove the listing from boxes that left
                # the play area, clear the tracker, and reinitialize it to track all but the omitted box 
                if centroid[0] > playarea[0] and \
                centroid[0] < playarea[0]+playarea[2] and \
                centroid[1] > playarea[1] and \
                centroid[1] < playarea[1]+playarea[3]: 
    
                    # append each centroid to a list 
                    centroids.append(centroid)
                    # x and y are coordinates of top left point 
                    (x, y, w, h) = [int(v) for v in box]
                    # numbering/labeling boxes 
                    if count_out != 0:
                        cv2.putText(frame, str(count_out), (int(centroid[0]), int(centroid[1])),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                        # display rectangles
                        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    else: 
                        cv2.putText(frame, str('r'), (int(centroid[0]), int(centroid[1])),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 64, 255), 2)
                        # display rectangles
                        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 64, 255), 2)
                else:
                    boxes[count_out] = 'NR' 
                    centroids.append('NR') # case where box for that child left frame
                    # deleting trackers and reinitializing  
                    del(trackers)
                    trackers = [None]*numberOfChildrenPlusRobot
                    for count_in, box in enumerate(boxes): 
                        trackers[count_in] = cv2.MultiTracker_create()
                        if box != 'NR': # if the box exists then re-initialize the tracker
                            trackerHolder = OPENCV_OBJECT_TRACKERS[args["tracker"]]()
                            box = tuple(box)
                            trackers[count_in].add(trackerHolder, frame, box) 

            else: # case where box for that child has not be chosen 
                centroids.append('NR')

        # instance that will store row of interactions to be placed in main 
        # interactions dataframe
        instance = pd.DataFrame()  
        # intPlaceholder is a list that is overwritten by interactions this is 
        # used to easily fill instance since DataFrames can be finnicky
        intPlaceholder = ["NR"]*numberOfChildrenPlusRobot

        # loop to calculate the distances between all centroids 
        if len(centroids) >= 2:
            # outer loop cycles through centroids, inner loop compares on centroid to the rest
            for count_out, centroid in enumerate(centroids):
                # converting to array for distance calculation
                centroid = np.array(centroid)
                # intializing interaction list that will be reset for each centroid 
                store = []
                # if the centroid is not registered the value in store that represents it 
                # should also be not registered 
                if centroid == 'NR':
                    store = ['NR']
                else: 
                    # looping through other centroids 
                    for count_in in range(len(centroids)): 
                        # if the centroid is not registered append NR to store for that centroid's
                        # position relative to the centroid that is looping 
                        if centroids[count_in] == 'NR':
                            dist = 'NR'
                        else:
                            centroids[count_in] = np.array(centroids[count_in])
                            # calculating the distance between centroids and converting to ft
                            dist = np.linalg.norm(centroid - centroids[count_in])*pix2ft

                        # categorizing distance between centroids 
                        if dist == 'NR': # dist is not registered bc centroid disappeared 
                            store.append('NR')
                        elif dist == 0.0: # 0 = Self Identification 
                            store.append('SELF')
                        elif dist < dirSocialInteraction: # < 3 ft = Directed Social Interaction (DSI)
                            if count_in == 0:
                                store.append('DSI with r')
                            else:
                                store.append('DSI with ' + str(count_in))
                        elif dist < socialPlay: # < 1 ft = Social Play (SP)
                            if count_in == 0:
                                store.append('SP with r')
                            else:
                                store.append('SP with ' + str(count_in))        
                        else: # if dist is larger than all categories there is no interaction (NI)
                            store.append('NI')
                            
                # storing interaction data 
                intPlaceholder[count_out:count_out+1] = [store]
            
            # creating dataframe to store centroids
            instance = pd.DataFrame([intPlaceholder],index = [time_track], columns = [box_title])
            # updating row of interactions   
            interactions = pd.concat([interactions, instance])

        for count, centroid in enumerate(centroids):
            if centroid != 'NR':
                # reverting centroids to list so they aren't stored as np.arrays
                centroid = centroid.tolist()
                centroids[count] = centroid
        

        if is_empty(frameTrack) == True:
            # switching frameTrack flag which starts concatenation of data in next frame 
            # from the video 
            frameTrack = True
            # initializing column headers and filling row with Nones
            data = pd.DataFrame([placeholder], index = [time_track], columns = [box_title])
            velocities = pd.DataFrame([placeholder], index = [time_track], columns = [box_title])
            # replace placeholders with centroid coordinates 
            data.iloc[0,0:len(centroids)] = centroids # first 0 index serves to constrain data replacement

        else:
            # initalize new row with placeholder values 
            addition = pd.DataFrame([centroids], index = [time_track], columns = [box_title])
            # concatenate to the dataframe
            data = pd.concat([data, addition])
            # indexing position from last frame 
            lastPlace = data.loc[lastTime,:]  
            # indexing position from current frame
            currentPlace = addition.loc[time_track,:]
            # velocity calculation
            velocity = [None]*numberOfChildrenPlusRobot
            for count in range(numberOfChildrenPlusRobot):
                if currentPlace[count] != 'NR' and lastPlace[count] != 'NR':
                    velocity[count] = (np.array(currentPlace[count])-np.array(lastPlace[count]))*pix2ft/(time_track-lastTime)
                elif currentPlace[count] == 'NR' or lastPlace[count] == 'NR':
                    velocity[count] = 'NR'
            velocityInstance = pd.DataFrame([velocity], index = [time_track], columns = [box_title])
            velocities = pd.concat([velocities, velocityInstance])
                    
    # show the output frame
    cv2.imshow("Frame", frame)
    # time.sleep(10)
    key = cv2.waitKey(1) & 0xFF
    # save last time for velocity calc
    lastTime = time_track

 # KEY COMMANDS (IN VIDEO LOOP)
 ######################################
    # defining key commands 
    keys = {ord("1"): 1, ord("2"): 2, ord("3"): 3, ord("4"): 4,
            ord("5"): 5, ord("6"): 6, ord("7"): 7, ord("8"): 8, 
            ord("9"): 9, ord("0"): 10, ord("r"): 0}
    if key in keys: 
        # if already tracking subject, then delete and reinitialize
        # this is for when the subject is occuled 
        number = keys[key]
        try:
            if trackers[number].getObjects() != ():
                trackers[number] = cv2.MultiTracker_create()

            # select the bounding box of the object we want to track (make
            # sure you press ENTER or SPACE after selecting the ROI)
            box = cv2.selectROI("Frame", frame, fromCenter=False,
                showCrosshair=True)            

            # create a new object tracker for the bounding box and add it
            # to our multi-object tracker
            trackerHolder = OPENCV_OBJECT_TRACKERS[args["tracker"]]()
            trackers[number].add(trackerHolder, frame, box)
        
        # ignore command if user enters an invalid key
        # this can occur because the number selected is out of range for the variable numberOfChildrenPlusRobot
        except IndexError:
            print("An invalid key was selected. Please try again.")
            pass

        # ignore error if user forgets to select a region after pausing to add a tracker
        except cv2.error:
            print("A region was not selected. Please try again.")
            pass

    # if 'b' key is pressed the bounding box of the play area will be selected
    elif key == ord('b') or key == ord('B'):
        playarea = cv2.selectROI("Frame", frame, fromCenter=False,
            showCrosshair=True)     
    # if "s" is pressed define scale 
    elif key == ord('s') or key == ord('S'):
        scale = cv2.selectROI("Frame", frame, fromCenter=False,
            showCrosshair=True)  
        
        squareDiagonal = (2**2+2**2)**(1/2) # assumes squares are 2x2 
        selectedDiagonal = (scale[2]**2+scale[3]**2)**(1/2) # scale[2] is width scale[3] is height         # length of square diagonal (feet) is divided by the selected diagonal (pixels)
        pix2ft = squareDiagonal/selectedDiagonal # multiply by a distance to convert into ft
         
    # if the `q` key was pressed, break from the loop
    elif key == ord("q") or key == ord('Q'):
        break

# CLEAN UP & EXPORT 
######################################
# if we are using a webcam, release the pointer
if not args.get("video", False):
    vs.stop()

# otherwise, release the file pointer
else:
    vs.release()

# close all windows
cv2.destroyAllWindows()

# exporting data to excel sheet 
data.to_excel('positions.xlsx', index = True, header=["Centroid"])
interactions.to_excel('interactions.xlsx', index = True, header=["Centroid"])
velocities.to_excel('velocities.xlsx', index = True, header=["Centroid"])
