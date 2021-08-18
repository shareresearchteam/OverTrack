# Infant-Robot Interaction
video tracking software

ROITrackerFullFrames is the most basic version of the tracker 
ROITrackerFullFrames_slim is a slimmed version of the basic tracker. Only tracks position and does not do the distance and other calculations.
ROITrackerFullFrames_aruco - uses an aruco marker on the robot to track position of the robot automatically 
ROITrackerThreadedFrames is the same as FullFrames, but it processes the video in a separate thread to increase speed
ROITrackerSelectFrames can take a frame every N seconds, this was deemed not useful due to issues with tracker accuracy caused by missing frames 

Dependencies:
```
numpy - pip3 install numpy
imutils - pip3 install imutils
cv2 - pip3 install opencv-contrib-python (needs to be contrib version)
pandas - pip3 install pandas
openpyxl - pip3 install openpyxl
```

Update path for excel sheet. Open two blank excel books and save them and enter the path to them at the bottom of the code. Example shown below
```
data.to_excel(r'C:\Users\Connor\Desktop\OSU_Lab\multi-object-tracking\Tracking Data\data.xlsx', index = True, header=["Centroid"])
interactions.to_excel(r'C:\Users\Connor\Desktop\OSU_Lab\multi-object-tracking\Tracking Data\interactions.xlsx', index = True, header=["Centroid"])
```

Alter numberOfChildrenPlusRobot variable to the number of children in the play group + 1 for the robot

To run:
python3 ROITrackerFullFrames.py --video videos/video_name.video_type --tracker csrt 

for the aruco file, add parameter --type (define aruco dictionary type) (default is DICT_5X5_1000)
