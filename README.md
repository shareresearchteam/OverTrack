# infant_robot_interaction
infant robot interaction project

ROITrackerFullFrames is the most basic version of the tracker  
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

Update path for excel sheet. Open two blank excel books and save them and enter the path to them at the bottom of the code.

exporting data to excel sheet 
data.to_excel(r'C:\Users\Connor\Desktop\OSU_Lab\multi-object-tracking\Tracking Data\data.xlsx', index = True, header=["Centroid"])

interactions.to_excel(r'C:\Users\Connor\Desktop\OSU_Lab\multi-object-tracking\Tracking Data\interactions.xlsx', index = True, header=["Centroid"])


python3 ROITrackerFullFrames.py --video videos/video_name.video_type --tracker csrt

Alter numberOfChildrenPlusRobot variable to the number of children in the play group + 1 for the robot

