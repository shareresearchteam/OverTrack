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

Update path to your computer
