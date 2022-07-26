# *OverTrack*: A Semi-Automatic Overhead Region-of-Interest Tracker
This tool is used to track regions-of-interest (ROI) from a single overhead camera by drawing bounding boxes and publish those locations in real time over a Robot-Operating-System (ROS) network. In some versions of this package, a single arUco marker can be tracked automatically in addition to the bounding boxes.

**ROITrackerFullFrames** - This file is the original and simplest version of the tracker.  
**ROITrackerFullFramesLegacy** - This file is the same as the original tracker but incorporates the legacy versions of the OpenCV MultiTracker tool for use with certain versions of Python3 and OpenCV.  
**ROITrackerFullFramesArUco** - This file incorporates tracking one aruco marker automatically. The marker would typically be placed on the robot.  
**ROITrackerFullFramesArUcoROS** - This file can be used with versions of ROS that are older than Noetic.  
**ROITrackerFullFramesArUcoROSLegacy** - This file is the same as the ROS file but incorporates the legacy versions of the OpenCV MultiTracker tool for use with certain versions of Python3 and OpenCV. This version is most compatible with ROS Noetic.  
**ROITrackerThreadedFrames** is the same as FullFrames, but it processes the video in a separate thread to increase speed. This has not been tested with ROS
**ROITrackerFullFramesFrame** - This file runs the video frame by frame if more accuracy is needed and time is not a factor.

# Compatibility
This tool has been successfully tested on Linux (Ubuntu) and Windows machines.

For Linux machines, this has been successfully tested on Ubuntu 18.04 and 20.04. For Windows, this has been successfully tested on Windows 10.  
This tool has been successfully tested on ROS versions Melodic and Noetic. For ROS Noetic, the legacy tracker must be used to work with Python 3.

# Installation

1. Install Dependencies:
```
numpy - pip3 install numpy
imutils - pip3 install imutils
cv2 - pip3 install opencv-contrib-python (needs to be contrib version)
pandas - pip3 install pandas
openpyxl - pip3 install openpyxl
```

2. Create a catkin workspace (if you do not have one already)
3. Navigate to the src folder in your catkin workspace: ```cd ~/catkin_ws/src```
4. Create a folder called camera_tracking. If you wish to use a different package name, update the CMakeLists.txt and launch file with the appropriate package name.
5. Clone this repository: ```git clone https://github.com/shareresearchteam/Overhead_Tracking_System```
6. Compile and build the package: ```cd ~/catkin_ws && catkin_make```
7. Add the catkin workspace to your ROS environment: ```source ~/catkin_ws/devel/setup.bash```
8. Run the launch file depending on your version of ROS: ```roslaunch camera_tracking camera_tracking.launch```

# Usage

The following commands are used to operate the tool. You must create a boundary first before the trackers will be visualized. They will still be tracked without a boundary but will not display.

**b** - Allows user to draw perimeter box. This is needed to ensure that if a tracked ROI leaves the perimeter, then it will be removed and can be re-drawn when the ROI comes back within the perimeter box.  
**s** - Allows user to draw a box that defines the scale. TODO - does this need some tweaking?  
**r** - Allows user to draw a box over the robot and track it independently of other ROI in the environment.  
**1-9** - Allows user to draw a box over a ROI and track it with the numerical indicator. The available number of boxes corresponds to the variable numberOfChildrenPlusRobot at the top of each script.  
**c** - Cancels a drawing operation.  
**q** - Quits the program and saves the data. Using control+c will not save the data to a spreadsheet.

# Other Notes
If this tool is being used on Windows, the file path much be specified. This is at the end of each script and an example of the file path listing is shown below.
```
data.to_excel(r'C:\Users\Connor\Desktop\OSU_Lab\multi-object-tracking\Tracking Data\data.xlsx', index = True, header=["Centroid"])
interactions.to_excel(r'C:\Users\Connor\Desktop\OSU_Lab\multi-object-tracking\Tracking Data\interactions.xlsx', index = True, header=["Centroid"])
```

The variable numberOfChildrenPlusRobot can be altered if more bounding boxes are needed (up to a maximum of 10). 

# Running without ROS
Example command for running on a video file (put video file in a folder called videos next to the script):
```python3 ROITrackerFullFrames.py --video videos/video_name.video_type --tracker csrt```

If using the aruco file, add parameter --type (define aruco dictionary type) (default is DICT_5X5_1000)
