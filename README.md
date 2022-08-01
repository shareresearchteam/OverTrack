# OverTrack
This tool is used to track regions-of-interest (ROI) from a single overhead camera by drawing bounding boxes and publishing those locations in real time using a Robot-Operating-System (ROS) network. The labeled arUco tag scripts enable tracking of a single arUco marker in addition to the bounding boxes.

**ROITrackerFullFrames** - This file will run the tracker with either a video or in real-time with no ROS setup.
**ROITrackerFullFramesLegacy** - This file is the same as the original tracker but incorporates the legacy versions of the OpenCV MultiTracker tool for use with certain versions of Python3 and OpenCV.  
**ROITrackerFullFramesArUcoROS** - This file can be used with versions of ROS that are older than Noetic.  
**ROITrackerFullFramesArUcoROSLegacy** - This file is the same as the ROS file but incorporates the legacy versions of the OpenCV MultiTracker tool for use with certain versions of Python3 and OpenCV. This version is most compatible with ROS Noetic.  

# Compatibility
This tool has been successfully tested on Linux (Ubuntu), Mac OS X Maverick, and Windows 10 and 11 machines.

For Linux machines, this has been successfully tested on Ubuntu 16.04, 18.04, and 20.04.  
This tool has been successfully tested on ROS versions Kinetic, Melodic, and Noetic. For ROS Noetic, the legacy tracker must be used to work with Python3.

# Linux Installation

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

# Windows Installation
These instructions do not include setting up ROS with Windows so only the non-ROS scripts will work correctly. Please see http://wiki.ros.org/Installation/Windows for further instructions on configuring ROS with Windows.
```
1. Install Windows Terminal if available - https://docs.microsoft.com/en-us/windows/terminal/install
2. Install Python if not already installed - https://www.python.org/downloads/windows/
3. Install dependencies listed above
4. Clone repository using either Terminal or web GUI.
6. Navigate to the cloned repository in Terminal and run script of choice.

# Mac Installation
Installation on Mac follows Linux instructions. The XCode Command Line Tools package or Homebrew may need to be installed:
https://www.freecodecamp.org/news/install-xcode-command-line-tools/

# Usage

The following commands are used to operate the tool. You must create a boundary first before the trackers will be visualized. They will still be tracked without a boundary but will not display.

**b** - Allows user to draw perimeter box. This is needed to ensure that if a tracked ROI leaves the perimeter, then it will be removed and can be re-drawn when the ROI comes back within the perimeter box.  
**s** - Allows user to draw a box that defines the scale. TODO - does this need some tweaking?  
**r** - Allows user to draw a box over the robot and track it independently of other ROI in the environment.  
**1-9** - Allows user to draw a box over a ROI and track it with the numerical indicator. The available number of boxes corresponds to the variable numberOfChildrenPlusRobot at the top of each script.  
**c** - Cancels a drawing operation.  
**q** - Quits the program and saves the data. Using control+c will not save the data to a spreadsheet.

# Other Notes
If this tool is being used on Windows without Terminal, the file path much be specified. This is at the end of each script and an example of the file path listing is shown below.
```
data.to_excel(r'C:\Users\Connor\Desktop\OSU_Lab\multi-object-tracking\Tracking Data\data.xlsx', index = True, header=["Centroid"])
interactions.to_excel(r'C:\Users\Connor\Desktop\OSU_Lab\multi-object-tracking\Tracking Data\interactions.xlsx', index = True, header=["Centroid"])
```

The variable numberOfChildrenPlusRobot can be altered if more bounding boxes are needed (up to a maximum of 10). 

# Running without ROS
Example command for running on a video file (put video file in a folder called videos next to the script):
```python3 ROITrackerFullFrames.py --video videos/video_name.video_type --tracker csrt```

If using the aruco file, add parameter --type (define aruco dictionary type) (default is DICT_5X5_1000)
