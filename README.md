# OverTrack
This tool is used to track regions-of-interest (ROI) from a single overhead camera by drawing bounding boxes and publishing those locations in real time using a Robot-Operating-System (ROS) network. The labeled arUco tag scripts enable tracking of a single arUco marker in addition to the bounding boxes.

This tool was constructed in part using OpenCV (https://opencv.org/), imutils (https://github.com/PyImageSearch/imutils), and pandas (https://pandas.pydata.org/).

**OverTrackFullFrames** - This file will run the tracker with either a video or in real-time with no ROS setup.

**OverTrackFullFramesLegacy** - This file is the same as the original tracker but incorporates the legacy versions of the OpenCV MultiTracker tool for use with certain versions of Python3 and OpenCV.  

**OverTrackFullFramesArUcoROS** - This file can be used with versions of ROS that are older than Noetic.  

**OverTrackFullFramesArUcoROSLegacy** - This file is the same as the ROS file but incorporates the legacy versions of the OpenCV MultiTracker tool for use with certain versions of Python3 and OpenCV. This version is most compatible with ROS Noetic.  

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
4. Clone this repository: ```git clone https://github.com/shareresearchteam/OverTrack```
5. Compile and build the package: ```cd ~/catkin_ws && catkin_make```
6. Add the catkin workspace to your ROS environment: ```source ~/catkin_ws/devel/setup.bash```
7. Run the launch file depending on your version of ROS: ```roslaunch camera_tracking camera_tracking.launch```

# Windows Installation
Only the non-ROS scripts have been tested with Windows. These instructions do not include setting up ROS with Windows. Please see http://wiki.ros.org/Installation/Windows for further instructions on configuring ROS with Windows.

1. Install Windows Terminal if available - https://docs.microsoft.com/en-us/windows/terminal/install
2. Install Python if not already installed - https://www.python.org/downloads/windows/
3. Install dependencies listed above
4. Clone repository using either Terminal or web GUI.
6. Navigate to the cloned repository in Terminal and run script of choice using Running Without ROS commands.

If you have issues with modules not being found while running, it is possible there is a Python version mismatch. Try uninstalling (pip3 uninstall dependency_name) and then using python3 -m pip install dependency_name for installing dependencies.

# Mac Installation
Installation on Mac follows Linux instructions. The XCode Command Line Tools package or Homebrew may need to be installed:
https://www.freecodecamp.org/news/install-xcode-command-line-tools/

# Usage

The following commands are used to operate the tool. You must create a perimeter box first before the trackers will be visualized. They will still be tracked without a perimeter box but will not display.

**b** - Allows user to draw perimeter box. This is needed to ensure that if a tracked ROI leaves the perimeter, then it will be removed and can be re-drawn when the ROI comes back within the perimeter box.  
**s** - Allows user to draw a box that defines the scale. 
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

# Citation
Please cite this paper in your publications if OverTrack helps your research.

Ameer Helmi, Connor Phillips, Fernando Castillo, Christine Zhan, Samuel W. Logan, and Naomi T. Fitter, "OverTrack: Overhead Camera Tracking Tool for
Child-Robot Interaction" Under Review for Inclusion in the International Conference on Social Robotics (ICSR), 2022. 
```
@inproceedings{helmi2022overtrack,
  title={OverTrack: Overhead Camera Tracking Tool for Child-Robot Interaction},
  author={Helmi, Ameer and Phillips, Connor and Castillo, Fernando and Zhan, Christine and Logan, Samuel W. and Fitter, Naomi T.},
  howpublished={Under Review for Inclusion in the International Conference on Social Robotics (ICSR)},
  pages={},
  year={2022},
}
```
