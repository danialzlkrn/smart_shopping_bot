# smart_shopping_bot

## ROS Configuration and User Manual for Smart Shopping Bot

---

## ROS Configuration (if there is no current workspace)

Make sure the workspace environment is set up as follows:

### ROS Noetic Installation

Follow the official installation guide:  
http://wiki.ros.org/noetic/Installation

### Create a ROS workspace

$ mkdir -p ~/catkin_ws/src  
$ cd ~/catkin_ws/  
$ catkin_make

### Create a ROS package

$ cd ~/catkin_ws/src/  
$ catkin_create_pkg med_buddy roscpp rospy std_msgs

### Build catkin workspace

$ cd ..  
$ catkin_make

### Source setup file

$ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc  
$ source ~/.bashrc

---

## User Manual

### Clone the project repository from GitHub

$ cd ~/catkin_ws/src/  
$ git clone https://github.com/danialzlkrn/smart_shopping_bot.git

### Install dependencies if it does not exist yet

$ pip install ultralytics opencv-python pyttsx3 SpeechRecognition torch torchvision torchaudio numpy

### If the base dependencies already exist

$ pip install ultralytics

### Build catkin workspace

$ cd ~/catkin_ws/  
$ catkin_make

---

## To launch the robot application, open 4 terminals and run the following commands:

### Terminal 1

$ roscore

### Terminal 2

$ roslaunch smart_shopping_bot smart_shopping.launch

### Terminal 3 (For CLI display)

$ rosrun smart_shopping_bot cli_display_py

### Terminal 4 (To start the USB camera)

$ roslaunch usb_cam usb_cam-test.launch

**(Optional, if you want to test the speech recognition function only)**

$ roslaunch smart_shopping_bot speech_function.launch

---

## User Speech Guidelines

The function is separated into two categories: Robot Speech Function and Robot Vision Function.

### Robot Speech Function

Right after running `speech_function.launch` or `smart_shopping.launch`, the user can ask:

**Asking product price:**
- "What is the price of _______"
- "________ price"
- "________ cost"

**Asking product location:**
- "Where is ________"

**Asking product information:**
- "Describe ________"

### Robot Vision Function

Right after running `smart_shopping.launch`, the user can ask:

**Recognizing product:**
- "What is this"

**Asking product price:**
- "What is the price of _______"
- "________ price"
- "________ cost"

**Asking product location:**
- "Where is ________"

**Asking product information:**
- "Describe ________"

If the robot receives different input, it will reply with:  
**"Sorry, I didn't catch that."**
