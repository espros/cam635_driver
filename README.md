# cam635_driver

## General description
The TOFcam-635 is a miniaturized and cost-optimized 3D TOF camera. It is based on the ESPROS proprietary time-of-flight (TOF) technology using the epc635 TOF chip. The camera controls the illumination and the imager chip to obtain distance and grayscale images.
The cameras are calibrated to provide accurate 3D depth images. By using the ROS driver from ESPROS, 3D point clouds in a world cartesian coordinate system are available.
The depth images are compensated against DRNU errors, modulation errors, ambient-light, temperature and reflectivity of the scene. Thanks to the high performance of the imager chip with the unique ambient-light suppression, the camera can be used in many cases under full sunlight conditions. The output of the TOFcam-635 is depth and grayscale images – allowing a variety of new applications, e.g. for mobile robotics. This module brings you right in front with the latest technology of 3D depth sensing. All the complex engineering and time consuming design tasks regarding optics, illumination and signal processing are already solved.

## Features
Field of view (FOV) of 50° x 19° (h x v)
Distance measurement ranges on white target:
Wide FOV: 0.1 ... 7.5 m, center beam (NFOV): 1.0 ... 15 m
Measurement rate up to 50 TOF measurements per second
Sun- and ambient-light tolerant up to 100 kLux
Calibrated and compensated
Temperature compensated
Easy to use
High speed serial interface UART 10 Mbit/s
Low power consumption
ROS driver available
Customized versions possible

## Applications
Distance measurement from centimeters to a few meters
Mobile robots, automatic vehicle guidance, collision avoidance
Scanner for SLAM data acquisition in mobile robots
People and object counting, in-cabin monitoring
Door opening, machine controlling and safeguarding, IoT
Gesture control (man-machine-interface)
Object classification

## Installation
System requirement: Linux operating system.
Download the “TOFCAM635_SW_Package” from the website www.espros.com, section Downloads, 02_Cameras_and_Modules.
There is enclosed the “TOFCAM635_ROS_driver” file. Unpack this ZIP file.
This driver works with two types of cameras: TOFcam635 and TOFcam635-S. The driver automatically detects which camera is connected. The camera TOFcam635-S has less parameters and unusable parameters will be ignored.

## Installation for Raspberry Pi
System recommended: Raspberry Pi4, Ubuntu 20.4, ROS Noetic.

1. Download and install Ubuntu 20.4 server https://ubuntu.com/download/raspberry-pi
2. > sudo apt update
3. > sudo apt install ubuntu-desktop
4. Install ROS noetic http://wiki.ros.org/noetic/Installation/Ubuntu
5. > sudo apt install ros-noetic-pcl-conversion
6. > sudo apt install ros-noetic-pcl-ros
7. > sudo apt-get install build-essential

## Running the ROS driver
Change to the home directory and open the bash-file:
> cd ~
> gedit .bashrc

Insert the following line at the end of the bash-file:
source ~/projects/cam635_driver/devel/setup.bash

Save the file and exit editor.

Start the ROS with GUI in terminal mode with the following command:
roslaunch espros_cam635 camera.launch

If you use in terminal mode the APIs only, without GUI: 
Start the ROS operating system in a Terminal1 with the command:	roscore
Start the TOFcam635 in a Terminal2 with the command: 		rosrun espros_tof_cam635 tof_cam635_node

More info about topics and dynamically reconfigurable parameters you will find in ROS_TOFcam635_v1.5.pdf document

