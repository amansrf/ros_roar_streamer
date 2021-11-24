<!-- TABLE OF CONTENTS -->
<details open="open">
  <summary><h2 style="display: inline-block">Table of Contents</h2></summary>
  <ol>
    <li>
      <a href="#about-the-package">About The Package</a>
    </li>
    <li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#prerequisites">Prerequisites</a></li>
        <li><a href="#installation">Installation</a></li>
      </ul>
    </li>
    <li><a href="#Usage">Usage</a></li>
    <li><a href="#contact">Contact</a></li>
  </ol>
</details>



<!-- ABOUT THE Package -->
## About The Package

This is a package to initialise communication with an iOS device running the ROAR_ios codebase, and establish communication with ROS. It connects to the iOS device and reads sensor data from the camera, depth-camera, imu, odometry and publishes the same on various rostopics in standard ROS message formats. It also listens for controls on a rostopic and streams the same to the iOS device.


<!-- GETTING STARTED -->
## Getting Started

To get a local copy up and running follow these simple steps.

### Prerequisites

It is assumed that your system(s) is running Ubuntu 18.04/20.04 and has a working installation of ROS1 (Melodic or later). This repo has not been tested on other linux distributions.

Please follow instructions on how to install [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) if you are using Ubuntu 18.04, or [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) if you are using Ubuntu 20.04. The desktop-full version needs to be installed.



### Installation

1. Make a ROS workspace if not done already.
    ```bash
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/src
    ```

2. Clone the repository and its dependencies
    ```bash
    git clone https://github.com/amansrf/ros_roar_streamer.git
    ```
3. Install required ROS packages
    ```bash
    cd ~/catkin_ws
    sudo apt-get update
    sudo rosdep init
    rosdep update
    rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y
    ```
4. Create a virtual environment and install python packages 
    ```bash
    virtualenv venv -p python3
    source ~/catkin_ws/venv/bin/activate
    pip install -r ~/catkin_ws/src/ros_roar_streamer/${ROS_DISTRO}_requirements.txt
    ```
5. Build all the packages and source the workspace
    ```bash
    catkin_make
    source ~/catkin_ws/devel/setup.bash
    ```

<!-- LICENSE -->
<!-- ## License
Distributed under the MIT License. See `LICENSE` for more information. -->

<!-- USAGE -->
## USAGE

Calibrate the iPhone and change the IP Address in 
~/catkin_ws/src/ros_roar_streamer/config.py to match what is shown in the app

Then launch the various ROS Nodes using:

```bash
roslaunch ros_roar_streamer streamer.launch
```

You should see a depth image stream and an rgb image stream on your screen!

<!-- CONTACT -->
## Contact
If you have any issues or find any bugs please feel free to contact:

Aman Saraf     - amansaraf99@gmail.com, aman_saraf@berkeley.edu
Sunisha Fernandez - sunisha.fernandez@gmail.com, sunisha.fernandez@berkeley.edu 


<!-- ACKNOWLEDGEMENTS -->
<!-- ## Acknowledgements and References

TO DO -->