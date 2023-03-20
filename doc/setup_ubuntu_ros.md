# Setup Ubuntu & ROS Melodic
To be able to use ROS Melodic, we need to install the following Ubuntu environment: Ubuntu 18.04.07. 

This version is officially supported by ROS Melodic.

Now follow these instructions to set up ROS Melodic for your Ubuntu environment:
    http://wiki.ros.org/melodic/Installation/Ubuntu

# Creating a workspace
To successfully run the code, a separate workspace needs to be created where all necessary libraries and codes will be located later.

```
mkdir -p ~/catkin_ws/src
```

Navigate to the "src" directory within your workspace:

```
cd ~/catkin_ws/src
```

Initialize the workspace by running the catkin_init_workspace command:

```
catkin_init_workspace
```

Navigate back to the root of your workspace:

```
cd ~/catkin_ws
```

Build the workspace by running the catkin_make command:

```
catkin_make
```

Finally, activate your workspace by sourcing the setup script:

```
source ~/catkin_ws/devel/setup.bash
```

# Import the RPLIDAR ROS package into catkin_ws
Slamtec provides a default code that can both capture and output data. 
To create the necessary programming environment, we will use their source folder for the first time, which will then be modified.
Open a terminal and navigate to your catkin workspace src folder:

```
cd ~/catkin_ws/src
```

Clone the project repository:

```
git clone https://github.com/Slamtec/rplidar_ros.git
```

Navigate to the root of your catkin workspace:

```
cd ~/catkin_ws
```

Run the catkin_make command to build the workspace:

```
catkin_make
```

If the build completes successfully, activate your catkin workspace:

```
source ~/catkin_ws/devel/setup.bash
```

# Run Code
To address the correct port, it first needs to be determined to which port the Lidar is connected.

```
ls -l /dev |grep ttyUSB
```

Add the authority of write: (such as /dev/ttyUSB0)

```
sudo chmod 666 /dev/ttyUSB0
```

To start the launcher, you must now call the following command:

```
roslaunch rplidar_ros rplidar.launch
```

Alternatively, you can display the data via the terminal window. The command for this is:

```
rosrun rplidar_ros rplidarNodeClient
```
