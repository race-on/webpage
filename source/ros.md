title: Ros Overview Setup

# What is ROS?
Ros is open-source framework that aims to simplify the complexities involved in writing robust, general robotics software. [Read more about ROS here.](https://http://wiki.ros.org/ROS/Introduction "Title")

## Ros Concepts
ROS has a number of important filesystem level concepts, including packages, metapackages, package manifests, repositories, message types and service types. For RaceOn, however, we'll focus mostly on the use of **packages** and **message types**. 

Additionally, ROS has several important concepts related to its "computation graph". "The Computation Graph is the peer-to-peer network of ROS processes that are processing data together." For the sake of Race On, the computation graph is composed of a number of ROS nodes that communicate through a publish/subscribe architecture. The pieces of the computational graph that are most relevant to RaceOn are **nodes**, **messages**, and **topics**. 

For an overview of ROS concepts, read [here](http://wiki.ros.org/ROS/Concepts "Title").
### Packages

In ROS, **packages** are the primary means of organizing software and are the smallest item you can build and release. They can contain any collection of files that should logically be contained together.

### Message Types

**Message types** describe message data structures and outline which data types they contain. When creating a message type, they should be contained in ```my_package/msg/MyMessageType.msg```. For more info about defining a custom message type in ROS, check out [creating a ROS msg and srv](https://http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv "Title").

### Nodes

**Nodes** are modular processes that perform some computation for your system. They should be a logical unit within your sytem. For example, for RaceOn, we're using 4 nodes (actuation.py, camera.py, control.py and pos_estimation.py), all of which handle a specific part of the car's behavior. Each of these nodes communicate by publishing and subscribing to messages on different **topics**. 

### Messages
**Messages** are simple data structures that can contain a combination of basic data types. 

### Topics
Nodes can send out and receive messages by publishing and subscribing to **topics**. Each topic has a name, which should be associated with the message types or message content being sent through it. For example, the control node in the race-on-ros repository subscribes to Pose messages on the ```position/error``` topic and publishes AckermanDrive messages on the ```control``` topic. Note that you can only send one message type through a single topic. [This is a classic ROS tutorial for setting up publisher and subscriber nodes.](https://http://wiki.ros.org/ROS/Concepts "Title")

For a more in depth discussion of common ROS concepts, check out [ROS concepts](https://http://wiki.ros.org/ROS/Concepts "Title").

# Preparation

Before we proceed with this quickstart quide we need to perform a few preparation steps. If you plan to perform these steps using a Jupyter terminal instead of ssh, please before starting close all Jupyter tabs except the terminal.

1. Update the list of packages and install all the available software updates.
    ```bash
    sudo apt update
    sudo apt upgrade
    ```
    
1. Install the OpenCV library for Python3. We will use the OpenCV functions for camera calibration and image preprocessing.
    ```bash
    sudo apt install python3-opencv
    ```
    
1. Instruct ROS to use Python 3 by setting the ROS_PYTHON_VERSION environment variable. 
    ```bash
    echo "export ROS_PYTHON_VERSION=3" >> ~/.bashrc
    ```
    The ```.bashrc``` file in the home folder contains all the commands which are executed when you open a terminal.

1. The ```startup``` folder in the home directory contains the scripts which are run everytime the Pi boots to setup the hardware and start the Jupyter and the ROS services. We need to modify the ROS script to allow the external devices connect to the ROS running on the Pi. For that open the file ```~/startup/ros.sh``` using the Jupyter interface and, if present, remove the line ```export ROS_HOSTNAME=raspberrypi"```. In addition, since these scripts are crucial for the Race On platform to function properly we will make them read only to prevent accedental edits. For that run the following command in the terminal.
    ```bash
    chmod a-w ~/startup/*
    ```
    where the argument ```a``` stands for all, ```-``` remove, and ```w``` write permission.

1. Now we can prepare the ROS workspace. The ```race-on-ros``` folder contains the GitHub repository of the Race On team and only Race On organizers can push updates back to GitHub. To solve this issue we need to first delete the folder, fork the GitHub repository using your own GitHub account where you can add collaborators, and then clone back on Pi the forked repository. To delete the folder, run 
    ```bash
    rm -rf ~/race-on-ros
    ``` 
    Next, go to the GitHub page of the [race-on-ros](https://github.com/race-on/race-on-ros) repository and click the fork button on the upper left corner. After a few seconds you should be redirected to your own ```race-on-ros``` repository. Clone the forked repository using the following command
    ```bash
    git clone https://github.com/<your-username>/race-on-ros.git
    ```
    Now you should see again the ```race-on-ros``` folder in your home directory but this time the repository is linked to your own GitHub account.
    
1. Before we can use the raceon package we need to build it since only source files are commited to GitHub. The following two commands are crucial and if you later experience problems with the ROS code then most likely you forgot to run them. You need to run these commands every time you add new files or dependencies to your packages so ROS knows about them.
    ```bash
    catkin_make -C ~/race-on-ros/
    source ~/race-on-ros/devel/setup.bash
    ```
    First line compiles all the packages in the workspace whereas the second command lets ROS know about the new compiled files. Moreover, the ```catkin_make``` command should also create two additional folders in the workspace, ```build``` and ```devel```. The ```build``` folder is the default location of the build space and is where cmake and make are called to configure and build your packages. The ```devel``` folder is the default location of the devel space, which is where your executables and libraries go before you install your packages. 
    
1. Congradulations, you successfully completed all the steps required to setup the Race On ROS environment. But before proceeding with the next section reboot to apply the updates.
    ```bash
    sudo reboot
    ```
    
# ROS Tutorial
Before we introduce the Race On code for ROS we will do a quick tutorial. In ROS, code is organized around packages which are folders with special structure inside the  ```src``` folder of the workspace. If you run ```ls ~/race-on-ros/src``` you will see that the ```race-on-ros``` workspace contains only the ```raceon``` package. In this tutorial we will create a new package called ```tutorial``` that will have two nodes, the ```publisher``` node that publishes a sequence of numbers to the ```data``` topic and the ```subscriber``` node that will subscribe to the ```data``` topic to receive the sequence.

1. To create a new package we have first to make ```~/race-on-ros/src``` the current directory and then run the ROS ```catkin_create_pkg``` to create a package folder.
    
    ```bash
    cd ~/race-on-ros/src
    catkin_create_pkg tutorial std_msgs rospy
    ```
    The arguments for the ```catkin_create_pkg``` command are the package name followed by the dependencies of the new package. In this case, the dependencies are ```std_msgs``` and ```rospy```. Since we will send ```Float32``` numbers
we will use ```std_msgs``` as it includes common message types representing primitive data types and other basic message constructs. And ```rospy``` since we will write our code in Python. If you plan to write code in C or C++ you must also add ```roscpp``` to the dependence list. In case you forget to add a package as dependence, you can do that later by editing one of the package configuration files. 

    To check the result of the command we can list the contents of the ```tutorial``` folder using ```ls -ahl tutorial/``` and you should see an output similar to this:
    ```bash
    total 24K
    drwxr-xr-x 3 pi pi 4.0K Feb 15 03:06 .
    drwxr-xr-x 4 pi pi 4.0K Feb 15 03:06 ..
    -rw-r--r-- 1 pi pi 6.9K Feb 15 03:06 CMakeLists.txt
    -rw-r--r-- 1 pi pi 2.7K Feb 15 03:06 package.xml
    drwxr-xr-x 2 pi pi 4.0K Feb 15 03:06 src
    ```
    Where ```package.xml``` contains the package configuration in XML format, ```CMakeLists.txt``` contains the instructions how to build the C++ code, and the ```src``` is the directory where you place all your source files. To change the dependency list of your package at a later time or costumize the package edit the ```package.xml``` file.
    
    To build the package run 
    ```bash
    cd ~/race-on-ros/
    catkin_make
    ```
    Since we did not write any code this will should not encounter any errors, just see that ```catkin```, the build tool of ROS, recognized the new package and traversed it contents.
    
    

# Setting Up ROS for Your Car
## Step by Step Instructions
1. First, install OpenCV for Python3.

    ```
    sudo apt install python3-opencv
    ```

    If you run into issues installing OpenCV, especially with problems fetching packages from the Raspbian reposiories, try ```sudo apt update``` to get updated information about packages.

2. Next, set the ROS_PYTHON_PATH in your ```.bashrc```.
    ``` 
    echo "export ROS_PYTHON_VERSION=3" >> ~/.bashrc
    ```

3. Open the file ```~/startup/ros.sh``` and, if present, remove the line ```export ROS_HOSTNAME=raspberrypi"```.

4. Now, ```sudo reboot```

5. If the directory ```race-on-ros``` is present, delete it using ```rm -rf ~/race-on-ros```.
6. Clone the race-on repository again, using 
    ```git clone https://github.com/race-on/race-on-ros.git```
7. Cd into the race-on-ros repository and build the workspace.
 ```
 cd ~/race-on-ros
 catkin_make
 ```
8. To run the ros nodes, run ```roslaunch raceon raceon.launch speed:=140```.
