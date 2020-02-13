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

3. Open the file ```~/startup/ros.h``` and, if present, remove the line ```export ROS_HOSTNAME=raspberrypi"```.

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