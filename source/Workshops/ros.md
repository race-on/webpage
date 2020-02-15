title: 1. Introduction to ROS

# Introduction to ROS

[ROS](https://wiki.ros.org/ROS/Introduction) is an open-source framework that aims to simplify the complexities involved in writing robust, general robotics software. ROS has a number of important filesystem level concepts, including packages, metapackages, package manifests, repositories, message types and service types. For Race On, however, we will focus mostly on the use of **packages** and **message types**. 
The power of ROS comes from the use of a computation graph similar to one used by Tensorflow, a machine learning library. The computation graph is composed from a number of nodes which either produce or consume data from multiple sources through a publish/subscribe architecture. At initialization, nodes declaretheir name and subscribe to the **topics** which provide all the necessary information for the node to properly function. Each **topic** has an associated **message type** which defines the format that the publishers and subscribers will use to exchange information. Some nodes might only publish data, such as the camera node, some might only subscribe to data topics, such as the motor and servo node, and some might do both such as the image processing node that receives images and outputs the car track position.

ROS concepts review.

* **Packages** - In ROS, packages are the primary means of organizing software and are the smallest item you can build and release. They can contain any collection of files that should logically be contained together.
* **Message Types** - Message types describe message data structures and outline which data types they contain. When creating a message type, they should be contained in the ```msg`` folder of your package. For more info about defining a custom message type in ROS, check out [creating a ROS msg and srv](https://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv).
* **Nodes** - Nodes are modular processes that perform some computation for your system. They should be a logical unit within your sytem. For example, for Race On, we are using 4 nodes (actuation.py, camera.py, control.py and pos_estimation.py), all of which handle a specific part of the car's behavior. Each of these nodes communicate by publishing and subscribing to messages on different **topics**. 
* **Topics** - Nodes can send out and receive messages by publishing and subscribing to **topics**. Each topic has a name, which should be associated with the message types or message content being sent through it. For example, the control node in the race-on-ros repository subscribes to Pose messages on the ```position/error``` topic and publishes AckermanDrive messages on the ```control``` topic. Note that you can only send one message type through a single topic. 
* **Messages** - Messages are simple data structures that can contain a combination of basic data types. 
For a more in depth discussion of common ROS concepts, check out [ROS concepts](https://wiki.ros.org/ROS/Concepts).

## Preparation

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
    
1. Congratulations, you successfully completed all the steps required to setup the Race On ROS environment. But before proceeding with the next section reboot to apply the updates.
    ```bash
    sudo reboot
    ```

# ROS Tutorial
Before we introduce the Race On code for ROS we will do a quick tutorial. In ROS, code is organized around [packages](http://wiki.ros.org/Packages) which are folders with special structure inside the  ```src``` folder of the workspace. If you run ```ls ~/race-on-ros/src``` you will see that the ```race-on-ros``` workspace contains only the ```raceon``` package. In this tutorial we will create a new package called ```tutorial``` that will have two nodes, the ```publisher``` node that publishes a sequence of numbers to the ```data``` topic and the ```subscriber``` node that will subscribe to the ```data``` topic to receive the sequence. [If you're looking for more, here's another tutorial which provides information on creating a package in a catkin workspace](http://wiki.ros.org/ROS/Tutorials/CreatingPackage).

1. To create a new package we have first to make ```~/race-on-ros/src``` the current directory and then run the ROS ```catkin_create_pkg``` to create a package folder.
    
    ```bash
    cd ~/race-on-ros/src
    catkin_create_pkg tutorial std_msgs rospy
    ```
    The arguments for the ```catkin_create_pkg``` command are the package name followed by the dependencies of the new package. In this case, the dependencies are ```std_msgs``` and ```rospy```. Since we will send ```Float32``` numbers
we will use ```std_msgs``` as it includes common message types representing primitive data types and other basic message constructs. And ```rospy``` since we will write our code in Python. If you plan to write code in C or C++ you must also add ```roscpp``` to the dependency list. In case you forget to add a package as dependence, you can do that later by editing one of the package configuration files. 

    To check the result of the command we can list the contents of the ```tutorial``` folder using ```ls -ahl tutorial/``` and you should see an output similar to this:
    ```bash
    total 24K
    drwxr-xr-x 3 pi pi 4.0K Feb 15 03:06 .
    drwxr-xr-x 4 pi pi 4.0K Feb 15 03:06 ..
    -rw-r--r-- 1 pi pi 6.9K Feb 15 03:06 CMakeLists.txt
    -rw-r--r-- 1 pi pi 2.7K Feb 15 03:06 package.xml
    drwxr-xr-x 2 pi pi 4.0K Feb 15 03:06 src
    ```
    Where ```package.xml``` contains the package configuration in XML format, ```CMakeLists.txt``` contains the instructions how to build the C++ code, and the ```src``` is the directory where you place all your source files. To change the dependency list of your package at a later time or customize the package, edit the ```package.xml``` file. However, since our code will be only written in Python, we will create a separate folder called ```scripts``` to place our Python programs using the ```mkdir tutorial/scripts``` command.
    
    To build the package run 
    ```bash
    cd ~/race-on-ros/
    catkin_make
    ```
    Since we did not write any code this will should not encounter any errors, just see that ```catkin```, the build tool of ROS, recognized the new package and traversed it contents.
    
1. Create the publisher node. In ROS, a "node" is an executable that is connected to the ROS network. Here we will create the publisher node which will continuously send samples generated using a sinusoidal signal. Using the Jupyter interface, navigate to the ```race-on-ros/src/tutorial/scripts/``` folder and create a new text file called ```publisher.py```. Paste this content inside the file and save it.
    ```python
    #!/usr/bin/env python

    import rospy
    from std_msgs.msg import Float32

    import math

    RATE = 10  # Publishing rate of new data per second
    FREQ = 1   # Frequency of the sinusoidal signal


    # Execute this when run as a script
    if __name__ == '__main__':

        rospy.init_node('publisher')

        pub  = rospy.Publisher('data', Float32, queue_size=1)
        rate = rospy.Rate(RATE)
        step = 0

        while not rospy.is_shutdown():

            # Generate new data point
            value = math.sin(2*math.pi*step/RATE)

            # Log and publish data
            rospy.loginfo("Publishing {:.3f}".format(value))
            pub.publish(value)

            # Advance the sequence
            step = step + 1

            # Wait to match the rate
            rate.sleep()
    ```
    The break down. 
    ```
    #!/usr/bin/env python
    ```
    This tells the terminal that this is a Python script. All Python files should have it.
    ```python
    import rospy
    from std_msgs.msg import Float32

    import math
    ```
    We import the Python libraries which we declared as dependencies for the tutorial package and the math library required for the to calculate the sinusoid values. The ```python rospy``` library provides all the ROS functionality to Python whereas ```python std_msgs``` library the standard message types in ROS. 
    ```python
    RATE = 10  # Publishing rate of new data per second
    FREQ = 1   # Frequency of the sinusoidal signal
    ```
    Next, we declare two global constants which are parameters of the generated sequence. 
    The code inside the if statement represents the main program. Here we define how the publisher node interfaces with the rest of ROS. The line ```rospy.init_node(NAME)```, is very important as it tells rospy the name of the node -- until rospy has this information, it cannot start communicating with the [ROS Master](http://wiki.ros.org/Master). In this case, the node will take on the name ```publisher```. NOTE: the name must be a base name, i.e. it cannot contain any slashes "/". The ```rospy.Publisher('data', Float32, queue_size=1)``` declares that your node is publishing to the ```data``` topic using the message type ```Float32```. Float32 here is actually the class std_msgs.msg.Float32. The ```queue_size``` argument limits the amount of queued messages if any subscriber is not receiving them fast enough. ```rate = rospy.Rate(RATE)``` this line creates a Rate object rate. With the help of its method sleep(), it offers a convenient way for looping at the desired rate. With its argument of 10, we should expect to go through the loop 10 times per second (as long as our processing time does not exceed 1/10th of a second!). Next we have a fairly standard rospy loop construct: checking the ```rospy.is_shutdown()``` flag and then doing work. You have to check ```is_shutdown()``` to check if your program should exit (e.g. if there is a ```Ctrl-C``` or otherwise). In this case, the "work" is a call to ```pub.publish(value)``` that publishes a string to our ```data``` topic. The loop calls ```rate.sleep()```, which sleeps just long enough to maintain the desired rate through the loop. This loop also calls ```rospy.loginfo(str)```, which performs triple-duty: the messages get printed to screen, it gets written to the Node's log file, and it gets written to ```rosout```. ```rosout``` is a handy tool for debugging: you can pull up messages using ```rqt_console``` instead of having to find the console window with your Node's output. 
    
    To run the code we need first to make the script executable by changing the file permission using ```chmod +x ~/race-on-ros/src/tutorial/scripts/publisher.py``` command. Now, we can run the code using ```rosrun tutorial publisher.py``` command. Note, tab autocompletion works with ROS command arguments.

1. The code for the subscriber is similar and the steps are the same.
    ```python
    #!/usr/bin/env python

    import rospy
    from std_msgs.msg import Float32


    # Automatically called by ROS when new message is received
    def callback(message):
        rospy.loginfo("Received {:.3f}".format(message.data))


    # Execute this when run as a script
    if __name__ == '__main__':

        # Init the node and subscribe for data
        rospy.init_node("subscriber")
        rospy.Subscriber("data", Float32, callback)


        # Prevents python from exiting until this node is stopped
        rospy.spin()
    ```
    Now you can open a second terminal to run the subscriber. Remember to make the subscriber script executable.
    
1. **Exerices**
    To better understand ROS concepts try to improve the above code by adding the following features:
    1. Create a launch file to start both nodes at the same time. Consult ROS documentation and raceon package code for inspiration.
    1. Replace the global constants in the ```publisher.py``` file with ROS parameters. Update the launch file to include the default parameter values. Find how to overwrite the parameter value when you launch the application.
    1. Create a custom message type that contains two float number. Set the value of the second float to be the cosine. Use rqt ROS tool to plot in real-time these values on your machine.


## Overview of Race On ROS Code
The ROS repository for Race On consists of 4 ROS nodes and one Python class that initializes the car configuration (including the servo's middle and rightmost values and the motor's min, max and brake speeds) and provides functions for steering and changing the speed of the car. The 4 nodes included in the repository are actuation.py, camera.py, control.py and pos_estimation.py. The nodes should be run using the file ```race-on-ros/src/launch/raceon.launch```. 

Read more below to learn more about each of the nodes and the launch file we've provided to run them.

### Launch File 
The launch file has a number of parameters for each of the nodes, which you can customize according to your needs. It also defines the topics to which the nodes publish and subscribe. For example, the camera node publishes to either ```camera/image``` or ```camera/image/compressed``` topic, depending on the value of the ```~publish_raw``` parameter, which is defined below on line 13 of the launch file.

#### Exercise
Read through the launch file to get a better understanding of how it works. Change or remove the parameters defined in it to see how the nodes respond.

### Camera Node

|   Topics                        | Message Type      |  Action       |
| --------------------------------| ------------------| ------------  |
| ```camera/image```              |   Image           |  Publish      |
|   ```camera/image/compressed``` |   CompressedImage |  Publish      | 

The camera node, which is found in ```race-on-ros/src/scripts/camera.py``` starts up the pi's camera and begins recording upon startup. Depending on the value of the ```~use_compressed_image``` parameter, it will either publish compressed images on the ```camera/image/compressed``` or uncompressed images on the ```camera/image``` topic. The **resolution**, **frames per second** and **publish_raw** parameters can be configured in the launch file.

#### Suggested Exercise
Try adjusting the frame rate from the launch file and see how it affects your car's trajectory.
#### Suggested Exercise
Try adjusting the frame rate from the launch file and see how it affects your car's trajectory!

### Position Estimation Node

|   Topics                        | Message Type      |  Action         |
| --------------------------------| ------------------| ------------    |
| ```camera/image```              |   Image           |  Subscribe      |
|   ```camera/image/compressed``` |   CompressedImage |  Subscribe      | 
|   ```position/error```          |   Pose            |  Publish        | 
|   ```position/track``           |   TrackPosition   |  Publish        | 

The position estimation node is found in ```race-on-ros/src/scripts/pos_estimation.py```. This node executes much of the position-estimation functionality you likely saw before in [the testcar-nomiddleline jupyter notebook](https://github.com/race-on/workshops/blob/master/testcar-nomiddleline.ipynb). Depending on the value of the **use_compressed_image** parameter, the camera node will either subscribe to the ```camera/image/compressed``` or ```camera/image``` topic. 

You can set the **scan line**, **peak threshold**, **track width** and **camera center** parameters in the launch file or adjust their default values in the node's code. The **scan line** parameter is the horizontal line in the image that you'll use to look for peaks. 

When the node receives an image message, it finds peaks along the horizontal scan line and uses them to estimate the car's position within the track. It then **publishes this position estimation to the "position/error" topic**. It also **publishes an estimation of the track position to the "position/track" topic**.

#### Suggested Exercise
Try changing the peak threshold and see how it affects your estimation of the track and car positions!

### Controller Node

|   Topics                        | Message Type      |  Action         |
| --------------------------------| ------------------| ------------    |
| ```position/error```            |   Pose            |  Subscribe      |
|   ```control```                 |   AckermanDrive   |  Publish        |

The controller node is found in ```race-on-ros/src/scripts/controller.py```. The control node is where you'll want to put your team's control logic (potentially a PID controller?). Currently, the control node uses a proportional gain coefficient to determine how much you should alter the servo's position to minimize the distance from the target position. 

You can change the **motor speed**, **target** (target x position on the track) and **kp** (proportional gain coefficient to use to scale the error) parameters in the launch file or alter their default values in the code.

When the controller node receives a Pose message containing the car's current position on the track, it calculates the error from the target position and estimates a new servo position value using **kp** to steer the car closer to the target. It then **publishes this new servo position and the current motor speed to the control topic**.

#### Suggested Exercises
Adjust the motor speed and proportional gain parameters and see how they alter your car's behavior.

### Actuator Node
|   Topics                        | Message Type      |  Action         |
| --------------------------------| ------------------| ------------    |
| ```control```                   |   AckermanDrive   |  Subscribe      |

The actuator node is found in ```race-on-ros/src/scripts/actuation.py```. It **subscribes to the ```control``` topic and is responsible for commanding the car**. To do so, it makes use of the car module, which has utility functions to set the car's speed and servo position.

When the actuator node receives a control message, it uses the car module functions to steer the car and set its speed.

### Car Module
The car module is a class that initializes the servo and motor and provides functions to alter the car's speed and the servo's position and also to brake the car.

## Setting Up ROS for Your Car
To run the ros nodes, run ```roslaunch raceon raceon.launch speed:=140```.
