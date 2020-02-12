title: Ros Setup

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