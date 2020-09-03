title: Projects

# Race On 2020 Projects

## Project Deliverables
Each project must include the following deliverables:

1. Demo Video – a short video showing the project in action along with a brief description and the result for different values of the configuration parameters. Bio of the team members who worked on the project.
2. An article/blog post – brief description of the technologies used and they connect together, detailed description of the project result and source code, examples how to use and description of the most important parameters and how to select them. Instructions, how to add to existing work, detailed performance analysis if applicable, future steps or direction that might yield better performance. Bio of the team members who worked on the project and their contact information.
3. Any additional requirements stated in the project description

## High Priority Projects

### **Pose estimation using visual tags** 
**Topic**: *Perception* **Priority**: *High* **Difficulty**: *Low*

<iframe width="560" height="315" src="https://www.youtube.com/embed/Y8WEGGbLWlA" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

Detecting the track ahead of the car is important for local steering decisions but might compromise the overall performance on the track. For example, when detecting a turn without knowing the map, the driving algorithm must assume the worst case scenario which could result in a significant decrease in speed. However, by knowing the map, the best speed for each turn can be used to  drastically improve the performance. For that, special tags such as **Apriltags 3** can be placed on the track. Upon detection of these tags,  the car position, orientation and speed can be inferred. The goal of this project is to evaluate **Apriltags 3** and other tags in terms of reliability of detection and required real-time CPU usage. Moreover, different placement strategies for those tags can also be evaluated.

References:

1. [AprilTags Github](https://github.com/AprilRobotics/apriltag)
2. [Alvar tags ROS page](https://wiki.ros.org/ar_track_alvar)
3. [Chilitags Github](https://github.com/chili-epfl/chilitags)
4. [AprilTags ROS package](http://wiki.ros.org/apriltags_ros)

Additional deliverables:

1. Tags to be used and their placement strategy


### **Localization using visual tags**
**Topic**: *Perception* **Priority**: *High* **Difficulty**: *Medium*

<iframe width="560" height="315" src="https://www.youtube.com/embed/Dty79dyhGEQ" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

Real cars have the advantage of using GPS for localization, but this is not the case for Race On cars. Knowing the exact position of the car on the track offers a big advantage as it allows a more precise path planning which heavily outperforms the stay-in-the-middle strategy. By placing two or three **Apriltags 3** or other similar tags on each segment, an accurate track map can be constructed and used for precise localization of the car. Moreover, this localization can be also used in estimating the car model parameters of the state-space controller.

References:

1. [Mobile Robot Localization, Chapter 7](https://raw.githubusercontent.com/yvonshong/Probabilistic-Robotics/master/Probabilistic-Robotics-en.pdf)
1. [Different localization algorithms](https://pythonrobotics.readthedocs.io/en/latest/modules/localization.html)
1. [Some insight on choosing AprilTags for localization](http://wiki.lofarolabs.com/index.php/Localizing_with_AprilTags)
1. [Similar project source code (in chinese)](https://github.com/lifuguan/AprilTag_Localization)


### **Frequency domain model estimation and control**
**Topic**: *Control* **Priority**: *High* **Difficulty**: *Medium*

The main goal of this project is to gain a deeper understanding of the system’s frequency response and apply it to design controllers such as PID and lead-lag. This will provide teams more formal tools to tune their vehicle gains.

We assume the vehicle lateral dynamics can be modelled as a second order system. The first step is to estimate its parameters (damping ratio and natural frequency), which will most likely depend on the longitudinal velocity. Then, the controller gains can be determined based on current velocity and desired system response.

Additional deliverables

1. Brief introduction frequency domain models, why they are useful for Race On and mathematical definition of the model being used 
2. Standard procedure to estimate model parameters
3. Sample procedure to determine system gains based on analytical and graphical control tools
4. Average lap times of different controllers

### **Time domain model estimation and control**

**Topic**: *Control* **Priority**: *High* **Difficulty**: *High*

This project has two main goals. The first is to develop a reliable procedure to estimate vehicle parameters necessary for state-space control approaches. The second is to use the state-space model for the design of a state-feedback control, which might also include a feedforward term.
We start by assuming the vehicle can be modelled by a two-wheel bicycle model. Next, we will  provide a standard procedure that any participant can follow to obtain all the physical parameters of the model. This phase includes some literature research and creativity to adapt the methods to the Race On environment. Then, a state-observer is necessary to estimate in real-time the system states, which will be used by the controller. The last step is to use the estimated states in a state-feedback controller. If time allows, this project can go on to more advanced observer/ controller design such as Kalman filtering, LQR and MPC.

Additional deliverables:

1. Brief introduction to time domain models, state-space notation, why they are useful for Race On and mathematical definition of the model being used
2. Standard procedure to estimate model parameters
Blog-post with brief introduction to observers and how to apply it to the Race On vehicle
3. Description of controller, including state-feedback and possible feedforward term. Sample procedure on defining gains
4. Average lap times of different controllers


### **State-space controller**
**Topic**: *Control* **Priority**: *High* **Difficulty**: *High*

<iframe width="560" height="315" src="https://www.youtube.com/embed/hnvj4rgrAy0" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

The main goal of this project is to implement an advanced controller design by utilizing the car dynamical model so as to achieve a better racing performance. This project can be subdivided into the car model estimation and control and state observer design. First, the car model must be estimated either using parametric (first principles model) and/or non-parametric (data-driven) methods. Second, a state-observer design is necessary to estimate the hidden state dynamics used for the higher level control. A typical implementation is the Extended Kalman Filter. Finally, given the model and observer designs, advanced control techniques (e.g. LQR, MPC, etc.) will be implemented and compared with the typical PID implementation. Comparisons in performance will be evaluated. 

References:

1. [Paper on LQR controller design for drifting an 1/10 RC car](https://borrelli.me.berkeley.edu/pdfpub/auto_drift.pdf)
1. [Thesis on LQR controller design for RC car](https://www.diva-portal.org/smash/get/diva2:1149520/FULLTEXT01.pdf)
1. [MPC controller paper](https://arxiv.org/pdf/1610.06534.pdf)
1. [MPC controller website](https://automatedcars.space/home/2017/4/18/firkxklwjl37emejf1eeevw7a6ulvh)
1. [Kalman filter python library](https://github.com/rlabbe/filterpy) + [book explaining the library](https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/)
1. [Reference controller implementation on a 1/10 car (BARC source code)](https://github.com/MPC-Berkeley/barc/tree/master/workspace/src/barc/src)


### **Simulation and Development Docker images**
**Topic**: *Simulation* **Priority**: *High* **Difficulty**: *Low*

In order to ensure a smooth collaboration, the same development environment must be used by all parties. The simplest way to achieve this is to use Docker images (lightweight virtual machines). The goal is to create two such images, one for simulation and one for development. The simulation image will be used by GitHub to run the code at each commit and check it’s output. The image contains all the necessary tools to simulate a Race On environment based on Gazebo and ROS. The development image is based on the simulation image but in addition to that contains tools to develop code. This image will be used by all technical leaders and specialists when working on the project. This approach will facilitate integrating each team’s work into the Race On project.

Additional deliverables:

1. Docker files for the two images


### **Reliable Camera Calibration and Bird’s Eye view image mapping**
**Topic**: *Perception* **Priority**: *High* **Difficulty**: *Low*


<iframe width="560" height="315" src="https://www.youtube.com/embed/yAYqt3RpT6c" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

Wide angle cameras introduce distortions that increase as we go farther from the camera center. These distortions make straight lines look curved. This is a big problem and introduces errors when performing visual odometry or visual localization which rely on a calibrated camera. Therefore, camera calibration must be the first step for any visual algorithm. The goal of this project is to design a calibration procedure that is easy to implement and can be used by teams at any time if they suspect that the camera is not calibrated.

References:

1. [Camera calibration explanation + code](https://nikolasent.github.io/opencv/2017/05/05/Camera-calibration-with-OpenCV.html)
1. [Bird’s eye view explanation + code](https://nikolasent.github.io/opencv/2017/05/07/Bird's-Eye-View-Transformation.html)
1. [Rolling Shutter Calibration + IMU based on camera + papers](https://github.com/ethz-asl/kalibr)


## Medium Priority Projects

### **Visual Odometry**
**Topic**: *Perception* **Priority**: *Medium* **Difficulty**: *Medium*

<iframe width="560" height="315" src="https://www.youtube.com/embed/homos4vd_Zs" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

Visual odometry is the problem of estimating the trajectory of the camera only from its image feed. In more details, given two images we want to estimate the translation over x, y, z and the rotation 𝜃x, 𝜃y, 𝜃z that if applied will transform the first image into the second one. This is a much simpler problem than localization, although the two are connected with the difference that in localization the position is absolute whereas in odometry the position is relative.

References:

1. [Tutorial](http://avisingh599.github.io/vision/monocular-vo/)
1. [An overview of image based localization](https://arxiv.org/pdf/1610.03660)
1. [Similar project](https://github.com/SonamYeshe/ENPM673-Perception-for-Autonomous-Robots/tree/master/2.0_VisualOdometry)


### **Rapidly Exploring Random Trees for Path Planning**
**Topic**: *Planning* **Priority**: *Medium* **Difficulty**: *Low*

<iframe width="560" height="315" src="https://www.youtube.com/embed/SZ9c_3HUVUE" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

Rapidly Exploring Random Trees is an algorithm to search for an optimal trajectory on a constrained map (keeping the car inside the track boundaries). The random part for generating trajectories can be biased towards a trajectory with desirable characteristics (higher car speed and margin of error). The goal of the project is to implement RRT and RRT* or other variants that receive a track configuration and output an optimal trajectory for the car to complete a lap. The algorithm must be configurable to allow a trade off between car speed and margin of error. As a bonus task, one could integrate the car dynamics model into the RRT algorithm.

References:

1. [Explanation](https://medium.com/@theclassytim/robotic-path-planning-rrt-and-rrt-212319121378)
1. [Source code + other similar algorithms](https://pythonrobotics.readthedocs.io/en/latest/modules/path_planning.html)
1. [Vehicle Path Planning paper](https://www.cs.cmu.edu/~motionplanning/reading/PlanningforDynamicVeh-1.pdf)


### **Integration with Webviz**
**Topic**: *Simulation* **Priority**: *Medium* **Difficulty**: *Low*

Webviz is a web-based application that can listen to ROS messages and visualize them in real-time or playback from a rosbag. The goal is to integrate webviz with the Race On starter code and explore different visualization templates and propose a better solution given the Race On problem. In addition, webviz can publish ROS messages thus enabling the design of a dashboard for car control.

References:

1. [Webviz Github page](https://github.com/cruise-automation/webviz)

Additional deliverables:

1. Dashboard configuration


## Low Priority Projects
### **Port existing code to ROS 2 and Real-Time Linux**

**Topic**: *Other* **Priority**: *Low* **Difficulty**: *High*

Currently, Race On starter code uses ROS 1 and heavily relies on Python 2.7 which as of 2020 is obsolete. ROS 2 and Python 3 is the way to go in the long run. The goal is to port the existing code to ROS 2 and Python 3. Moreover, Linux kernel was not designed for Real-Time applications but there exists a [patch](https://lemariva.com/blog/2019/09/raspberry-pi-4b-preempt-rt-kernel-419y-performance-test) that fixes this. This kernel makes the Raspberry Pi slower by at most 10% but instead guarantees that parts of the program will be executed on time if configured so. The goal is to patch a Pi image and provide instruction on how to set up the ROS code to benefit from the real-time kernel.


## Additional Reference Materials

- https://f1tenth.org/learn.html
- https://github.com/SonamYeshe/ENPM673-Perception-for-Autonomous-Robots 
- [Various Robotics Algorithms implemented in Python](https://pythonrobotics.readthedocs.io/en/latest/getting_started.html)
