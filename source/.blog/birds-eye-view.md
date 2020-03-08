title: Bird's Eye View – Look from Above
---

# Bird's Eye View – Look from Above

In this short blog post we will use OpenCV to transform our camera image into one similar to the sattelite view from your maps application. Since our camera has a wide viewing angle lens, we will need also to compensate for the lens distortion. In the end, we will be able to transform the image on the left into the one on the right. Note, this transformation does not add any new information to the image, however, it makes easier for us to program more complex behaviours for specific track configurations.

![initial image](link) ![final image](link)

## Prerequisite

Since this transformation is done using functions from the OpenCV library we need to install it on our Raspeberry Pi. For that, run the next two command and confirm when asked. If you already have the Python library for OpenCV installed, skip this step.

    ```bash
    sudo apt update
    sudo apt install python3-opencv
    ```

To test the installation, open a terminal, type ```python3```, and when the Python prompt appears, ```>>>```, import the OpenCV library by typing ```import cv2```. The library should load without any errors. 

## Perspective transformation

Two types of transformatiom, shrinking and widening.

Code for shrinking and explanation

Lines are not straight

## Camera calibration

What we need: checker board and images with the board in different locations. Link to matlab for more info about the fish eye effect.

Calibration output, and why saving map files only

Full view, decide how much to crop

## Perspective transformation with calibrated camera

Update coordinates, fast code, estimated time

## Jupyter notebook

## References
