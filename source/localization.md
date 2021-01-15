# Real-Time Localization using AprilTags on a Raspberry Pi
Date: September 4, 2020 <br>
Author: Theodore Lewitt <br>

## Overview
In this workshop, we will use a wide angle camera to estimate your car's position on the track. This information can help your car drive closer to the edge of the track and pick a faster line.
This workshop will teach you computer vision fundamentals like camera calibration, rotation and translation transformations, and image processing using OpenCV and a Raspberry Pi camera. There are additional oppurtunities to learn more about camera hardware fundamentals or control theory algorithms like Kalman filtering.

## Final Product
[![Thumbnail](https://img.youtube.com/vi/7iOmVOEvfgw/0.jpg)](https://www.youtube.com/watch?v=7iOmVOEvfgw)

## System Requirements
For each frame the camera sends to the Raspberry Pi, we need to process the data quickly enough to run in real time. For more concrete requirements, we can look at the servo motor on the car which can only update at 50 Hz. So we don't need anything faster than 50 FPS but will need greater than 20 FPS otherwise the car can travel too far in between updates.

The other requirement is high localization accuracy so our car never is disqualified for going off the track. The white boundaries of the Race On track are 2 inches thick, so we ideally want accuracy up to 2 inches so our car can hug the track around the turns for the fastest time. However, most teams take a more conservative approach and we will require less than 5 inch error.


- FPS: 20 - 50, ideally >40
- Max Localization Error: 5 inches (12.7 cm)

## What You'll Need
- Printed AprilTags, download [here] (https://github.com/AprilRobotics/apriltag-imgs/tree/master/tagStandard41h12), resize to 6 inches by 6 inches using software like [GIMP]( https://www.gimp.org/downloads/), and print!
- Printed Checkerboard for camera calibration, download [here](https://markhedleyjones.com/storage/checkerboards/Checkerboard-A4-30mm-8x6.pdf) and print!
- [Raspberry Pi Wide Angle Camera Module](https://www.seeedstudio.com/Raspberry-Pi-Wide-Angle-Camera-Module.html) 

## Set-Up
- Install OpenCV on the Pi following this [guide](https://www.pyimagesearch.com/2019/09/16/install-opencv-4-on-raspberry-pi-4-and-raspbian-buster/)
- Clone this [repo](https://github.com/teddylew12/race_on_cv) and install other dependencies using ```pip install -r requirements.txt```
- Measure the size of your AprilTags in meters. ADD PHOTO HERE

## Camera Calibration
Before we start using the AprilTags for localization, we need some more information about the camera. The Wide Angle Camera Module is a fisheye camera with 160 degrees of view, compared to 80–90 degrees on a normal camera. The trade-off with an fisheye camera is straight lines become curves near the edges of the image. ADD PHOTO

We preform camera calibration to understand where a point in 3D space maps to the 2D pixel coordinates in the image. The output of the camera calibration is  the intrinsic matrix *K* and the distortion array *D*. To learn some of the theory behind camera calibration and what *K* means, check out this [blog](http://ksimek.github.io/2013/08/13/intrinsic/). 
OpenCV has the fisheye calibration module, which finds both *K* and *D* from a set of images of a checkerboard. Use the *fisheye_calibration.py* script to obtain *K* and *D*.

## Scene Setup
This is the final step before diving into the actual code, we need to place the printed AprilTags on a flat surface (wall, floor, ceiling,…).

1. Determine a global coordinate frame origin, ideally the bottom left corner of the room as it will make the following steps easier and keep all the numbers positive. From this point, we will use positive X to the right, positive Y going up and positive Z going out of the wall towards you. <br>
2. Place AprilTags around the room, in pairs of 2 or 3. The detections start failing from farther than 3 meters. Make sure the tag is not rotated or upside down, using a visualization script like [visualize_frame.py](https://github.com/teddylew12/race_on_cv/blob/master/visualize_frame.py). <br>
3. Measure the position of the tags, in inches from your global origin.<br>
4. Determine the orientation of the tags in reference to your global coordinate system in terms of Euler angles. Euler angles are a way to describe rotations in 3D and consist of 3 angles; X, Y and Z. To read more about Euler angles, I'd recommend this [article](http://www.chrobotics.com/library/understanding-euler-angles). If your tag is on a wall, the only angle that will change is the Y angle. If your tag is on a floor or ceiling, the only angle that will change is the X angle. <br>
5. Save all of these measurements using the add_tag() function from [tag.py](https://github.com/teddylew12/race_on_cv/blob/master/tag.py), which takes the tag id, coordinates (in inches) and Euler angles (in radians). This function will automatically convert inches to meters for you.

## Getting Images from the Raspberry Pi
The PiCamera module provides many ways to capture frames from the camera, but many don't provide enough FPS for our use case.  For example, the [capture_continuous](https://picamera.readthedocs.io/en/release-1.13/api_camera.html#picamera.PiCamera.capture_continuous) function allows us to rapidly capture individual frames, but is limited to 20–30 FPS. Instead we record a video, and write to a custom class that can get up to 90 FPS. <br>
Our custom class, called Stream, is where all of the image processing and AprilTag detection takes place. This happens inside the main function write(), which receives raw YUV camera data from camera.start_recording() function.
```
#Inside real_time.py
with PiCamera() as camera:
  stream = Stream()
  try:
    # Start recording frames to the stream object
    camera.start_recording(stream, format='yuv')
    t0 = time()

    while True:
      camera.wait_recording()
      # If the time limit is reached, end the recording
      if (time() - t0) > MAX_TIME:
          camera.stop_recording()
          break
```

## Processing the Images
The majority of the processing is within the *write()* function of the Stream class.  There are 5 main steps that take an raw camera image to an measurement of the camera's location. As on 1/13/21, here are the major parts of the algorithm and time it takes ADD TIMES
1. Raw Data to OpenCV Image 
2. Undisortion
3. AprilTag Detection
4. Pose Estimation
5. Filtering

### Raw Data to OpenCV Image
AprilTags detect on grayscale images, so instead of capturing a RGB image and converting to grayscale, we capture a YUV image. An YUV image is composed of three channels: luminance(Y), chroma blue (U), chroma red (V). The Y channel corresponds to a grayscale image, so when the camera writes to the stream, we only need to read in the first channel of the image to get the grayscale. OpenCV represents it's images as numpy arrays in Python. We use np.frombuffer() to read the bytes into a numpy array.
```
# Within stream.py, data is the raw bytes from the camera
res = (640,480)
I_raw = np.frombuffer(data, dtype=np.uint8, count=res[1]*res[0])
I_raw = I_raw.reshape(res[1], res[0])
```
### Undistortion
Once we have the raw image, we need to apply un-distortion to our image to reverse all of the fisheye distortion before detecting AprilTags. Un-distortion has two steps, one that can be pre-calculated and one that needs to be performed on each frame. <br>
The pre-calculated step is computing a rectification and un-distortion matrices using cv2.fisheye.initUndistortRectifyMap(). These matrices are inputted into the cv2.remap() function, which needs to be called on each frame and outputs the corrected image. <br>

```
### In real_time.py
# K and D are camera intrinsics from the Camera Calibration
# Res is the resolution of the image
map_1, map_2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, res, cv2.CV_16SC2)

### In stream.py
I = cv2.remap(I_raw, map_1, map_2)
```

### AprilTag Detection
To use the python AprilTags bindings from this repo, you first create a Detector object. There are a bunch of different arguments for creating a Detector, and I'll highlight the most important ones.
- Families: The families of AprilTags it should look for, we use tagStandard41h12.
- nthreads: Number of threads to use for detection, using more will speed up detections.
- quad-decimate: Uses a lower resolution image for detection, improving speed but reducing accuracy/pose estimation. Defaults to 1.0 but I'd recommend a value between 2.0 and 4.0 for optimal tradeoff.
- debug: Defaults to 0 but if set to 1 will save images from each step in the detection process. Very computationally expensive, should only be used on single frames.

To use the Detector, we call the detect() function on a image. For pose estimation, we set estimate_tag_pose to True and supply the tag size and four elements from the camera's K matrix. 

The output of the detect() functions is a list of Detection objects. Each Detection contains a lot of information about the tag it detected and I'll highlight the important ones.
- Tag ID: The ID of the Tag
- Corners: A list of 4 tuples containing the pixel coordinates of the corners. These wrap counter clockwise from the bottom left corner.
- $Pose_R$: A 3x3 Rotation matrix of the tag in the camera frame
- $Pose_t$: A 3x1 Translation vector of the tag in the camera frame

```
### In real_time.py
detector = Detector(families="tagStandard41h12",nthreads=4)

### In stream.py
PARAMS = [K[0,0],K[1,1],K[2,0],K[2,1]] #elements from the K matrix
TAG_SIZE = .10 #Tag size from Step 1 in meters 
detections = detector.detect(I, estimate_tag_pose=True, camera_params=PARAMS,tag_size=TAG_SIZE)
```

### Pose Estimation
We now have all of the information we need to estimate the position of the camera. What we want is the position of the camera in the global frame, and we can break that into two steps. We get the location of the camera in the tag frame and combine with the location of the tag in the global frame.

First, we need the rotation matrix *R* and translation vector *t* of the camera in the tag frame. We can get these by inverting the Pose_R and Pose_T we get from the Apriltags. R is an orthogonal rotation matrix, meaning the inverse is the same as the transpose. t is a translation vector, so its inverse just negates everything.

$^{camera}R_{tag} = Pose_R^{T}$ <br>
$^{camera}t_{tag} = = -1 * Pose_t$<br>

To get the exact location of the camera in the tag frame, we start with the tag center in the tag frame, multiply by $^{camera}R_{tag}$ and translate by $^{camera}t_{tag}$ . But the coordinates of the tag center in the tag frame is $(0,0,0)$ so our final equation is
$^{camera}Position = ^{camera}R_{tag} * ^{camera}t_{tag}$ <br>

The next step is a correction between the official tag frame on the AprilTags website and the tag frame that I chose to use. My reasoning for choosing to use a slightly different tag frame is that my tag frame aligns with the global frame if all the Euler angles are 0 which helps me visualize the global transformations. The official AprilTags tag frame has the z-axis pointing from the tag center into the wall. The x-axis is to the right in the image taken by the camera, and y is down. This means to transform to our global frame with X right, Y up and Z out of the wall we need $X \rightarrow X, Y \rightarrow -Y, Z \rightarrow -Z$. We can use a 3x3 diagonal permutation matrix $P$ to model this, with either 1 or -1 on the diagonal elements. 
$$ P=
\begin{bmatrix} 1 & 0 & 0\\
0 & -1 & 0 \\
0 & 0 & -1 \end{bmatrix}
$$

$Position_{unofficial} = P * ^{tag}Position$<br>

Now we go from the tag frame to the global frame using the information from the look-up table we populated when doing the scene setup. We have our global rotation matrix $R_{g}$ and global translation vector $t_{g}$. <br>

$Position_global = R_{g} * ^{unofficial}Position + t_{g}$<br>

In Python, we can use the matrix mulptiplcation symbol @ introduced in 3.6. Combining all of the transformations becomes
```
unofficial_tag_position = P @ Pose_R.T @ (-1 * Pose_T)
global_position = R_g @ unofficial_tag_position + t_g
```
If there are multiple tags in a single frame, we estimate the position using all the tags, and then average the estimates.

### Filtering
We now have a pose estimate of the camera position, but how accurate is this measurement? In my work, I've found the Apriltags are accurate to +/- 10 cm when viewed close to straight on and from closer than 2 meters. We do have a lot more information we can use to improve these measurements. <br>
First, we can assume the camera won't move too drastically between frames, so we can use a moving average to smooth out some of the noise from frame to frame. The parameter choice here is how many time steps to average, with more time steps resulting in a more smooth trajectory over time. <br>
Now, with a little more information about where our camera should be, for example if we know the inputs to our self-driving car's motor at the previous frame, we can use a g-h filter or a Kalman filter to further improve our position accuracy. There is a lifetime of really interesting work in this field and one of the best intro books I've read is [here](https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python).

```
pose = moving_average(global_position,previous_positions,num_time_steps)
```

## Putting it all together\
Here is psuedocode for real_time.py
```
#Camera Calibration Parameters
camera_info = load_camera_information()

#Scene Setup
tags=Tag()
tags.add_tag(tag_id,x,y,z,theta_x,theta_y,theta_z)

### Detector
detector = Detector(families="tagStandard41h12",nthreads=4)
#Main Loop
with PiCamera() as camera:
  stream = Stream(tags,camera_info,detector)
  try:
    # Start recording frames to the stream object
    camera.start_recording(stream, format='yuv')
    t0 = time()

    while True:
      camera.wait_recording()
      # If the time limit is reached, end the recording
      if (time() - t0) > MAX_TIME:
          camera.stop_recording()
          break


```
Psuedocode for stream.py
```
def write(raw_bytes):
  I_raw = np.frombuffer(raw_bytes)
  I = undistort(I_raw)
  detected_tags = detector.detect(I)
  raw_position_estimate = estimate_pose(detected_tags)
  smoothed_position = moving_average(raw_position_estimate)
```

## Future Improvements
There are lots of improvements to be made to this, reducing localization error and improving FPS.
#### Scence Setup
- Print higher quality AprilTags on foam board to ensure a flat surface
- Place tags in triangle clusters to improve localization accuracy
#### Camera
- Reduce motion blur caused by moving camera by increasing shutter speed. Increase analog gain to compensate for darker photos. Picamera has a exposure mode setting that I would switch to SPORT
- Improve camera calibration using more images and better setup to improve undistortion.
#### AprilTag Detector
- Find a optimal quad_decimate parameter that balances FPS and detection accuracy
- Play with quad_sigma parameter and see if it improves detection accuracy
- Play with decode_sharpening parameter and see it if improves detection accuracy
#### Pose Estimation
- Adding orientation as well as position will help inform the car's trajectory
#### Filtering
- Add more information about the car model and the inputs from the servo to utilize a Kalman filter

