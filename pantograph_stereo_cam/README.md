# Stereo vision package
## Description

This package adds the necesary code to use a stereo vision system with the pantograph.

The Stereo vision system is composed of 2 usb cameras ([specs:](https://www.waveshare.com/ov5640-5mp-usb-camera-b.htm)) mounted to a 3D printed support, all the code is in Python and extensively uses the OpenCV library. 

The calibration of the cameras can be done using the _calib.py_ script, a modified version of the program made by Dr. T.BATPUREV, for more information check his [github](https://github.com/TemugeB/python_stereo_camera_calibrate)

## Usage
The stereo_vision node can be launched in the ros2 launch file. 

Once the stereo_vision node is launched, 2 windows are displayed, in the first the images and masks of each camera are displayed as well as the calculated coordinates of the needle marker. In the second window, there are trackbars that can be used to adjust the parameters for the detection of the marker. 

By pressing the **space** key the current 3D position of the needle marker is recorded and saved in the `config` folder, the number of recorded points can be adjusted at line 189 of `stereo_tracker.py` 

The program closes by pressing the **ESC** key.

## Camera calibration

If the position of the cameras or the cameras themselves are changed, it is necesary to recalibrate the stereo-vision system.

For this simply run the script `calib.py` along with the calibration parameters file:
```
python3 calib.py calibration_settings.yaml
``` 

Then follow the steps explained in [Dr. BATPUREV GitHub :](https://github.com/TemugeB/python_stereo_camera_calibrate)

The calibration program produces the folder `config` in which the camera parameters as well as the images used for the calibration can be found. These files are then used by the `stereo_tracker.py` to perform the triangulation of the position of the needle tip in the robot world frame.   