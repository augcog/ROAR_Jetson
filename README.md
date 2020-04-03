# ROARVR

## Intro
This is the code for ROAR platform. It features low-latency jitter-free VR support which provides you with immersive driving experience. In addition, it is extensible with respect to control side. By default, it is controlled by analog controller with help of Arduino, where Arduino receives signal from analog controller and control the car, which we call **Analog-Control** mode. When there is no analog controller on, the system switches to **Jetson-Control** mode, where Arduino receives signal from Jetson. In this case, you are allowed to implement your novel physical (e.g. joysticks) or algorithmic (e.g. autonomous pilot) controller and easily plug it in our system.

## Setup

### Jetson
Clone this repository into your Jetson. The code runs on Python 3. Pre-requisite libraries include: `numpy`, `opencv-python`, `statistics`, `prettytable`, `pyserial`, `pyrealsense2`. Use `pip` to install them. If you have MIPI camera connected or you want to involve VR, you also need to install `Nvidia Accelatated GStreamer`, you can find the setup guidance [here](https://developer.download.nvidia.com/embedded/L4T/r32_Release_v1.0/Docs/Accelerated_GStreamer_User_Guide.pdf?BlbMrXc01wJrcGEdNwtlAEY35R0ofBnDCcpbfH9g71zqPsrglP7iv5hqz5_LciiElQF-TU38MzH9vO70egx8Fo7CgUvgJxcYrKVlPczq30tkevp9TbEg1nZJjtUmx7_DtTArOCqYbbH6coyDRsnPganEgVEkKVqCE33mXV__VE_2LGytTSE
).

### Arduino
Then we need to upload the Arduino code into Arduino board. This is done in Jetson. In Jetson, launch **Arduino IDE**. Select `Arduino Nano` in `Tools -> Board: "..."` and `/dev/ttyUSB0` in `Tools -> Port: "..."`. Then click "Upload" button to burn the code into the board.


### PC
This part is necessary only if you want to involve VR into your tryout. To clarify, the system totally works find without VR. First and foremost, you have to have your Jetson and PC connected to the same local network and make sure network is in good condition to get smooth and jitter-free video streaming. Then setup VR device. Install [Oculus software](https://www.oculus.com/setup/) and setup your VR device according to guidance. And you need to have **Unity** installed. Finally clone [this repository](https://github.com/augcog/IRG-RACING-VR) into your PC. 
To develop and build the unity project, make sure you intstall [GStreamer](https://gstreamer.freedesktop.org/documentation/installing/on-windows.html?gi-language=c) first and then [build OpenCV with GStreamer](https://cv-tricks.com/how-to/installation-of-opencv-4-1-0-in-windows-10-from-source/).

## Run
Enter `ROARVR` folder. Before you run the main program, you may want to modify the configuration in `myconfig.py`. For those whose cars have no MIPI camera mounted as rear camera, set `ENABLE_CSIC` false to disable it. Note that `ENABLE_CISC` has impact on details about how to customize your own `Controller` (will be explained later).

If you get VR involved, set `CLIENT_IP` to ip address of your PC in the format as `"192.168.1.50"`. You can also specify ip address in command line. You can change `IMAGE_W` and `IMAGE_H` to get a different resolution, but along with that, you need to also change some parameters in Unity. First click on object 'Utility' in 'SampleScene', under the 'Inspector' tab on the right side, you can see two public variables `width` and `height`,these is the receiving resolution. Set these two values the same as `IMAGE_W` and `IMAGE_H`, which is the sending resolution. Besides, you may want to costomize the rendering resolution for either frontview window or rearview mirror, these settings can be found under the Inspector tab in `Canvas/FrontView` and `backmirror/Canvas/Image`.

If you want to play it out in **Jetson-Control** mode, you can change `THROTTLE_MAX` and `STEERING_MAX` to automatically scale the values sent to Arduino to limit maximum speed and angle. To use your own custom `Controller` (will be explained later) instead of `NaiveController`, change `CONTROLLER` parameter to the name of the `Controller` class you define.

After the configuration is all set, execute `./roar-vr.py` in command line to run in **Analog-Control** mode or `./roar-vr.py -c` to run in **Jetson-Control** mode. When you are playing with VR, you can specify ip address of PC in here by adding parameter in the command as `--ip "192.168.1.50"`. This will override the `CLIENT_IP` setting in `myconfig.py`.

To enable VR, you then need to launch **Unity** in PC, open our another repository of Unity part and click "Play". When you want to stop everything, there is a bug here. Never stop program running on Jetson before stopping the game playing in **Unity**, otherwise **Unity** may hang.

## Customize
To customize your own `Controller`, you need to create your own `XXXController` class INSIDE `controller.py`, having it inherited from `Controller` class and overriding `update` and `run_threaded` method. Please refer to `controller.py` to see the template.

The most important method is `run_threaded`. The main vehicle thread will call this function to get throttle and steering value for next time step in each loop, and control the car accordingly. The function takes a sequence of data fetched from camera as input and return throttle and steering values as output. These values both range from -1 to 1. If `ENABLE_CSIC` is `False`, which means only the front camera is used, inputs are `image_array`, `depth_array`, `imu_acl_x`, `imu_acl_y`, `imu_acl_z`, `imu_gyr_x`, `imu_gyr_y`, `imu_gyr_z`. If `ENABLE_CSIC` is `True`, which means the rear MIPI camera is also used, inputs are `image_array`, `image_array_rear`, `depth_array`, `imu_acl_x`, `imu_acl_y`, `imu_acl_z`, `imu_gyr_x`, `imu_gyr_y`, `imu_gyr_z`. What you need to do is computing throttle and steering according to these information. You can truncate the list according to your need. For example, if only RGB images from both cameras are necessary to control the car in your method, the signature of this method can be `run_threaded(self, image_array, image_array_rear, **args)`. You can also choose not to override `run_threaded` and use `run_threaded` of the base class. But then you need to implement `update` method to update `self.new_throttle` and `self.new_steering`.

If you want to implement some logic that you do not want to be synchronous with the main vehicle thread and block the main thread, you can write it in `update` method, which runs in another thread. An example would be `NaiveController`, where we naively get new throttle and steering data from command line input. Receiving input from command line is blocking, so we implement it in `update` since we do not want it to make the car stuck.
