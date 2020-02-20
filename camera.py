import os
import time
import numpy as np
from PIL import Image
import glob
from irmark1.utils import rgb2gray


class BaseCamera:

    def run_threaded(self):
        return self.frame


class CSICamera(BaseCamera):
    '''
    Camera for Jetson Nano IMX219 based camera
    Credit: https://github.com/feicccccccc/donkeycar/blob/dev/donkeycar/parts/camera.py
    gstreamer init string from https://github.com/NVIDIA-AI-IOT/jetbot/blob/master/jetbot/camera.py
    '''
    def gstreamer_pipeline(self, capture_width=3280, capture_height=2464, output_width=224, output_height=224, framerate=21, flip_method=0) :   
        return 'nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM), width=%d, height=%d, format=(string)NV12, framerate=(fraction)%d/1 ! nvvidconv flip-method=%d ! nvvidconv ! video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! videoconvert ! appsink' % (
                capture_width, capture_height, framerate, flip_method, output_width, output_height)

    def gstreamer_pipelineout(self, output_width=1280, output_height=720, framerate=21,client_ip='127.0.0.1') : 
        return 'appsrc ! videoconvert ! video/x-raw, format=(string)BGRx, width=%d, height=%d, framerate=(fraction)%d/1 ! videoconvert ! video/x-raw, format=(string)I420 ! omxh264enc tune=zerolatency bitrate=8000000 speed-preset=ultrafast ! video/x-h264, stream-format=byte-stream ! rtph264pay mtu=1400 ! udpsink host=%s port=5000 sync=false async=false'%(output_width,output_height,framerate,client_ip)
    
    def __init__(self, image_w=160, image_h=120, image_d=3, capture_width=640, capture_height=480, framerate=60, gstreamer_flip=0,client_ip='127.0.0.1'):
        '''
        gstreamer_flip = 0 - no flip
        gstreamer_flip = 1 - rotate CCW 90
        gstreamer_flip = 2 - flip vertically
        gstreamer_flip = 3 - rotate CW 90
        '''
        self.w = image_w
        self.h = image_h
        self.running = True
        self.frame = None
        self.flip_method = gstreamer_flip
        self.capture_width = capture_width
        self.capture_height = capture_height
        self.framerate = framerate
        self.client_ip=client_ip

    def init_camera(self):
        import cv2

        # initialize the camera and stream
        self.camera = cv2.VideoCapture(
            self.gstreamer_pipeline(
                capture_width =self.capture_width,
                capture_height =self.capture_height,
                output_width=self.w,
                output_height=self.h,
                framerate=self.framerate,
                flip_method=self.flip_method),
            cv2.CAP_GSTREAMER)
        self.out_send = cv2.VideoWriter(self.gstreamer_pipelineout(
                output_width=self.w,
                output_height=self.h,
                framerate=self.framerate,
                client_ip=self.client_ip),cv2.CAP_GSTREAMER,0,self.framerate,(self.w,self.h), True)

        self.poll_camera()
        print('CSICamera loaded.. .warming camera')
        time.sleep(2)
        
    def update(self):
        self.init_camera()
        while self.running:
            self.poll_camera()

    def poll_camera(self):
        import cv2
        self.ret , frame = self.camera.read()
        self.out_send.write(frame)
        self.frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        

    def run(self):
        self.poll_camera()
        return self.frame

    def run_threaded(self):
        return self.frame
    
    def shutdown(self):
        self.running = False
        print('stoping CSICamera')
        time.sleep(.5)
        del(self.camera)
        out_send.release()

class V4LCamera(BaseCamera):
    '''
    uses the v4l2capture library from this fork for python3 support: https://github.com/atareao/python3-v4l2capture
    sudo apt-get install libv4l-dev
    cd python3-v4l2capture
    python setup.py build
    pip install -e .
    '''
    def __init__(self, image_w=160, image_h=120, image_d=3, framerate=20, dev_fn="/dev/video0", fourcc='MJPG'):

        self.running = True
        self.frame = None
        self.image_w = image_w
        self.image_h = image_h
        self.dev_fn = dev_fn
        self.fourcc = fourcc

    def init_video(self):
        import v4l2capture

        self.video = v4l2capture.Video_device(self.dev_fn)

        # Suggest an image size to the device. The device may choose and
        # return another size if it doesn't support the suggested one.
        self.size_x, self.size_y = self.video.set_format(self.image_w, self.image_h, fourcc=self.fourcc)

        print("V4L camera granted %d, %d resolution." % (self.size_x, self.size_y))

        # Create a buffer to store image data in. This must be done before
        # calling 'start' if v4l2capture is compiled with libv4l2. Otherwise
        # raises IOError.
        self.video.create_buffers(30)

        # Send the buffer to the device. Some devices require this to be done
        # before calling 'start'.
        self.video.queue_all_buffers()

        # Start the device. This lights the LED if it's a camera that has one.
        self.video.start()


    def update(self):
        import select
        from irmark1.parts.image import JpgToImgArr

        self.init_video()
        jpg_conv = JpgToImgArr()

        while self.running:
            # Wait for the device to fill the buffer.
            select.select((self.video,), (), ())
            image_data = self.video.read_and_queue()
            self.frame = jpg_conv.run(image_data)


    def shutdown(self):
        self.running = False
        time.sleep(0.5)


class RS_D435i(object):
    '''
    Intel RealSense depth camera D435i combines the robust depth sensing capabilities of the D435 with the addition of an inertial measurement unit (IMU).
    ref: https://www.intelrealsense.com/depth-camera-d435i/
    '''
    def gstreamer_pipelineout(self, output_width=1280, output_height=720, framerate=21,client_ip='127.0.0.1') : 
        return 'appsrc ! videoconvert ! video/x-raw, format=(string)BGRx, width=%d, height=%d, framerate=(fraction)%d/1 ! videoconvert ! video/x-raw, format=(string)I420 ! omxh264enc tune=zerolatency bitrate=8000000 speed-preset=ultrafast ! video/x-h264, stream-format=byte-stream ! rtph264pay mtu=1400 ! udpsink host=%s port=5001 sync=false async=false'%(output_width,output_height,framerate,client_ip)

    def __init__(self, image_w=640, image_h=480, image_d=3, image_output=True, framerate=30,client_ip='127.0.0.1'):
        #Using the image_output will grab two image streams from the fisheye cameras but return only one.
        #This can be a bit much for USB2, but you can try it. Docs recommend USB3 connection for this.
        self.image_output = image_output
        self.client_ip=client_ip
        # Declare RealSense pipeline, encapsulating the actual device and sensors
        self.pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.gyro)
        cfg.enable_stream(rs.stream.accel)

        if self.image_output:
            cfg.enable_stream(rs.stream.color, image_w, image_h, rs.format.bgr8, framerate) # color camera

            self.out_send = cv2.VideoWriter(self.gstreamer_pipelineout(
                output_width=image_w,
                output_height=image_h,
                framerate=framerate,
                client_ip=self.client_ip),cv2.CAP_GSTREAMER,0,framerate,(image_w,image_h), True)

            cfg.enable_stream(rs.stream.depth,  image_w, image_h,  rs.format.z16, framerate) # depth camera

        # Start streaming with requested config
        self.pipe.start(cfg)
        self.running = True

        zero_vec = (0.0, 0.0, 0.0)
        self.gyro = zero_vec
        self.acc = zero_vec
        self.img = None
        self.dimg = None


    def poll(self):
        try:
            frames = self.pipe.wait_for_frames()
        except Exception as e:
            logging.error(e)
            return

        if self.image_output:
            color_frame = frames.get_color_frame()

            depth_frame = frames.get_depth_frame()
            self.img = np.asanyarray(color_frame.get_data())
            self.dimg = np.asanyarray(depth_frame.get_data())
            self.out_send.write(self.img)

        # Fetch IMU frame
        accel = frames.first_or_default(rs.stream.accel)
        gyro = frames.first_or_default(rs.stream.gyro)
        if accel and gyro:
            self.acc = accel.as_motion_frame().get_motion_data()
            self.gyro = gyro.as_motion_frame().get_motion_data()
            # print('realsense accel(%f, %f, %f)' % (self.acc.x, self.acc.y, self.acc.z))
            # print('realsense gyro(%f, %f, %f)' % (self.gyro.x, self.gyro.y, self.gyro.z))

    def update(self):
        while self.running:
            self.poll()

    def run_threaded(self):
        return self.img, self.dimg

    def run(self):
        self.poll()
        return self.run_threaded()

    def shutdown(self):
        self.running = False
        time.sleep(0.1)
        self.pipe.stop()
