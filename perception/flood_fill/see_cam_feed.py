import numpy as np
import cv2
import pyrealsense2 as rs


# Setup
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
profile = pipeline.start(config)
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
align_to = rs.stream.color
align = rs.align(align_to)

def start():
    freeze = False

    while True:
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        aligned_depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        gray = cv2.normalize(
            depth_image, None, 255, 0, norm_type=cv2.NORM_MINMAX, 
            dtype=cv2.CV_8U
        )
        gray_3c = cv2.merge([gray, gray, gray])
        im_color = cv2.applyColorMap(gray_3c, cv2.COLORMAP_JET)
        
        # (s1, s2) = (start on horizontal, start on vertical)
        # (e1, e2) = (end on horizontal  , end on vertical  )
        start, end, color, thicc = (40, 350), (600, 470), (255, 0, 0), 2
        drawn_color = cv2.rectangle(color_image, start, end, color, thicc)
        drawn_depth = cv2.rectangle(im_color   , start, end, color, thicc)
        
        drawn_color[:start[1], :, :] = 0
        drawn_color[:, :start[0], :] = 0
        drawn_color[end[1]:,   :, :] = 0
        drawn_color[:,   end[0]:, :] = 0

        # cv2.imshow('Color', drawn_color)
        cv2.imshow('Depth', drawn_depth)

        if cv2.waitKey(100) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()
    pipeline.stop()

start()

"""
except:
    print('Error')

finally:
    pipeline.stop()
"""
