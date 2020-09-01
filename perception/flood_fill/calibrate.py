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
        if not freeze:
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
        
        cv2.imshow('Display', im_color)

        if cv2.waitKey(0) & 0xFF == ord('q'):
            break

        if cv2.waitKey(0) & 0xFF == ord('c'):
            im_color[:400, :] = 0
            cv2.imshow('Display', im_color)

        if cv2.waitKey(0) & 0xff == ord('n'):
            freeze = False

    cv2.destroyAllWindows()
    pipeline.stop()

start()

"""
except:
    print('Error')

finally:
    pipeline.stop()
"""
