import pyrealsense2 as rs
import numpy as np
from util import get_xyz
import time
# import cupoch as g3d
import open3d as o3d

pipeline = rs.pipeline()
config = rs.config()

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
profile = pipeline.start(config)

align_to = rs.stream.color
align = rs.align(align_to)

stream = profile.get_stream(rs.stream.depth)
intr = stream.as_video_stream_profile().get_intrinsics()
print("Intrinsics:", intr)

frames = pipeline.wait_for_frames()
aligned_frames = align.process(frames)

aligned_depth_frame = aligned_frames.get_depth_frame()

pc = rs.pointcloud()
t1 = time.time()
points = pc.calculate(aligned_depth_frame)
vtx = np.ndarray(buffer=points.get_vertices(), dtype=np.float32, shape=(640*480, 3))
t2 = time.time()

print('VTX Shape:', vtx.shape, 'Check:', 640 * 480, 'FPS Test:', 1 / (t2 - t1))

print(vtx)
