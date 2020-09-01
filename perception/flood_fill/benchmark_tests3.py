import pyrealsense2 as rs
import numpy as np
from util import get_inv_k
import time

pipeline = rs.pipeline()
config = rs.config()

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
profile = pipeline.start(config)

align_to = rs.stream.color
align = rs.align(align_to)

stream = profile.get_stream(rs.stream.depth)
intr = stream.as_video_stream_profile().get_intrinsics()
inv_k = get_inv_k(intr)
print("Intrinsics:\n", np.linalg.inv(inv_k))

d1, d2 = 640, 480
idx, jdx = np.indices((d1, d2))
idx_back = np.clip(idx - 1, 0, idx.max()).flatten()
idx_front = np.clip(idx + 1, 0, idx.max()).flatten()
jdx_back = np.clip(jdx - 1, 0, jdx.max()).flatten()
jdx_front = np.clip(jdx + 1, 0, jdx.max()).flatten()
idx = idx.flatten()
jdx = jdx.flatten()

rand_idx = np.random.choice(np.arange(idx.shape[0]), size=20000, replace=False)
f1 = (idx_front * d2 + jdx)[rand_idx]
f2 = (idx_back  * d2 + jdx)[rand_idx]
f3 = (idx * d2 + jdx_front)[rand_idx]
f4 = (idx * d2 + jdx_back )[rand_idx]
norm_fill = np.zeros((idx.shape[0]))
norm = np.array([-0.9852, 0.0001, -0.1709]) # purely test

def normalize_v3(arr):
    lens = np.sqrt(arr[:, 0]**2 + arr[:,1]**2 + arr[:,2]**2) + 1e-10
    arr[:,0] /= lens
    arr[:,1] /= lens
    arr[:,2] /= lens
    return arr

pc = rs.pointcloud()
time_array = []
for _ in range(1000):
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)
    aligned_depth_frame = aligned_frames.get_depth_frame()

    t1 = time.time()
    points = pc.calculate(aligned_depth_frame)
    vtx = np.ndarray(buffer=points.get_vertices(), dtype=np.float32, shape=(640*480, 3))
    
    x = vtx[f1, :] - vtx[f2, :]
    y = vtx[f3, :] - vtx[f4, :]
    xyz_norm = normalize_v3(np.cross(x, y))
    norm_flat = xyz_norm @ norm
    norm_fill[rand_idx] = norm_flat
    norm_matrix = np.abs(norm_fill.reshape((d1, d2)))
    
    t2 = time.time()
    time_array.append(t2 - t1)

print('VTX Shape:', vtx.shape, 'FPS Test:', 1 / np.mean(time_array))
