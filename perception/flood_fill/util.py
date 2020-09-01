import numpy as np
import pyrealsense2 as rs

"""
Intrinsics: 
width: 640, height: 480, 
ppx: 318.462, ppy: 240.021, 
fx: 384.72, fy: 384.72, model: 4, 
coeffs: [0, 0, 0, 0, 0]

[[384.720,   0.000, 316.720]
 [  0.000, 384.720, 240.021]
 [  0.000,   0.000,   1.000]]

"""

def get_inv_k(intrin):
    k = np.identity(3)
    k[0, 2] = intrin.ppx
    k[1, 2] = intrin.ppy
    k[0, 0] = intrin.fx
    k[1, 1] = intrin.fy
    return np.linalg.inv(k)

def get_xyz(depth_image, intrin):
    pix2xyz = lambda i, j : [
        depth_image[i, j] * (i - intrin.ppx) / intrin.fx,
        depth_image[i, j] * (j - intrin.ppy) / intrin.fy,
        depth_image[i, j]
    ]
    d1, d2 = depth_image.shape
    idx, jdx = np.indices(depth_image.shape)
    xyz_matrix = np.reshape(pix2xyz(idx, jdx), (3, d1 * d2)).T
    
    return xyz_matrix

def get_xyz2(depth_img, inv_intrin):
    bool_mat = depth_img > (depth_img.min() - 1)
    ground_loc = np.where(bool_mat)
    depth_val = depth_img[bool_mat]
    ground_loc = ground_loc * depth_val
    raw_points = np.vstack([ground_loc, depth_val])

    return inv_intrin @ raw_points
