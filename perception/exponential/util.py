import numpy as np
import cv2
import time
import traceback

def reg_img_to_world(depth_img, sky_level=0.9, depth_scaling_factor=1000) -> np.ndarray:
    # (Intrinsic) K Matrix
    k = np.identity(3)
    k[0, 2] = 800.0 / 2.0 
    k[1, 2] = 600.0 / 2.0
    k[0, 0] = k[1, 1] = 800 / (2.0 * np.tan(70.0 * np.pi / 360.0))
    # k[1, 1] = 600 / (2.0 * np.tan(70.0 * np.pi / 360.0))
    
    # print(k.dtype)
    
    intrinsics_matrix = k
    
    # get a 2 x N array for their indices
    bool_mat = depth_img > (depth_img.min() - 1)
    ground_loc = np.where(bool_mat)
    depth_val = depth_img[bool_mat] * depth_scaling_factor
    ground_loc = ground_loc * depth_val

    # compute raw_points
    raw_points = np.vstack([ground_loc, depth_val])

    # convert to cords_y_minus_z_x
    return np.linalg.inv(intrinsics_matrix) @ raw_points

def img_to_world(depth_img, sky_level=0.2, depth_scaling_factor=1000) -> np.ndarray:
    # (Intrinsic) K Matrix
    k = np.identity(3)
    k[0, 2] = 800 / 2.0 
    k[1, 2] = 600 / 2.0
    k[0, 0] = k[1, 1] = 800 / (2.0 * np.tan(70.0 * np.pi / 360.0))
    # k[1, 1] = 600 / (2.0 * np.tan(70.0 * np.pi / 360.0))
    intrinsics_matrix = k
    
    # get a 2 x N array for their indices
    bool_mat3 = (0.1 < depth_img) * (depth_img < sky_level)
    # bool_mat2 = 0.1 < depth_img
    # bool_mat3 = bool_mat * bool_mat2
    ground_loc = np.where(bool_mat3)
    depth_val = depth_img[bool_mat3] * depth_scaling_factor
    ground_loc = ground_loc * depth_val

    # compute raw_points
    raw_points = np.vstack([ground_loc, depth_val])

    # convert to cords_y_minus_z_x
    return np.linalg.inv(intrinsics_matrix) @ raw_points

def get_roll_stats(depth_image):
    xyz = img_to_world(depth_image).T
    xyz_samp = xyz[np.random.choice(xyz.shape[0], 400, replace=False), :]
    u, s, vt = np.linalg.svd(xyz_samp - xyz_samp.mean())
    
    reg = vt[2]
    no_y = np.array(reg, copy=True)
    no_y[1] = 0
    nvt, nx = np.linalg.norm(reg), np.linalg.norm(no_y)
    cos_ang = np.dot(reg, no_y) / (nvt * nx)
    
    unitcross = lambda a, b : np.cross(a, b) / np.linalg.norm(np.cross(a, b))
    rot_axis = unitcross(vt[2], no_y)
    
    return np.arccos(cos_ang), rot_axis # radians

def roll_frame(depth_image, ang, rot_axis, no_axis=False):
    if no_axis:
        return depth_image
    
    xyz = reg_img_to_world(depth_image).T
    xyz_mean = xyz.mean(axis=0)
    xyz = xyz - xyz_mean
    
    cos_ang = np.cos(ang)
    x, y, z = rot_axis
    
    c = cos_ang
    s = np.sqrt(1-c*c)
    C = 1-c
    rmat = np.array([[x*x*C+c, x*y*C-z*s, x*z*C+y*s ],
                     [y*x*C+z*s, y*y*C+c, y*z*C-x*s ],
                     [z*x*C-y*s, z*y*C+x*s, z*z*C+c]])
    rot_xyz = rmat @ (xyz.T)
    rot_xyz = rot_xyz.T + xyz_mean
    return rot_xyz[:, 2].reshape(depth_image.shape) / 1000
