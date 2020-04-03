#!/usr/bin/env python3
import os
import time
import argparse
import numpy as np

from config import load_config
import vehicle
from camera import RS_D435i, CSICamera
from sender import Sender
from receiver import Receiver
from controller import *
from datastore import TubHandler


def drive(cfg, client_ip=None, to_control=False):
    V = vehicle.Vehicle()
    client_ip = client_ip or cfg.CLIENT_IP

    cam_front = RS_D435i(image_w=cfg.IMAGE_W, image_h=cfg.IMAGE_H, image_d=cfg.IMAGE_DEPTH,
            framerate=cfg.CAMERA_FRAMERATE, client_ip=client_ip)
    outputs_front = ['cam/image_array', 'cam/depth_array', 'imu/acl_x', 'imu/acl_y', 'imu/acl_z', 'imu/gyr_x', 'imu/gyr_y', 'imu/gyr_z' ]
    V.add(cam_front, outputs=outputs_front, threaded=True)

    if cfg.ENABLE_CSIC:
        cam_rear = CSICamera(image_w=cfg.IMAGE_W, image_h=cfg.IMAGE_H, image_d=cfg.IMAGE_DEPTH,
                framerate=cfg.CAMERA_FRAMERATE, gstreamer_flip=cfg.CSIC_CAM_GSTREAMER_FLIP_PARM, client_ip=client_ip)
        outputs_rear = ['cam_image_array_rear']
        V.add(cam_rear, outputs=outputs_rear, threaded=True)

    if to_control:
        ctr = eval(cfg.CONTROLLER)()
        if cfg.ENABLE_CSIC:
            ctr_inputs = ['cam/image_array', 'cam_image_array_rear', 'cam/depth_array', 'imu/acl_x', 'imu/acl_y', 'imu/acl_z', 'imu/gyr_x', 'imu/gyr_y', 'imu/gyr_z' ]
        else:
            ctr_inputs = ['cam/image_array', 'cam/depth_array', 'imu/acl_x', 'imu/acl_y', 'imu/acl_z', 'imu/gyr_x', 'imu/gyr_y', 'imu/gyr_z' ];

        V.add(ctr, inputs=ctr_inputs, outputs=['throttle', 'steering'], threaded=True)
        V.add(Sender(cfg), inputs=['throttle', 'steering'], threaded=True)
    else:
        V.add(Receiver(client_ip, cfg), outputs=['throttle', 'steering'], threaded=True)

    if cfg.TO_SAVE:
        if cfg.ENABLE_CSIC:
            save_inputs = ['cam/image_array', 'cam_image_array_rear']
            save_types = ['image_array', 'image_array']
        else:
            save_inputs = ['cam/image_array']
            save_types = ['image_array']
        if cfg.SAVE_DEPTH:
            save_inputs += ['cam/depth_array']
            save_types += ['lossless_image_array']
        if cfg.SAVE_IMU:
            save_inputs += ['imu/acl_x', 'imu/acl_y', 'imu/acl_z', 'imu/gyr_x', 'imu/gyr_y', 'imu/gyr_z']
            save_types += ['float', 'float', 'float', 'float', 'float', 'float']
        th = TubHandler(cfg.SAVE_PATH)
        tub = th.new_tub_writer(inputs=save_inputs, types=save_types)
        V.add(tub, inputs=save_inputs, outputs=['tub/num_records'], run_condition='recording')

    V.start(rate_hz=cfg.DRIVE_LOOP_HZ, 
            max_loop_count=cfg.MAX_LOOPS)


if __name__ == '__main__':
    cfg = load_config('myconfig.py')

    parser = argparse.ArgumentParser();
    parser.add_argument('--ip', required=False, default=None, type=str)
    parser.add_argument('-c', '--control', required=False, action='store_true')
    args = parser.parse_args()
    
    drive(cfg, args.ip, args.control)
