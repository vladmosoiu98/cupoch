import sys
import math
import pyrealsense2.pyrealsense2 as rs
import numpy as np
from enum import IntEnum

from datetime import datetime

import cupoch as cph

#cph.initialize_allocator(cph.PoolAllocation, 1000000000)


class Preset(IntEnum):
    Custom = 0
    Default = 1
    Hand = 2
    HighAccuracy = 3
    HighDensity = 4
    MediumDensity = 5


def get_intrinsic_matrix(frame):
    intrinsics = frame.profile.as_video_stream_profile().intrinsics
    out = cph.camera.PinholeCameraIntrinsic(424, 240, intrinsics.fx, intrinsics.fy, intrinsics.ppx, intrinsics.ppy)
    return out
    
def calc_sin(angle):
	return math.sin(math.radians(angle))

def calc_cos(angle):
	return math.cos(math.radians(angle))
	
def calc_transformation(degx, degy, degz):
    # Initial matrix - flip the image
    flip_mat = np.array([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
    
    # Calculate the angles sin / cos
    sinx = calc_sin(degx)
    siny = calc_sin(degy)
    sinz = calc_sin(degz)
    cosx = calc_cos(degx)
    cosy = calc_cos(degy)
    cosz = calc_cos(degz)
    
    print("matrix x:")
    matX = np.array([[1, 0, 0, 0], [0, cosx, -sinx, 0], [0, sinx, cosx, 1], [0, 0, 0, 1]])
    print(matX)
    
    print("matrix y:")
    matY = np.array([[cosy, 0, siny, 0], [0, 1, 0, 0], [-siny, 0, cosy, 0], [0, 0, 0, 1]])
    print(matY)
    
    print("matrix z:")
    matZ = np.array([[cosz, -sinz, 0, 0], [sinz, cosz, 0, 0], [0, 1, 0, 0], [0, 0, 0, 1]])
    print(matZ)
    
    transf_mat = np.matmul(flip_mat, matX)
    print("first matmul:")
    print(transf_mat)
    
    transf_mat = np.matmul(transf_mat, matY)
    print("second matmul:")
    print(transf_mat)
    
    transf_mat = np.matmul(transf_mat, matZ)
    print("last matmul:")
    print(transf_mat)
    
    return transf_mat
	
def calc_transf_on_axis(axis, deg):

    # Calculate the angle sin / cos
    sindeg = calc_sin(deg)
    cosdeg = calc_cos(deg)

    if (axis == 'x'):
    	print("matrix x:")
    	mat = np.array([[1, 0, 0, 0], [0, cosdeg, -sindeg, 0], [0, sindeg, cosdeg, 0], [0, 0, 0, 1]])
    	print(mat)
    elif (axis == 'y'):
    	  print("matrix y:")
    	  mat = np.array([[cosdeg, 0, sindeg, 0], [0, 1, 0, 0], [-sindeg, 0, cosdeg, 0], [0, 0, 0, 1]])
    	  print(mat)
    elif (axis == 'z'):
    	  print("matrix z:")
    	  mat = np.array([[cosdeg, -sindeg, 0, 0], [sindeg, cosdeg, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    	  print(mat)

    return mat
    
def calc_transf_img_flipped(mat):
	flip_mat = np.array([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
	return np.matmul(flip_mat, mat)

def multiply_mat(matA, matB):
	return np.matmul(matA, matB)
	

if __name__ == "__main__":

    # Create a pipeline
    pipeline = rs.pipeline()

    # Create a config and configure the pipeline to stream
    #  different resolutions of color and depth streams
    config = rs.config()

    config.enable_stream(rs.stream.depth, 424, 240, rs.format.z16, 30)

    # Start streaming
    profile = pipeline.start(config)
    depth_sensor = profile.get_device().first_depth_sensor()

    # Using preset HighAccuracy for recording
    depth_sensor.set_option(rs.option.visual_preset, Preset.Default)

    vis = cph.visualization.Visualizer()
    vis.create_window()

    pcd = cph.geometry.PointCloud()
    
    ##############################################################################
    
    #flip_transform = calc_transformation(90, 45, 45)
    mat = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    flip_z = calc_transf_on_axis('z', 90)
    flip_x = calc_transf_on_axis('x', 45)
    flip_y = calc_transf_on_axis('y', 45)
    flip_img_y = calc_transf_on_axis('y', 180)
    
    # Method flip at the end:
    #flip_90 = multiply_mat(mat, flip_z)
    #flip_45_1 = multiply_mat(flip_90, flip_x)
    #flip_45_2 = multiply_mat(flip_45_1, flip_y)
    #flip_transform = multiply_mat(flip_45_2, flip_img_y)
    
    # Method flip first
    flip_img = calc_transf_img_flipped(flip_z)
    flip_45_1 = multiply_mat(flip_img, flip_y)
    flip_45_2 = multiply_mat(flip_45_1, flip_x)
    flip_transform = flip_45_2

    print("final:")
    print(flip_transform)

    ##############################################################################
  
    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    intrinsic = cph.camera.PinholeCameraIntrinsic(get_intrinsic_matrix(depth_frame))

    # Streaming loop
    frame_count = 0
    try:
        while True:

            #dt0 = datetime.now()

            # Get frameset of color and depth
            frames = pipeline.wait_for_frames()

            depth_frame = frames.get_depth_frame()

            if not depth_frame:
                continue

            #dt1 = datetime.now()
            depth_image = cph.geometry.Image(np.array(depth_frame.get_data()))

            #dt2 = datetime.now()
            temp = cph.geometry.PointCloud.create_from_depth_image(depth_image, intrinsic)
            
            # Apply transformation matrix
            temp.transform(flip_transform)
            pcd.points = temp.points


            #dt3 = datetime.now()
            if frame_count == 0:
                vis.add_geometry(pcd)

            vis.update_geometry(pcd)
            vis.poll_events()
            vis.update_renderer()

            #dt4 = datetime.now()
            #process_time = dt4 - dt0
            #print("FPS: " + str(1 / process_time.total_seconds()))
            '''print(
                "1. Get frame:",
                (dt1 - dt0).total_seconds(),
                "2. Initialize images:",
                (dt2 - dt1).total_seconds(),
                "3. Construct pointcloud:",
                (dt3 - dt2).total_seconds(),
                "4. Display pointcloud:",
                (dt4 - dt3).total_seconds(),
            )'''
            frame_count += 1

    finally:
        pipeline.stop()
    vis.destroy_window()
