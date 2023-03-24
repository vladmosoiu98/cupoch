import sys
import pyrealsense2.pyrealsense2 as rs
import numpy as np
from enum import IntEnum

from datetime import datetime

import cupoch as cph

cph.initialize_allocator(cph.PoolAllocation, 1000000000)


class Preset(IntEnum):
    Custom = 0
    Default = 1
    Hand = 2
    HighAccuracy = 3
    HighDensity = 4
    MediumDensity = 5


def get_intrinsic_matrix(frame):
    intrinsics = frame.profile.as_video_stream_profile().intrinsics
    out = cph.camera.PinholeCameraIntrinsic(
        424, 240, intrinsics.fx, intrinsics.fy, intrinsics.ppx, intrinsics.ppy)
    return out


if __name__ == "__main__":

    # Create a pipeline
    pipeline = rs.pipeline()

    # Create a config and configure the pipeline to stream
    #  different resolutions of color and depth streams
    config = rs.config()

    config.enable_stream(rs.stream.depth, 424, 240, rs.format.z16, 15)

    # Start streaming
    profile = pipeline.start(config)
    depth_sensor = profile.get_device().first_depth_sensor()

    # Using preset HighAccuracy for recording
    depth_sensor.set_option(rs.option.visual_preset, Preset.Default)

    vis = cph.visualization.Visualizer()
    vis.create_window()

    pcd = cph.geometry.PointCloud()
    flip_transform = [[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]]

    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    intrinsic = cph.camera.PinholeCameraIntrinsic(
        get_intrinsic_matrix(depth_frame))

    # Streaming loop
    frame_count = 0
    try:
        while True:

            # dt0 = datetime.now()

            # Get frameset of color and depth
            frames = pipeline.wait_for_frames()

            depth_frame = frames.get_depth_frame()

            if not depth_frame:
                continue

            # dt1 = datetime.now()
            depth_image = cph.geometry.Image(np.array(depth_frame.get_data()))

            # dt2 = datetime.now()
            temp = cph.geometry.PointCloud.create_from_depth_image(
                depth_image, intrinsic)

            # Apply transformation matrix
            temp.transform(flip_transform)
            pcd.points = temp.points

            # dt3 = datetime.now()
            if frame_count == 0:
                vis.add_geometry(pcd)

            vis.update_geometry(pcd)
            vis.poll_events()
            vis.update_renderer()

            # dt4 = datetime.now()
            # process_time = dt4 - dt0
            # print("FPS: " + str(1 / process_time.total_seconds()))
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
