'''
Author: Tawn Kramer
File: realsense2.py
Date: April 14 2019
Notes: Parts to input data from Intel Realsense 2 cameras
'''

import time
import logging

import numpy as np
import pyrealsense2 as rs


class RS_CAM(object):
    '''
    The Intel Realsense product line are smart devices which use sensor fusion and internal processing to output extra
    information, such as 3D-data or location information compared to basic cameras.
    '''

    def __init__(self, image_output):
        self.image_output = image_output
        # Declare RealSense pipeline, encapsulating the actual device and sensors
        self.pipe = rs.pipeline()
        self.cfg = rs.config()
        self.img = None

    def poll(self):
        try:
            frames = self.pipe.wait_for_frames()
        except Exception as e:
            logging.error(e)
            return

        if self.image_output:
            # We will just get one image for now.
            # Empty frame
            frame = None
            self.img = np.asanyarray(frame.get_data())

    def update(self):
        while self.running:
            self.poll()

    def run_threaded(self):
        return self.img

    def run(self):
        self.poll()
        return self.run_threaded()

    def shutdown(self):
        self.running = False
        time.sleep(0.1)
        self.pipe.stop()


class RS_D435(RS_CAM):
    '''
    The Intel Realsense D435 camera is a device which uses an RGB camera and IR light and camera to give color-embedded
    depth data.
    '''

    def __init__(self, image_w, image_h, framerate=60, image_output=True, depth_output=False):
        super().__init__(image_output)
        self.depth_output = depth_output

        # Declare RealSense pipeline, encapsulating the actual device and sensors
        # self.pipe = rs.pipeline()
        # cfg = rs.config()

        if self.depth_output:
            self.cfg.enable_stream(rs.stream.depth)

        if self.image_output:
            # Check that the image size and FPS are allowed for this camera
            fps_opts = [6, 15, 30, 60]
            resolution_opts = [[320, 180], [320, 240], [424, 240], [640, 360],
                               [640, 480], [848, 480], [960, 540], [1280, 720], [1920, 1080]]
            # color_opts = ['YUYV', 'BGR8', 'RGBA8', 'BGRA8', 'Y16', 'RGB8', 'RAW16']

            resolution = [image_w, image_h]
            if resolution not in resolution_opts:
                print("Image size '{}' is not allowed for this camera!".format(resolution))
                print("The allowed sizes are: ")
                for sz in resolution_opts:
                    print(sz)
                raise(Exception())

            if framerate not in fps_opts:
                print("Framerate {} is not allowed for this camera!".format(framerate))
                print("The allowed framerates are: {}".format(fps_opts))
                raise(Exception())

            # Enable RGB stream
            self.cfg.enable_stream(rs.stream.color, image_w, image_h, rs.format.rgb8, framerate)

        # Start streaming with requested config
        self.pipe.start(self.cfg)
        self.running = True
        self.depth = None

    def poll(self):
        try:
            frames = self.pipe.wait_for_frames()
        except Exception as e:
            logging.error(e)
            return

        if self.image_output:
            # Get RGB image
            rgb_frame = frames.get_color_frame()
            self.img = np.asanyarray(rgb_frame.get_data())

        if self.depth_output:
            # Fetch depth frame
            depth_frame = frames.get_depth_frame()
            self.depth = np.asanyarray(depth_frame.get_data())

    def run_threaded(self):
        if self.image_output and self.depth_output:
            return self.img, self.depth
        if self.image_output:
            return self.img
        if self.depth_output:
            return self.depth


class RS_T265(RS_CAM):
    '''
    The Intel Realsense T265 camera is a device which uses an imu, twin fisheye cameras,
    and an Movidius chip to do sensor fusion and emit a world space coordinate frame that
    is remarkably consistent.
    '''

    def __init__(self, image_output=True, pose_output=False):
        super().__init__(image_output)

        self.pose_output = pose_output

        # # Declare RealSense pipeline, encapsulating the actual device and sensors
        # self.pipe = rs.pipeline()
        # self.cfg = rs.config()
        
        if self.pose_output:
            self.cfg.enable_stream(rs.stream.pose)

        # Using the image_output will grab two image streams from the fisheye cameras but return only one.
        # This can be a bit much for USB2, but you can try it. Docs recommend USB3 connection for this.
        if self.image_output:
            # Right now it's required for both streams to be enabled
            self.cfg.enable_stream(rs.stream.fisheye, 1)  # Left camera
            self.cfg.enable_stream(rs.stream.fisheye, 2)  # Right camera

        # Start streaming with requested config
        self.pipe.start(self.cfg)
        self.running = True
        
        zero_vec = (0.0, 0.0, 0.0)
        self.pos = zero_vec
        self.vel = zero_vec
        self.acc = zero_vec

    def poll(self):
        try:
            frames = self.pipe.wait_for_frames()
        except Exception as e:
            logging.error(e)
            return

        if self.image_output:
            # We will just get one image for now.
            # Left fisheye camera frame
            left = frames.get_fisheye_frame(1)
            self.img = np.asanyarray(left.get_data())

        pose = None
        if self.pose_output:
            # Fetch pose frame
            pose = frames.get_pose_frame()

        if pose:
            data = pose.get_pose_data()
            self.pos = data.translation
            self.vel = data.velocity
            self.acc = data.acceleration
            logging.debug('realsense pos(%f, %f, %f)' % (self.pos.x, self.pos.y, self.pos.z))

    def run_threaded(self):
        if self.image_output and self.pose_output:
            return self.pos, self.vel, self.acc, self.img
        if self.image_output:
            return self.img
        if self.pose_output:
            return self.pos, self.vel, self.acc


if __name__ == "__main__":
    c = RS_T265()
    while True:
        pos, vel, acc = c.run()
        print(pos)
        time.sleep(0.1)
    c.shutdown()
