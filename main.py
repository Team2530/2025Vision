import numpy as np
import ArducamDepthCamera as ac
from scipy import stats
from math import *
import pickle
import ntcore
import cv2
import threading
from flask import Flask, Response
from queue import Queue, Empty
from threading import Thread
import traceback

class ReefPost:
    def __init__(self, distance=0, angle=0):

        '''
        the distance of the center of the cross section of the reefpost 
            on the plane intersecting the aperture and 
            parallel to the ground plane
        relative to the camera, in meters
        
        A double
        '''
        self.distance = distance

        '''
        the angle formed by
            the camera's principal axis and 
            the segment containing 
                the aperture and 
                the center of the cross section of the reefpost 
                    on the plane intersecting the aperture and 
                    parallel to the ground plane
        in radians.

        If the reefpost is on the right side of the camera view,
        the value should be negative.

        If the reefpost is in the center of the camera view,
        the angle should be zero.

        A double
        '''
        self.angle = angle
    # end __init__
# end class ReefPost


class ReefPostPublisher:
    def __init__(self):
        self.counter = 0
        self.ntinst = ntcore.NetworkTableInstance.getDefault()
        self.ntinst.startClient4("coralvision")
        self.ntinst.setServer("10.25.30.2")
        self.nttable = self.ntinst.getTable("CoralVision/raw")
        self.framepub = self.nttable.getIntegerTopic("frame").publish()
        self.anglepub = self.nttable.getDoubleArrayTopic("angles").publish()
        self.distpub = self.nttable.getDoubleArrayTopic("distances").publish()

    def publish_posts(self, posts: list):

        if self.counter % 100 == 0:
            print(self.counter)
            for post in posts:
                print(post.distance, post.angle)

        angles = [x.angle for x in posts]
        distances = [x.distance for x in posts]

        self.anglepub.set(angles)
        self.distpub.set(distances)
        self.framepub.set(self.counter)
        self.counter += 1
        self.ntinst.flush()
        
# end class ReefpostPublisher

class ReefPostDetector:
    def __init__(self):

        def configure_camera():
            cam = ac.ArducamCamera()
            ret = cam.open(ac.Connection.CSI, 0)
            if ret != False:
                print(f"Failed to open camera. Error code {ret}")
                exit(0)
            ret = cam.start(ac.FrameType.DEPTH)
            if ret != False:
                print(f"Failed to open camera. Error code {ret}")
                cam.close()
                exit(0)
            return cam
        # end configure_camera
        self.camera = configure_camera()

        self.time = 0
        pass
    # end __init__

    def get_frame(self):
        self.time = self.time + 1
        frame = self.camera.requestFrame(2000)
        if frame is None or not isinstance(frame, ac.DepthData):
            print(f"Frame {self.time}, dropped")
            return []
        return frame

    def detect_reef_posts(self, frame):
        """
        Returns a list of detected ReefPost objects
        """
        buffer_depth = frame.depth_data / 1000.  # distance from each point in meters
        buffer_confidence = frame.confidence_data

        def horizontal_local_minima(image):
            local_minima = np.ones_like(image, dtype=bool)
            for shift in range(1, 11):
                left_shift = np.roll(image, shift, axis=1)
                right_shift = np.roll(image, -shift, axis=1)
                local_minima &= (image <= left_shift) & (image <= right_shift)
            return local_minima

        # Step 1: scan depth horizontally for local minima.
        buffer_depth_blurred = cv2.GaussianBlur(buffer_depth, (11, 11), 5)
        depth_minima = horizontal_local_minima(buffer_depth_blurred)
        depth_minima &= (buffer_confidence > 30)

        # Step 2: Search the horizontal axis intersecting the principal axis for local minima
        center_row = buffer_depth.shape[0] // 2
        center_pixels_of_minima = np.where(depth_minima[center_row, :])[0]

        def x2angle(x: float) -> float:
            camera = {
                "image_width": 240,
                "image_height": 180,
                "fov_diagonal": radians(70)
            }
            camera["image_diagonal"] = sqrt(camera['image_width'] ** 2 + camera['image_height'] ** 2)
            camera["focal_length"] = (camera["image_diagonal"] / 2) / (tan(camera["fov_diagonal"] / 2))
            x_center = camera["image_width"] / 2 - x
            return atan(x_center / camera["focal_length"])

        def is_valid_post(pixel, depth):
            neighborhood = buffer_depth[max(center_row - 11, 0):center_row + 11,
                                        max(pixel - 11, 0):pixel + 11]
            return not np.all(np.abs(neighborhood - depth) < 0.2)

        # Step 3 & 4 combined with clearer filtering
        reef_posts = [
            ReefPost(distance=buffer_depth[center_row, pixel], angle=x2angle(pixel))
            for pixel in center_pixels_of_minima
            if is_valid_post(pixel, buffer_depth[center_row, pixel])
        ]

        self.camera.releaseFrame(frame)

        return reef_posts
    # end detect_reef_posts
# end class ReefPostDetector

class VideoStream:
    def __init__(self):
        self.app = Flask(__name__)
        self.framebuffer = Queue()
        self.setup_routes()
    # end VideoStream.__init__

    def publish_frame(self, frame):
        _, jpeg = cv2.imencode('.jpg', frame)
        return (b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n')
    # end VideoStream.publish_frame

    def setup_routes(self):
        @self.app.route('/')
        def index():
            return """
            <html>
                <head>
                    <title>Video Stream</title>
                </head>
                <body>
                    <h1>OpenCV Video Stream</h1>
                    <img src=\"/video_feed\">
                </body>
            </html>
            """
        # end 

        @self.app.route('/video_feed')
        def video_feed():
            def generate_frames():
                while True:
                    try:
                        frame = self.framebuffer.get(timeout=1)
                        yield self.publish_frame(frame)
                    except Empty:
                        continue

            return Response(generate_frames(),
                            mimetype='multipart/x-mixed-replace; boundary=frame')
    # end VideoStream.setup_routes

    def start(self):
        thread = Thread(target=self.app.run, kwargs={'debug': False, 'host': '0.0.0.0', 'port': 5000}, daemon=True)
        thread.start()
    # end VideoStream.start
# end class VideoStream




def main():
    print("swaaws")
    detector = ReefPostDetector()
    publisher = ReefPostPublisher() 
    streamer = VideoStream()
    try:
        while True:
            frame = detector.get_frame()
            if not isinstance(frame, ac.Frame):
                continue
            reefposts = detector.detect_reef_posts(frame)
            publisher.publish_posts(reefposts)

            # rendering and serving on port 5000
            buffer_depth = frame.depth_data/1000.     # distance from each point in meters        
            buffer_confidence = frame.confidence_data # confidence data from camera
            preview = np.nan_to_num(buffer_depth)
            preview = (preview * (255.0 / 2)).astype(np.uint8)
            preview = cv2.applyColorMap(preview, cv2.COLORMAP_RAINBOW)
            streamer.publish_frame(preview)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)
        traceback
    finally:
        detector.camera.stop()
        detector.camera.close()
# end main

if __name__ == "__main__":
    main()
