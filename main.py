import numpy as np
import ArducamDepthCamera as ac
from scipy import stats
from math import *
import pickle
import ntcore
import cv2



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

    def publish(self, posts: list):

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


    def detect_reef_posts(self):
        """
        Returns a list of detected ReefPost objects
        """
        self.time = self.time + 1
        frame = self.camera.requestFrame(2000)
        if frame is None or not isinstance(frame, ac.DepthData):
            print(f"Frame {self.time}, dropped")
            return []
        buffer_depth = frame.depth_data/1000.     # distance from each point in meters        
        buffer_confidence = frame.confidence_data # 

        def horizontal_local_minima(image):
            # Initialize local minima as True array
            local_minima = np.ones_like(image, dtype=bool)
            # Check against multiple horizontal neighbors (10 shifts in each direction)
            for shift in range(1, 11):
                left_shift = np.roll(image, shift, axis=1)
                right_shift = np.roll(image, -shift, axis=1)
                local_minima &= (image <= left_shift) & (image <= right_shift)
            return local_minima

        # Step 1: scan depth horizontally for local minima. 
        def horizontal_local_minima(image):
            # Initialize local minima as True array
            local_minima = np.ones_like(image, dtype=bool)
            # Check against multiple horizontal neighbors (10 shifts in each direction)
            for shift in range(1, 11):
                left_shift = np.roll(image, shift, axis=1)
                right_shift = np.roll(image, -shift, axis=1)
                local_minima &= (image <= left_shift) & (image <= right_shift)
            return local_minima

        # Step 1: scan depth horizontally for local minima. 
        # This should generate an image with lines that follow the 
        # center of the reefposts, with possibly other junk we can 
        # filter out later.
        buffer_depth_blurred = cv2.GaussianBlur(buffer_depth, (11, 11), 5)
        depth_minima = horizontal_local_minima(buffer_depth_blurred)
        depth_minima = depth_minima & (buffer_confidence > 30)
        depth_minima_uint8 = depth_minima.astype(np.uint8) * 255

        # Step 2: Search the horizontal axis intersecting the principal axis for local minima
        center_pixels_of_minima = np.where(depth_minima[buffer_depth.shape[0]//2, :])

        # Step 3: get depth and angle values of local minima
        center_pixel_depth_values = buffer_depth[buffer_depth.shape[0]//2, center_pixels_of_minima][0]
        def x2angle(x: float) -> float:
            '''
            Given the horizontal position of a pixel in the image,
            compute the angle in a way that's friendly for the robot
            to digest. This would usually come right before sending
            such a value to the robot via network tables
            '''
            camera = {
                "image_width": 240,
                "image_height": 180,
                "fov_diagonal": radians(70)
            }
            camera["image_diagonal"] = sqrt( camera['image_width']**2 + camera['image_height']**2 )
            camera["focal_length"] = camera["image_diagonal"] / ( 2 * tan(camera["fov_diagonal"]/2) )
            camera["fov_horizontal"] = 2 * atan(camera["image_width"] / (2*camera["focal_length"]))

            x_center = x - camera["image_width"] / 2
            # Negating the result because the robot expects clockwise rotation.
            # In other words, objects to the right to have a negative angle
            return -atan(x_center / camera["fov_horizontal"])
        # end x2angle
        center_pixel_angle_values = [x2angle(i) for i in center_pixels_of_minima[0]]

        # Step 4: put angles and distances together into ReefPost objects
        reef_posts = [ReefPost(distance=d, angle=a) for d, a in zip(center_pixel_depth_values, center_pixel_angle_values)]

        
        self.camera.releaseFrame(frame)
        
        return reef_posts
    # end detect_reef_posts
# end class ReefPostDetector




def x2angle(x: float) -> float:
    '''
    Given the horizontal position of a pixel in the image,
    compute the angle in a way that's friendly for the robot
    to digest. This would usually come right before sending
    such a value to the robot via network tables
    '''
    camera = {
        "image_width": 240,
        "image_height": 180,
        "fov_diagonal": radians(70)
    }
    camera["image_diagonal"] = sqrt( camera['image_width']**2 + camera['image_height']**2 )
    camera["focal_length"] = camera["image_diagonal"] / ( 2 * tan(camera["fov_diagonal"]/2) )
    camera["fov_horizontal"] = 2 * atan(camera["image_width"] / (2*camera["focal_length"]))

    x_center = x - camera["image_width"] / 2
    # Negating the result because the robot expects clockwise rotation.
    # In other words, objects to the right to have a negative angle
    return -atan(x_center / camera["fov_horizontal"])
# end x2angle




def main():
    print("swaaws")
    detector = ReefPostDetector()
    publisher = ReefPostPublisher() 
    while True:
        reefposts = detector.detect_reef_posts()
        publisher.publish(reefposts)
    
    self.camera.stop()
    self.camera.close()
# end main

if __name__ == "__main__":
    main()
