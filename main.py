from networktables import NetworkTables
import cv2
import numpy as np
import ArducamDepthCamera as ac
from scipy import stats
from math import *




class ReefPost:
    def __init__(self):

        '''
        the distance of the center of the cross section of the reefpost 
            on the plane intersecting the aperture and 
            parallel to the ground plane
        relative to the camera, in meters
        
        A double
        '''
        self.distance = None

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
        self.angle = None
    # end __init__
# end class ReefPost


class ReefPostPublisher:
    def __init__(self):
        self.counter = 0
        self.NetworkTables.initialize(server="10.25.30.2") #todo -> find actual roborio thing
        self.ntTable = NetworkTables.getTable("SmartDashboard")

    def publish(self, posts: list):
        angles = [x.angle for x in posts]
        distances = [x.distance for x in posts]

        self.ntTable.putNumberArray("CoralVision/raw/angles", angles)
        self.ntTable.putNumberArray("CoralVision/raw/distances", distances)
        self.ntTable.putNumber("CoralVision/raw/frame", self.counter)
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

        # magic for detecting reef posts        
        reef_posts = []

        # Step 1: scan depth horizontally for local minima. 
        # This should generate an image with lines that follow the 
        # center of the reefposts, with possibly other junk we can 
        # filter out later.
        buffer_depth_blurred = cv2.GaussianBlur(buffer_depth, (5, 5), 1.5)

        # Step 2: search for lines with Hough Lines? Or contour detection?
        # contour detection might be easier to process
        
        # Step 3: filter contours by length and height/width ratio

        # Step 4: for each contour, fit a line of depth values to interpolate
        

        # end magic

        
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
    detector = ReefPostDetector()
    publisher = ReefPostPublisher() 
    while True:
        reefposts = detector.detect_reef_posts()
        publisher.publish(reefposts)
# end main

if __name__ == "__main__":
    main()