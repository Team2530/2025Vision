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

        '''
        the time at which the distance and angle were acquired
        an integer representing the number of frames captured so far
        '''
        self.timestamp = None
    # end __init__
# end class ReefPost



class ReefpostPublisher:
    # NetworkTables.initialize(server="10.25.30.2") #todo -> find actual roborio thing
    # ntTable = NetworkTables.getTable("SmartDashboard")
    # ntTable.putNumberArray("Dists to poles", distArray)
    # ntTable.putNumberArray("Angles to poles", angArray)
    
    def __init__(self):
        pass
    def PublishReefPost(self, reefpost):
        pass
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
        buffer_depth = frame.depth_data/1000.
        buffer_confidence = frame.confidence_data
        return buffer_depth, buffer_confidence
    # end detect_reef_posts
# end class ReefPostDetector




def x2angle(x: float) -> float:
    '''
    Given the horizonta position of a pixel in the image,
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
    r = ReefPostDetector()
    depth, confidence = r.detect_reef_posts()
    depth[confidence<30]=0
    depth[depth>1]=0
    kernel = np.ones((5, 5), np.uint8)  # You can adjust the size and shape
    # depth_filtered = cv2.erode(depth, kernel, iterations=1)
    # depth_filtered = cv2.dilate(depth_filtered, kernel, iterations=1)
    depth_filtered=depth
    a = np.min(depth_filtered[depth_filtered>0])
    b = np.max(depth_filtered[depth_filtered>0])
    depth_uint8 = ((depth-a) / (b-a) * 255 ).astype(np.uint8)
    depth_uint8 = cv2.medianBlur(depth_uint8, 5)
    depth_uint8 = cv2.erode(depth_uint8, kernel, iterations=1)
    # blurred = cv2.GaussianBlur(depth_uint8, (5, 5), 1.5)
    blurred = depth_uint8
    cv2.imshow('Canny Edges', blurred)
    cv2.waitKey(0)
    edges = cv2.Canny(blurred, 100, 200, 11)


    cv2.imshow('Canny Edges', edges)
    cv2.waitKey(0)
    

    # Assume edges and depth arrays are already defined as per your code
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)

    results = []

    center_row = depth.shape[0] // 2

    for cnt in contours:
        area = cv2.contourArea(cnt)         
        if area > 100: 
            continue
        mask = np.zeros(depth.shape, dtype=np.uint8)
        cv2.drawContours(mask, [cnt], -1, color=255, thickness=-1)

        min_depth_points = []

        for row in range(depth.shape[0]):
            masked_row = np.where(mask[row, :] > 0, depth[row, :], 0)
            nonzero_vals = masked_row[masked_row > 0]

            if nonzero_vals.size > 5:
                min_depth = np.min(nonzero_vals)
                min_depth_points.append((row, min_depth))

        if len(min_depth_points) >= 40:  # At least 2 points required for fitting
            rows, depths = zip(*min_depth_points)
            slope, intercept, _, _, _ = stats.linregress(rows, depths)

            # Interpolate depth at the center row
            depth_at_center = slope * center_row + intercept
            results.append({
                'contour': cnt,
                'slope': slope,
                'intercept': intercept,
                'depth_at_center': depth_at_center
            })
        
        cv2.drawContours(mask, [cnt], -1, 255, thickness=cv2.FILLED)
        cv2.imshow('Canny Edges', mask)
        cv2.waitKey(0)

    # Example: print results
    for i, res in enumerate(results):
        print(f"Object {i}: Depth at image center row = {res['depth_at_center']:.2f}")
        

    # Display the edges
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()