import cv2
import pickle
import numpy as np
from math import *

# with open('lined_up.pickle', 'rb') as handle:
#     p = pickle.load(handle)
#     buffer_depth = p["depth"]
#     buffer_confidence = p["confidence"]

with open('lined_up_depth.npy', 'rb') as f:
    buffer_depth = np.load(f)

with open('lined_up_confidence.npy', 'rb') as f:
    buffer_confidence = np.load(f)

print(buffer_depth.shape)
print(buffer_confidence.shape)

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
cv2.imshow("preview", depth_minima_uint8)
cv2.waitKey(0)

# Step 2: Search the horizontal axis intersecting the principal axis for local minima
center_pixels_of_minima = np.where(depth_minima[buffer_depth.shape[0]//2, :])
print(center_pixels_of_minima)

# Step 3: get depth and angle values of local minima
center_pixel_depth_values = buffer_depth[buffer_depth.shape[0]//2, center_pixels_of_minima][0]
print(center_pixel_depth_values)
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
print(center_pixel_angle_values)

# Step 4: put angles and distances together into ReefPost objects
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
reef_posts = [ReefPost(distance=d, angle=a) for d, a in zip(center_pixel_depth_values, center_pixel_angle_values)]
for post in reef_posts:
    print(post.distance, post.angle)

    
# end magic




def getPreviewRGB(preview: np.ndarray, confidence: np.ndarray) -> np.ndarray:
    preview = np.nan_to_num(preview)
    preview[confidence < 30] = (0, 0, 0)
    return preview

result_image = (buffer_depth_blurred * (255.0 / 4)).astype(np.uint8)
result_image = cv2.applyColorMap(result_image, cv2.COLORMAP_RAINBOW)
result_image = getPreviewRGB(result_image, buffer_confidence)

cv2.imshow("preview", result_image)
cv2.waitKey(0)

