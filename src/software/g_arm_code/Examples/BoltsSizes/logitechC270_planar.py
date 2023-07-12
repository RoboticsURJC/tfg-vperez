import numpy as np

# Define camera parameters
focal_length = 840  
image_width = 1280
image_height = 720
camera_matrix = np.array([[focal_length, 0, image_width/2],
                          [0, focal_length, image_height/2],
                          [0, 0, 1]])

# Define plane parameters
plane_normal = np.array([0, 0, 1])  # assuming plane is parallel to the ground

# Cordinate system relative to de camera, Z is depth
def get_3d_point_from_2d_point(pixel, planeDistance):
    u, v = pixel
    # Construct a ray from camera center through 2D point
    ray = np.linalg.inv(camera_matrix) @ np.array([u, v, 1]) # Multiply matrix
    ray /= np.linalg.norm(ray)

    # Calculate distance from camera center to plane
    t = (planeDistance - np.array([0, 0, 0]).dot(plane_normal)) / ray.dot(plane_normal)

    # Calculate 3D point on plane
    point_3d = np.array([0, 0, 0]) + t * ray

    return point_3d
