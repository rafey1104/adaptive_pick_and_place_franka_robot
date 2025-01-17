import pybullet as p
import numpy as np 
from src.utils import pb_image_to_numpy

from src.robot import Robot
from src.simulation import Simulation
from src.utils import depth_to_point_cloud

class Perception:
    
    def coarse_localization(rgb, depth, seg, projection_matrix):
    # """
    # Perform coarse object localization using RGB-D and segmentation data.
    # Args:
    #     rgb (np.ndarray): RGB image.
    #     depth (np.ndarray): Depth image.
    #     seg (np.ndarray): Segmentation mask.
    #     projection_matrix (np.ndarray): Camera projection matrix.
    # Returns:
    #     object_positions (dict): Dictionary mapping object IDs to centroids.
    # """
    
        unique_ids = np.unique(seg)
        unique_ids = unique_ids[unique_ids != -1]  # Exclude background (-1)

        object_positions = {}
        
        for obj_id in unique_ids:
            mask = (seg == obj_id)
            point_cloud = depth_to_point_cloud(depth, projection_matrix, mask)
            centroid = point_cloud.mean(axis=0)
            object_positions[obj_id] = centroid

        return object_positions
    
    def refine_localization() #ICP algo
        return 


