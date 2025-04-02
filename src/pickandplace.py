#****** RAFEY's Pick and Place Code******#

# import numpy as np
# from scipy.spatial.transform import Rotation as R

# from control import Control 
# from robot import Robot

# class PickAndPlace:
#     def __init__(self):
#         # Define robot parameters (replace with your actual robot's parameters)
#         self.link_lengths = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]  # Example
#         self.joint_limits = [(-2.9671, 2.9671), (-1.8326, 1.8326), (-2.9671, 2.9671),
#                              (-3.1416, 0.0), (-2.9671, 2.9671), (-0.0873, 3.8223),
#                              (-2.9671, 2.9671)]  # Example

#         # Gripper parameters (replace with your actual gripper's parameters)
#         self.gripper_open_value = 0.04
#         self.gripper_closed_value = 0.0

#         # Define table and object dimensions (example values)
       
#         self.control = Control()
#         self.robot = Robot()



#     def pick(self, object_pose):
#         """
#         Placeholder: Implements the pick operation.
#         This function defines the sequence of actions needed to pick up the object.
#         """
#         print("Starting pick operation...")

#         # 1. Pre-grasp approach (move above the object)
#         pre_grasp_position = object_pose[:3] + [0, 0, 0.1]  # Approach from above (10cm)
#         pre_grasp_orientation = R.from_euler('z', 0)
#         initial_joints = self.robot.get_joint_positions()  # Initial guess for joint angles
#         joint_angles = self.control.ik_solver(pre_grasp_position, pre_grasp_orientation, initial_joints)
#         if joint_angles is None:
#             print("IK failed for pre-grasp position")
#             return False
#         self.robot.position_control(joint_angles) #Move robot to the joint angles

#         # 2. Open the gripper
#         self.robot.open_gripper()

#         # 3. Move to grasp position
#         grasp_position = object_pose[:3]
#         grasp_orientation = R.from_euler('z', 0) #Adjust rotation as needed
#         joint_angles = self.control.ik_solver(grasp_position, grasp_orientation, joint_angles)
#         if joint_angles is None:
#             print("IK failed for grasp position")
#             return False
#         self.robot.position_control(joint_angles) #Move robot to the joint angles

#         # 4. Close the gripper
#         self.robot.close_gripper()

#         # 5. Post-grasp lift (move up with the object)
#         post_grasp_position = object_pose[:3] + [0, 0, 0.1]
#         post_grasp_orientation = R.from_euler('z', 0)
#         joint_angles = self.control.ik_solver(post_grasp_position, post_grasp_orientation, joint_angles)
#         if joint_angles is None:
#             print("IK failed for post-grasp position")
#             return False
#         self.robot.position_control(joint_angles) #Move robot to the joint angles

#         print("Pick operation complete.")
#         return True

#     def place(self, place_pose):
#         """
#         Placeholder: Implements the place operation.
#         This function defines the sequence of actions needed to place the object at the goal location.
#         """
#         print("Starting place operation...")

#         # 1. Pre-place approach (move above the place location)
#         pre_place_position = place_pose[:3] + [0, 0, 0.1]  # Approach from above (10cm)
#         pre_place_orientation = R.from_euler('z', 0)
#         initial_joints = [0, 0, 0, 0, 0, 0, 0]  # Initial guess for joint angles
#         joint_angles = self.control.ik_solver(pre_place_position, pre_place_orientation, initial_joints)
#         if joint_angles is None:
#             print("IK failed for pre-place position")
#             return False
#         self.move_to_joint_angles(joint_angles) #Move robot to the joint angles

#         # 2. Move to place position
#         place_position = place_pose[:3]
#         place_orientation = R.from_euler('z', 0)
#         joint_angles = self.inverse_kinematics(place_position, place_orientation, joint_angles)
#         if joint_angles is None:
#             print("IK failed for place position")
#             return False
#         self.robot.position_control(joint_angles) #Move robot to the joint angles

#         # 3. Open the gripper
#         self.robot.open_gripper()

#         # 4. Post-place retreat (move up after releasing the object)
#         post_place_position = place_pose[:3] + [0, 0, 0.1]
#         post_place_orientation =  R.from_euler('z', 0)
#         joint_angles = self.inverse_kinematics(post_place_position, post_place_orientation, joint_angles)
#         if joint_angles is None:
#             print("IK failed for post-place position")
#             return False
#         self.robot.position_control(joint_angles) #Move robot to the joint angles

#         print("Place operation complete.")
#         return True

# def main():
#     # Initialize the PickAndPlace instance
#     pick_and_place = PickAndPlace()

#     # Define object and goal poses
#     object_pose = (-0.050183952461055004, -0.46971427743603356, 1.3231258620680433)
#     object_orientation = (0.0, 0.0, 0.0, 1.0)
#     # goal_pose = (0.65, 0.8, 1.24)
#     # goal_orientation = (-0.0, -0.0, 0.7071067811865475, 0.7071067811865476)

   

#     # # Perform pick operation
#     # print("\nExecuting pick operation...")
#     # pick_success = pick_and_place.pick(object_pose, object_orientation)

#     # if pick_success:
#     #     # Perform place operation
#     #     print("\nExecuting place operation...")
#     #     place_success = pick_and_place.place(goal_pose, goal_orientation)

#     #     if place_success:
#     #         print("\nPick and place task completed successfully!")
#     #     else:
#     #         print("\nPlace operation failed.")
#     # else:
#     #     print("\nPick operation failed.")

# if __name__ == "__main__":
#     main()

# object_pose = (296.22743805,  61.04207592,   0.99762923)

# object_orientation = (0.0, 0.0, 0.0, 1.0)
# # Convert quaternion to rotation matrix
# goal_orientation_matrix = sim.control.quaternion_to_rotation_matrix(object_orientation)
# print("Goal orientation roation matrix:", goal_orientation_matrix)

# # Get the initial joint angles from the robot
# initial_joint_angles = sim.robot.get_joint_positions()

# print("Inital joint angles of the robot:", initial_joint_angles)

# # calculating the new joint angles that the franka has to be in to get reach the object location 

# final_joint_angles = sim.control.ik_solver(
#     goal_position=np.array(object_pose),
#     goal_orientation=goal_orientation_matrix,
#     initial_joint_angles=initial_joint_angles,
#     max_iterations=100,
#     tolerance=1e-3,
#     learning_rate=0.1
#     )

# sim.robot.position_control(final_joint_angles)


import os
import glob
import yaml
import numpy as np
import pybullet as p
import open3d as o3d
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from typing import Sequence, Tuple


from typing import Dict, Any
from pybullet_object_models import ycb_objects  # type:ignore

from src.simulation import Simulation as sim


def move_down_to_object(self, goal_position, initial_joint_angles, step_size=0.01, max_steps=100):
    """
    Move the end-effector downwards to the object's position.
    """
    current_joint_angles = initial_joint_angles.copy()
    for step in range(max_steps):
        # Get current end-effector position
        ee_pos, _ = self.get_ee_pose(current_joint_angles)

        # Check if the end-effector is close to the goal position
        if np.linalg.norm(ee_pos - goal_position) < step_size:
            print("Reached the object position.")
            break

        # Move downwards by a small step
        goal_position[2] -= step_size  # Move downwards in the z-axis

        # Solve IK for the new goal position
        current_joint_angles = self.ik_solver(
            goal_position=goal_position,
            initial_joint_angles=current_joint_angles,
            max_iterations=10,
            tolerance=1e-3,
            learning_rate=0.1
        )

        # Move the robot to the new joint angles
        self.robot.position_control(current_joint_angles)

        # Step the simulation
        for _ in range(10):  # Adjust the number of steps as needed
            p.stepSimulation()



def pick_object(self, object_position, initial_joint_angles):
    """
    Pick the object by opening the gripper, moving down, closing the gripper, and moving back up.
    """
    # Step 1: Open the gripper
    print("Opening the gripper...")
    self.robot.open_gripper()
    for _ in range(100):  # Wait for the gripper to open
        p.stepSimulation()

    # Step 2: Move to a position slightly above the object
    print("Moving to the object...")
    goal_position = np.array(object_position) + np.array([0, 0, 0.1])  # 10 cm above the object
    final_joint_angles = self.ik_solver(
        goal_position=goal_position,
        initial_joint_angles=initial_joint_angles,
        max_iterations=100,
        tolerance=1e-3,
        learning_rate=0.1
    )
    self.robot.position_control(final_joint_angles)
    for _ in range(100):  # Wait for the robot to reach the position
        p.stepSimulation()

    # Step 3: Move down to the object
    print("Moving down to the object...")
    self.move_down_to_object(
        goal_position=np.array(object_position),
        initial_joint_angles=final_joint_angles
    )

    # Step 4: Close the gripper
    print("Closing the gripper...")
    self.robot.close_gripper()
    for _ in range(100):  # Wait for the gripper to close
        p.stepSimulation()

    # Step 5: Move back up
    print("Moving back up...")
    goal_position = np.array(object_position) + np.array([0, 0, 0.1])  # 10 cm above the object
    final_joint_angles = self.ik_solver(
        goal_position=goal_position,
        initial_joint_angles=self.robot.get_joint_positions(),
        max_iterations=50,    ## reduced from 100 to 50 because program freezes after 47 iterations
        tolerance=1e-3,
        learning_rate=0.1
    )
   
    self.robot.position_control(final_joint_angles)
    for _ in range(100):  # Wait for the robot to move back up
        p.stepSimulation()

    print("Object picked successfully.")


def pick_and_place(sim, object_position, target_position):
    """
    Perform a pick-and-place operation:
    1. Open the gripper.
    2. Move to the object.
    3. Access the ee camera images and create point cloud images and sample the grasp on the point cloud 
    4. Detect the object using yolo11
    5. Move down to the object
    6. Close the gripper.
    7. Move back up.
    8. Move to the target position.
    9. Move down to the target position 
    10. Open the gripper to release the object.
    """
    # Define the desired orientation (identity quaternion: no rotation)
    desired_quaternion = np.array([1.0, 0.0, 0.0, 0.0])  # Quaternion (w, x, y, z)
    desired_orientation = sim.control.quaternion_to_rotation_matrix(desired_quaternion)  # Convert to rotation matrix

    # Step 1: Open the gripper
    print("Opening the gripper...")
    sim.robot.open_gripper()
    for _ in range(100):  # Wait for the gripper to open
        sim.step()

    # Step 2: Move to the object (slightly above it)
    print("Moving to the object...")
    goal_position = np.array(object_position) + np.array([0, 0, 0.1])  # 10 cm above the object
    initial_joint_angles = sim.robot.get_joint_positions()
    final_joint_angles = sim.control.ik_solver(
        goal_position=goal_position,
        goal_orientation=desired_orientation,  # Pass the rotation matrix
        initial_joint_angles=initial_joint_angles,
        max_iterations=100,
        tolerance=1e-3,
        learning_rate=0.1
    )
    sim.robot.position_control(final_joint_angles)
    for _ in range(200):  # Wait for the robot to reach the position
        sim.step()

    # Step 3: Access the ee camera images and create point cloud images and sample the grasp on the point cloud 
    print("Capturing EE camera images...")
    # _, depth_image, _ = self.sim.capture_ee_images()
    rgb_image, depth_image, segmentation_mask = sim.get_ee_renders()
    
    # print(depth_image)
    # # Camera parameters
    # fx = 640 / (2.0 * np.tan(np.radians(70 / 2.0)))  # Assume FOV = 70 degrees
    # fy = fx  # Assuming square pixels
    # cx, cy = 640 / 2, 480 / 2  # Principal point

    # # Create intrinsic matrix
    # intrinsic = o3d.camera.PinholeCameraIntrinsic()
    # intrinsic.set_intrinsics(640, 480, fx, fy, cx, cy)

    # # Convert depth image to Open3D format
    # depth_image_o3d = o3d.geometry.Image(depth_image)

    # print(depth_image_o3d)

    # # Generate point cloud from depth image
    # point_cloud = o3d.geometry.PointCloud.create_from_depth_image(
    #     depth_image_o3d,
    #     intrinsic,
    #     depth_scale=1000.0,  # Adjust based on your depth image scale
    #     depth_trunc=5.0   # Truncate distances beyond this value
    # )

    # print(point_cloud)
    # print(np.asarray(point_cloud.points))
    # point_cloud.paint_uniform_color([1, 0, 0])  # Red color
    # o3d.visualization.draw_geometries([point_cloud])
    # Camera parameters

    ee_pos, ee_ori = sim.robot.get_ee_pose() 

    # Apply offset to calculate camera position
    ee_cam_offset = [0.0, 0.0, 0.1]  # Offset from configuration
    cam_pos = [
        ee_pos[0] + ee_cam_offset[0],
        ee_pos[1] + ee_cam_offset[1],
        ee_pos[2] + ee_cam_offset[2]
    ]        
    
    # Use end-effector orientation for camera alignment
    rotation_matrix = np.array(p.getMatrixFromQuaternion(ee_ori)).reshape(3, 3)

    # Camera target direction (forward vector)
    cam_forward = rotation_matrix[:, 2]
    cam_target = [
        cam_pos[0] + cam_forward[0],
        cam_pos[1] + cam_forward[1],
        cam_pos[2] + cam_forward[2]
    ]
   
    # Camera up direction (Y-axis)
    cam_up = rotation_matrix[:, 1]

    fov = 70                       # Field of view (degrees)
    aspect = 640 / 480             # Aspect ratio (width/height)
    near = 0.01                     # Near clipping plane
    far = 5.0                      # Far clipping plane
    width, height = 640, 480
    focal_length = width / (2 * np.tan(np.deg2rad(fov) / 2))

    # Compute view and projection matrices
    view_mat = p.computeViewMatrix(cam_pos, cam_target, cam_up)
    proj_mat = p.computeProjectionMatrixFOV(fov, aspect, near, far)

    # Get camera image (returns RGB, depth, segmentation)
    img = p.getCameraImage(width, height, view_mat, proj_mat)

    # Extract depth buffer (img[3] = depth buffer)
    
    rgb_image = img[2]  # Shape: (height, width, 4) (RGBA format)
    
    # # Display the image using matplotlib
    # plt.imshow(seg_s)
    # plt.axis('off')  # Hide axes
    # plt.show()


    depth_buffer = img[3]  # Shape: (height, width)

    # Convert depth buffer to actual distances (linearize if needed)
    # PyBullet's depth buffer is in [0, 1], where 1 = far plane, 0 = near plane
    depth = far * near / (far - (far - near) * depth_buffer)
    print(f"Depth value: {np.min(depth_image):.3f} to {np.max(depth_image):.3f}")

    # For visualization, we invert and normalize
    depth_image = 1 - depth_buffer  # Invert for better visualization (optional)
    depth_image = (depth_image - np.min(depth_image)) / (np.max(depth_image) - np.min(depth_image))  # Normalize to [0, 1]
    print(f"Depth image value: {np.min(depth_image):.3f} to {np.max(depth_image):.3f}")

    # # Display the depth image
    # plt.imshow(depth_image, cmap='gray')  # Grayscale for depth
    # plt.colorbar(label='Depth (0=near, 1=far)')
    # plt.title("Depth Image")
    # plt.axis('off')
    # plt.show()

    ## To create a pointcloud from depth_s:
    rgb_image = img[2][:, :, :3]  # Extract RGB

    # Convert depth buffer to actual distances
    depth = far * near / (far - (far - near) * depth_buffer)

    # Generate 3D points
    fx, fy = focal_length, focal_length
    cx, cy = width / 2, height / 2

    points = []
    colors = []

    for v in range(height):
        for u in range(width):
            Z = depth[v, u]
            if Z > 0:  # Ignore invalid depths
                X = (u - cx) * Z / fx
                Y = (v - cy) * Z / fy
                points.append([X, Y, Z])

                # Attach RGB color (if needed)
                colors.append(rgb_image[v, u] / 255.0)  # Normalize RGB to [0,1]

    # Convert to NumPy array
    points = np.array(points)
    colors = np.array(colors)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors)
    # o3d.visualization.draw_geometries([pcd])
    
    #***Sampling the grasp around the point cloud***#
    object_position = np.array([0.0, -0.425, 0.5])  # Object's position
    num_grasps = 10  # Number of grasps to sample
    offset = 0.1  # Maximum sampling distance around the object

    grasp_poses_list = []
    for _ in range(num_grasps):
        # Sample a random translation within the offset sphere
        random_direction = np.random.uniform(-1, 1, 3)
        random_direction /= np.linalg.norm(random_direction)  # Normalize
        random_magnitude = np.random.uniform(0, offset)
        translation = object_position + random_direction * random_magnitude

        # Generate a random rotation
        random_rotation = R.random().as_matrix()

        assert random_rotation.shape == (3, 3)
        assert translation.shape == (3,)
        grasp_poses_list.append((random_rotation, translation))
    
    for i, (rotation, translation) in enumerate(grasp_poses_list):
        print(f"Grasp {i+1}:")
        print(f"Translation: {translation}")
        print(f"Rotation Matrix:\n{rotation}\n")
    
    if grasp_poses_list is None:
        print("No valid grasp found!")
        return False

    return
    # Step 4: Detect the object using yolo11

    # Step 5: Move down to the object
    print("Moving down to the object...")
    goal_position = np.array(object_position)  # Move to the object's exact position
    final_joint_angles = sim.control.ik_solver(
        goal_position=goal_position,
        goal_orientation=desired_orientation,  # Pass the rotation matrix
        initial_joint_angles=final_joint_angles,
        max_iterations=100,
        tolerance=1e-3,
        learning_rate=0.1
    )
    sim.robot.position_control(final_joint_angles)
    for _ in range(200):  # Wait for the robot to reach the position
        sim.step()

    # Step 6: Close the gripper
    print("Closing the gripper...")
    sim.robot.close_gripper()
    for _ in range(200):  # Wait for the gripper to close
        sim.step()

    # Step 7: Move back up
    print("Moving back up...")
    goal_position = np.array(object_position) + np.array([0, 0, 0.1])  # 10 cm above the object
    final_joint_angles = sim.control.ik_solver(
        goal_position=goal_position,
        goal_orientation=desired_orientation,  # Pass the rotation matrix
        initial_joint_angles=sim.robot.get_joint_positions(),
        max_iterations=100,
        tolerance=1e-3,
        learning_rate=0.1
    )
    sim.robot.position_control(final_joint_angles)
    for _ in range(200):  # Wait for the robot to move back up
        sim.step()

    # Step 8: Move to the target position
    print("Moving to the target position...")
    goal_position = np.array(target_position) + np.array([0, 0, 0.1])  # 10 cm above the target
    final_joint_angles = sim.control.ik_solver(
        goal_position=goal_position,
        goal_orientation=desired_orientation,  # Pass the rotation matrix
        initial_joint_angles=sim.robot.get_joint_positions(),
        max_iterations=100,
        tolerance=1e-3,
        learning_rate=0.1
    )
    sim.robot.position_control(final_joint_angles)
    for _ in range(200):  # Wait for the robot to reach the target position
        sim.step()

    # Step 9: Move down to the target position
    print("Moving down to the target position...")
    goal_position = np.array(target_position)  # Move to the target's exact position
    final_joint_angles = sim.control.ik_solver(
        goal_position=goal_position,
        goal_orientation=desired_orientation,  # Pass the rotation matrix
        initial_joint_angles=final_joint_angles,
        max_iterations=100,
        tolerance=1e-3,
        learning_rate=0.1
    )
    sim.robot.position_control(final_joint_angles)
    for _ in range(200):  # Wait for the robot to reach the target position
        sim.step()

    # Step 10: Open the gripper to release the object
    print("Opening the gripper to release the object...")
    sim.robot.open_gripper()
    for _ in range(200):  # Wait for the gripper to open
        sim.step()

    print("Pick-and-place operation completed.")


