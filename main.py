import os
import glob
import yaml
import time
import pybullet as p

from pyntcloud import PyntCloud

import open3d as o3d
import numpy as np

from typing import Dict, Any

from pybullet_object_models import ycb_objects  # type:ignore

# #import for megapose6d
# from megapose.datasets.object_dataset import RigidObjectDataset
# from megapose.inference.pose_estimator import PoseEstimator
# from megapose.inference.types import DetectionsType, PoseEstimatesType
# from megapose.lib3d.transform import Transform


from src.simulation import Simulation
from src.perception import Perception as perc
from src.control import Control
from src.robot import Robot

TABLE_SCALING = 2.0

from src.pickandplace import pick_and_place

# # Load the robot settings from the YAML configuration
# def load_robot_settings(config_path: str) -> Dict[str, Any]:
#     with open(config_path, 'r') as file:
#         config = yaml.safe_load(file)  # Load the YAML
#         print("Loaded config:", config)  # Debug: print the loaded configuration
#         return config.get('robot_settings', {})  # Extract robot settings

# robot_settings = load_robot_settings('configs/test_config.yaml')
# class PickAndPlace:
#     def __init__(self, robot, control):
#         self.control = Control()
#         self.robot = Robot(urdf=robot_settings.get("urdf", "default.urdf"),
#             init_position=robot_settings.get("default_init_pos", [0, 0, 0]),
#             orientation=robot_settings.get("default_init_ori", [0, 0, 0]),
#             arm_index=robot_settings.get("arm_idx", []),
#             gripper_index=robot_settings.get("gripper_idx", []),
#             ee_index=robot_settings.get("ee_idx", 0),
#             arm_default=robot_settings.get("default_arm", []),
#             table_scaling=robot_settings.get("table_scaling", TABLE_SCALING))
        
#     def pick(self, object_position):
#         print("Starting pick operation...")
#         pre_grasp_position = object_position + np.array([0, 0, 0.1])
#         pre_grasp_orientation = np.array([
#             [1, 0, 0],
#             [0, 0, -1],
#             [0, 1, 0]
#         ]) 
#         initial_joints = self.robot.get_joint_positions()
        
#         final_joints = self.control.ik_solver(
#             pre_grasp_position,
#             pre_grasp_orientation,
#             initial_joints,
#             self.robot.lower_limits,
#             self.robot.upper_limits
#         )
        
#         if final_joints is None:
#             print("IK Solver failed to find a solution.")
#             return

#         print("Final Joint Angles:", final_joints)

#         # Step 4: Generate Interpolated Trajectory
#         trajectory = np.linspace(initial_joint_angles, final_joint_angles, num_steps)

#         # Step 5: Move Robot Step by Step
#         for step, joint_angles in enumerate(trajectory):
#             self.robot.position_control(joint_angles)  # Send angles to robot
#             print(f"Step {step+1}/{num_steps}: Moving to {joint_angles}")
#             time.sleep(0.05)  # Small delay to simulate smooth movement

#         print("Reached goal position!")
        
#         self.robot.position_control(final_joints)
#         self.robot.open_gripper()
        
#         final_joints = self.control.ik_solver(
#             object_position,
#             np.eye(3),
#             final_joints
#         )
#         if final_joints is None: return False
        
#         self.robot.position_control(final_joints)
#         self.robot.close_gripper()
        
#         post_grasp_position = object_position + np.array([0, 0, 0.1])
        
#         final_joints = self.control.ik_solver(
#             goal_position,
#             goal_orientation,
#             initial_joint_angles,
#             self.robot.lower_limits,
#             self.robot.upper_limits
#         )

#         print(final_joints)
#         if final_joints is None: return False
        
#         self.robot.position_control(final_joints)
#         return True

#     def place(self, goal_position):
#         print("Starting place operation...")
#         pre_place_position = goal_position + np.array([0, 0, 0.1])
#         initial_joints = self.robot.get_joint_positions()
        
#         final_joints = self.control.ik_solver(
#             pre_place_position,
#             np.eye(3),
#             initial_joints
#         )
#         if final_joints is None: return False
        
#         self.robot.position_control(final_joints)
        
#         final_joints = self.control.ik_solver(
#             goal_position,
#             np.eye(3),
#             final_joints
#         )
#         if final_joints is None: return False
        
#         self.robot.position_control(final_joints)
#         self.robot.open_gripper()
#         return True

def run_exp(config: Dict[str, Any]):
    # Example Experiment Runner File
    print("Simulation Start:")
    print(config['world_settings'], config['robot_settings'])
    object_root_path = ycb_objects.getDataPath()
    files = glob.glob(os.path.join(object_root_path, "Ycb*"))
    obj_names = [file.split('/')[-1] for file in files]
    sim = Simulation(config)
    projection_matrix = np.array(sim.projection_matrix).reshape(4, 4)
    width, height = 480, 640 # from image dimensions
    fx = projection_matrix[0, 0]
    fy = projection_matrix[1, 1]
    cx = projection_matrix[0, 2]
    cy = projection_matrix[1, 2]
    ##megapose implementation 
    # Load MegaPose model
    #pose_estimator = PoseEstimator()
    

    for obj_name in obj_names:
        for tstep in range(10):
            sim.reset(obj_name)
            print((f"Object: {obj_name}, Timestep: {tstep},"
                   f" pose: {sim.get_ground_tuth_position_object}"))
            pos, ori = sim.robot.pos, sim.robot.ori
            print(f"Robot inital pos: {pos} orientation: {ori}")
            l_lim, u_lim = sim.robot.lower_limits, sim.robot.upper_limits
            print(f"Robot Joint Range {l_lim} -> {u_lim}")
            sim.robot.print_joint_infos()
            jpos = sim.robot.get_joint_positions()
            print(f"Robot current Joint Positions: {jpos}")
            jvel = sim.robot.get_joint_velocites()
            print(f"Robot current Joint Velocites: {jvel}")
            ee_pos, ee_ori = sim.robot.get_ee_pose()
            print(f"Robot End Effector Position: {ee_pos}")
            print(f"Robot End Effector Orientation: {ee_ori}")
            print("Performing pick and place for:", obj_name)

            # object_pose = (111.60049248, -223.69529814,  147.07774477)

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
            # print('Moving to new location.......')
            # print("Final joint angles that the franka has to be in are:", final_joint_angles)

            # sim.robot.position_control(final_joint_angles)

            # time_to_wait = 5  # seconds
            # start_time = time.time()

            # p.setRealTimeSimulation(1)

            # while time.time() - start_time < time_to_wait:
            #     p.stepSimulation()
            #     time.sleep(0.01)  # Slow down the simulation
    
            #     current_pos, current_ori = sim.robot.get_ee_pose()
            #     print(f"Current end-effector position: {current_pos}")
             # # Perform pick-and-place for the current object
            object_position = np.array([0.0, -0.425, 0.5])  # Object position
            target_position = np.array([0.55, 0.58, 0.43])  # Target position
            pick_and_place(sim, object_position, target_position)
            

          
             
            obs_position_guess = np.zeros((2, 3))
            print((f"[{i}] Obstacle Position-Diff: "
                f"{sim.check_obstacle_position(obs_position_guess)}"))
            goal_guess = np.zeros((7,))
            print((f"[{i}] Goal Obj Pos-Diff: "
                f"{sim.check_goal_obj_pos(goal_guess)}"))
            print(f"[{i}] Goal Satisfied: {sim.check_goal()}")
    sim.close()


if __name__ == "__main__":
    with open("configs/test_config.yaml", "r") as stream:
        try:
            config = yaml.safe_load(stream)
            print(config)
        except yaml.YAMLError as exc:
            print(exc)
    run_exp(config)
