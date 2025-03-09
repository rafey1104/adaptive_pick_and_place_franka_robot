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

#from src.pickandplace import PickAndPlace

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

            object_pose = (111.60049248, -223.69529814,  147.07774477)

            object_orientation = (0.0, 0.0, 0.0, 1.0)
            # Convert quaternion to rotation matrix
            goal_orientation_matrix = sim.control.quaternion_to_rotation_matrix(object_orientation)
            
            print("Goal orientation roation matrix:", goal_orientation_matrix)

            # Get the initial joint angles from the robot
            initial_joint_angles = sim.robot.get_joint_positions()

            print("Inital joint angles of the robot:", initial_joint_angles)

            # calculating the new joint angles that the franka has to be in to get reach the object location 

            final_joint_angles = sim.control.ik_solver(
                goal_position=np.array(object_pose),
                goal_orientation=goal_orientation_matrix,
                initial_joint_angles=initial_joint_angles,
                max_iterations=100,
                tolerance=1e-3,
                learning_rate=0.1
                )
            
            sim.robot.position_control(final_joint_angles)
            print('Moving to new location.......')
            print("Final joint angles that the franka has to be in are:", final_joint_angles)

            sim.robot.position_control(final_joint_angles)

            # time_to_wait = 5  # seconds
            # start_time = time.time()

            # p.setRealTimeSimulation(1)

            # while time.time() - start_time < time_to_wait:
            #     p.stepSimulation()
            #     time.sleep(0.01)  # Slow down the simulation
    
            #     current_pos, current_ori = sim.robot.get_ee_pose()
            #     print(f"Current end-effector position: {current_pos}")
    
            

            # final_ee_pos, final_ee_ori = sim.robot.get_ee_pose()
            # print("New end-effector position:", final_ee_pos)
            # print("New end-effector orientation (quaternion):", final_ee_ori)


            for i in range(1000):
                sim.step()
                goal_position = np.array([0.65, 0.8, 1.24])

                # Object position
                object_position = np.array([-0.05018395, -0.46971428, 1.4])

                

              

                # print(sim.robot.gripper_idx)
                # print(sim.robot.ee_idx)
                # print(sim.robot.arm_idx)
                # print(sim.robot.default_arm) 
                
                # sim.robot.open_gripper()
                # p.stepSimulation()
                # time.sleep(11) 
                # sim.robot.close_gripper()
                # p.stepSimulation()
                # time.sleep(11) 
                
                                
                # pap = PickAndPlace()

                # # Perform pick and place operations
                # if pap.pick(object_position):
                #     pap.place(goal_position)

                
                # #Validating the Solution
                # ee_pos_ontop_Of_obj = sim.robot.get_ee_pose(ctrl.final_joint_angles)
                # print("End-effector reached position:",  ee_pos_ontop_Of_obj)

                

                ## PLANNING 

                ## CONTROL
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
