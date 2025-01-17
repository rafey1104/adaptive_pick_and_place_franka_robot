import os
import glob
import yaml
import pybullet as p

import numpy as np

from typing import Dict, Any

from pybullet_object_models import ycb_objects  # type:ignore

from src.simulation import Simulation
from src.perception import Perception as perc

def run_exp(config: Dict[str, Any]):
    # Example Experiment Runner File
    print("Simulation Start:")
    print(config['world_settings'], config['robot_settings'])
    object_root_path = ycb_objects.getDataPath()
    files = glob.glob(os.path.join(object_root_path, "Ycb*"))
    obj_names = [file.split('/')[-1] for file in files]
    sim = Simulation(config)
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
            for i in range(10000):
                sim.step()
                # for getting renders
                # rgb_ee, depth_ee, seg_ee = sim.get_ee_renders()
                # print(seg_ee)
                ## PERCEPTION PIPELINE 
                #Coarse localization using static camera 
                rgb_s, depth_s, seg_s = sim.get_static_renders()
                projection_matrix = np.array(sim.projection_matrix).reshape(4, 4)
                
                object_positions = perc.coarse_localization(rgb_s, depth_s, seg_s, projection_matrix)

                print("Object Positions:", object_positions)

                for obj_id, centroid in object_positions.items():
                    x, y, z = centroid
                    print(f"Object {obj_id}: Centroid at {x:.2f}, {y:.2f}, {z:.2f}")
                    p.addUserDebugText(f"Obj {obj_id}", [x, y, z], textSize=1.5, lifeTime=5)

                
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
