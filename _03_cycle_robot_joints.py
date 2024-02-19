
import pybullet as p
import numpy as np

from _03_stand_robots import Robot

if __name__ == "__main__":

    # robot = Robot(robot_name="Go1", fix_base=True)
    # robot = Robot(robot_name="Go2", fix_base=True)
    robot = Robot(robot_name="H1", fix_base=True)
    # robot = Robot(robot_name="H1_hands", fix_base=True)

    dt = 0.002
    time_to_simulate = 100 # seconds
    steps_to_simulate = int(time_to_simulate / dt)

    # Simulate
    for i in range(steps_to_simulate):
        joint_idx = (i // (int(1/dt))) % robot.num_dof
        joint_cycle_time = (i % int(1/dt)) * dt
        pos_offset = np.sin(joint_cycle_time * 2 * np.pi)

        joint_pos_targets = robot.initial_joint_positions.copy()
        joint_pos_targets[joint_idx] += pos_offset

        robot.step_with_pd_targets(joint_pos_targets=joint_pos_targets,
                                   Kp=40.,
                                   Kd=1.,
                                   )
    
    # Disconnect from the simulation
    p.disconnect()
