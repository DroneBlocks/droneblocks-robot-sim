
import pybullet as p
import numpy as np

from _03_stand_robots import Robot

if __name__ == "__main__":

    robot = Robot(robot_name="H1_hands", fix_base=True)

    # add a box to the simulation
    cubeId = p.loadURDF("assets/box.urdf", basePosition=[0.4, 0., 1.5], baseOrientation=p.getQuaternionFromEuler([0, 0, 0]))

    dt = 0.002
    time_to_simulate = 20 # seconds
    steps_to_simulate = int(time_to_simulate / dt)

    # Simulate
    joint_pos_targets = robot.initial_joint_positions.copy()
    for i in range(steps_to_simulate):

        robot.step_with_pd_targets(joint_pos_targets=joint_pos_targets,
                                   Kp=40.,
                                   Kd=1.,
                                   )
    
    # Disconnect from the simulation
    p.disconnect()
