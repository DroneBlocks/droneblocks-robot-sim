
import pybullet as p
import numpy as np

from _03_stand_robots import Robot

if __name__ == "__main__":

    ######
    # CREATING THE SCENE
    ######

    # add the robot to the simulation
    robot = Robot(robot_name="H1_hands", fix_base=True)

    # add a table to the simulation
    tableId = p.loadURDF("assets/table.urdf", basePosition=[0.4, 0., 0.6], baseOrientation=p.getQuaternionFromEuler([0, 0, 0]), useFixedBase=True)

    # add a box to the simulation
    boxId = p.loadURDF("assets/box.urdf", basePosition=[0.4, 0., 1.5], baseOrientation=p.getQuaternionFromEuler([0, 0, 0]))
    
    # add some small spheres to the simulation
    sphereIds = []
    for i in range(25):
        sphereId = p.loadURDF("assets/sphere.urdf", basePosition=[0.4+(i%3)*0.1, 0.7+(i//3)*0.1, 0.2], baseOrientation=p.getQuaternionFromEuler([0, 0, 0]))
        sphereIds.append(sphereId)

    ######
    # SIMULATION LOOP
    ######

    dt = 0.002
    time_to_simulate = 2000 # seconds
    steps_to_simulate = int(time_to_simulate / dt)
    joint_pos_targets = robot.initial_joint_positions.copy()

    for i in range(steps_to_simulate):

        robot.step_with_pd_targets(joint_pos_targets=joint_pos_targets,
                                   Kp=40.,
                                   Kd=1.,
                                   )
        
        ######
        # GETTING OBJECT STATE
        ######

        # get the position of the box
        box_pos, box_ori = p.getBasePositionAndOrientation(boxId)

        # get the position of the robot
        robot_pos, robot_ori = p.getBasePositionAndOrientation(robot.robot)

        # get the position of the hands
        left_hand_pos, left_hand_ori, _, _, _, _ = p.getLinkState(robot.robot, 15)
        right_hand_pos, right_hand_ori, _, _, _, _ = p.getLinkState(robot.robot, 33)

        ######
        # IMPLEMENTING A CUSTOM CONTROLLER
        ######

        # set the pelvis angle so the robot will always try to face towards the box
        robot_box_vec = np.array(box_pos) - np.array(robot_pos)
        robot_box_angle = np.arctan2(robot_box_vec[1], robot_box_vec[0])
        joint_pos_targets[10] = robot_box_angle


    # Disconnect from the simulation
    p.disconnect()
