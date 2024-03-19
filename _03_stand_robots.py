
import pybullet as p
import time
import numpy as np
import pybullet_data

from _02_load_robots import load_robot

# Let's create a Robot class to contain the generic functionality
class Robot:
    def __init__(self, robot_name, fix_base=False):

        ## Add reset function

        self.load_params(robot_name)

        self.robot = load_robot(
            urdf_path=self.urdf_path,
            initial_base_position=self.initial_base_position,
            num_dof=self.num_dof,
            initial_joint_positions=self.initial_joint_positions,
            joint_indices=self.joint_indices,
            camera_distance=self.camera_distance,
            fix_base=fix_base,
        )

        self.time_counter = 0
        
    def get_joint_states(self):
        joint_states = p.getJointStates(self.robot, self.joint_indices)
        joint_pos = np.array([joint_states[n][0] for n in range(self.num_dof)])
        joint_vel = np.array([joint_states[n][1] for n in range(self.num_dof)])
        return joint_pos, joint_vel

    def step_with_torques(self, torques):
        p.setJointMotorControlArray(bodyIndex=self.robot,
                            jointIndices=self.joint_indices,
                            controlMode=p.TORQUE_CONTROL,
                            forces=torques)
        p.stepSimulation()
        time.sleep(1./500.)
        self.time_counter += 1

    def step_with_pd_targets(self, joint_pos_targets, Kp, Kd):
        joint_pos, joint_vel = self.get_joint_states()

        torques = Kp * (np.array(joint_pos_targets) - joint_pos) - Kd * (joint_vel)
        
        self.step_with_torques(torques)

    def load_params(self, robot_name):
        if robot_name == "Go1":
            self.urdf_path="assets/go1_description/urdf/go1.urdf"
            self.initial_base_position=[0, 0, 0.38]
            self.num_dof=12
            self.initial_joint_positions=[0.0, 0.80, -1.5, 0.0, 0.80, -1.5, 0.0, 0.8, -1.5, 0.0, 0.8, -1.5]
            self.joint_indices=[2, 3, 4, 9, 10, 11, 16, 17, 18, 23, 24, 25]
            self.camera_distance = 0.8
        elif robot_name == "Go2":
            self.urdf_path="assets/go2_description/urdf/go2_description.urdf"
            self.initial_base_position=[0, 0, 0.38]
            self.num_dof=12
            self.initial_joint_positions=[0.0, 0.80, -1.5, 0.0, 0.80, -1.5, 0.0, 0.8, -1.5, 0.0, 0.8, -1.5]
            self.joint_indices=[2, 3, 4, 8, 9, 10, 14, 15, 16, 20, 21, 22]
            self.camera_distance = 0.8
        elif robot_name == "H1":
            self.urdf_path="assets/h1_description/urdf/h1.urdf"
            self.initial_base_position=[0, 0, 1.3]
            self.num_dof=18
            self.initial_joint_positions=[0.0] * 18
            self.joint_indices=[0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17]
            self.camera_distance = 1.5
        elif robot_name == "H1_hands":
            self.urdf_path="assets/h1_description/urdf/h1_with_hand.urdf"
            self.initial_base_position=[0, 0, 1.3]
            self.num_dof=19
            self.initial_joint_positions=[0.0] * 19
            self.joint_indices=[0, 1, 2, 3, 4, 5, 6, 7, 8, 9,
                    10, 11, 12, 13, 14, 29, 30, 31, 32,]
            self.camera_distance = 1.5
        else:
            print(f"Robot {robot_name} not available!")


if __name__ == "__main__":

    # robot = Robot(robot_name="Go1", fix_base=False)
    # robot = Robot(robot_name="Go2", fix_base=False)
    # robot = Robot(robot_name="H1", fix_base=False)
    robot = Robot(robot_name="H1_hands", fix_base=False)

    # Simulate
    joint_pos_targets = robot.initial_joint_positions
    dt = 0.002
    time_to_simulate = 100 # seconds
    steps_to_simulate = int(time_to_simulate / dt)

    for i in range(steps_to_simulate):
        robot.step_with_pd_targets(joint_pos_targets=joint_pos_targets,
                                   Kp=40.,
                                   Kd=1.,
                                   )
    
    # Disconnect from the simulation
    p.disconnect()
