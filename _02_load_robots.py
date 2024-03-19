
import pybullet as p
import time
import numpy as np

def load_robot(
        urdf_path,
        initial_base_position,
        num_dof,
        initial_joint_positions,
        joint_indices,
        camera_distance,
        fix_base,
):

    # Start PyBullet in GUI mode
    physicsClient = p.connect(p.GUI)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
    p.configureDebugVisualizer(rgbBackground=[0, 0, 0])
    p.setPhysicsEngineParameter(fixedTimeStep=0.002,
                                numSolverIterations=50, 
                                solverResidualThreshold=1e-30, 
                                numSubSteps=1,)


    # Load a plane and a few objects
    plane = p.loadURDF("assets/plane.urdf", useMaximalCoordinates=True)
    robot = p.loadURDF(urdf_path, 
                    basePosition=initial_base_position, 
                    baseOrientation=p.getQuaternionFromEuler([0, 0, 0]),
                    useFixedBase=fix_base,
                    )

    p.setGravity(0, 0, -9.81)

    # Camera parameters
    cameraDistance = camera_distance    # How far the camera is from the target
    cameraYaw = 45        # Yaw angle in degrees
    cameraPitch = -15     # Pitch angle in degrees
    cameraTargetPosition = [initial_base_position[0],
                            initial_base_position[1],
                            initial_base_position[2] - 0.3]  # Focus just below the base position

    # Set the camera view
    p.resetDebugVisualizerCamera(cameraDistance, cameraYaw, cameraPitch, cameraTargetPosition)

    for j in range(p.getNumJoints(robot)):
        print(p.getJointInfo(robot, j))

    # Set initial position
    for j in range(num_dof):
        p.resetJointState(  robot,
                            joint_indices[j], 
                            targetValue=initial_joint_positions[j],
                            targetVelocity=0,
        )
        
    p.setJointMotorControlArray(bodyIndex=robot,
                    jointIndices=joint_indices,
                    controlMode=p.VELOCITY_CONTROL,
                    targetVelocities=[0] * num_dof,
                    forces=[0] * num_dof,
    )
    p.setJointMotorControlArray(bodyIndex=robot,
                    jointIndices=joint_indices,
                    controlMode=p.TORQUE_CONTROL,
                    forces=[0] * num_dof,
    )
    p.stepSimulation()

    return robot

if __name__ == "__main__":

    robot_name = "Go1"

    if robot_name == "Go1":
        urdf_path="assets/go1_description/urdf/go1.urdf"
        initial_base_position=[0, 0, 0.40]
        num_dof=12
        initial_joint_positions=[0.0, 0.80, -1.5, 0.0, 0.80, -1.5, 0.0, 0.8, -1.5, 0.0, 0.8, -1.5]
        joint_indices=[2, 3, 4, 9, 10, 11, 16, 17, 18, 23, 24, 25]
        camera_distance = 0.8
    elif robot_name == "Go2":
        urdf_path="assets/go2_description/urdf/go2_description.urdf"
        initial_base_position=[0, 0, 0.40]
        num_dof=12
        initial_joint_positions=[0.0, 0.80, -1.5, 0.0, 0.80, -1.5, 0.0, 0.8, -1.5, 0.0, 0.8, -1.5]
        joint_indices=[2, 3, 4, 8, 9, 10, 14, 15, 16, 20, 21, 22]
        camera_distance = 0.8
    elif robot_name == "H1":
        urdf_path="assets/h1_description/urdf/h1.urdf"
        initial_base_position=[0, 0, 1.4]
        num_dof=18
        initial_joint_positions=[0.0] * 18
        joint_indices=[0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17]
        camera_distance = 1.5
    elif robot_name == "H1_hands":
        urdf_path="assets/h1_description/urdf/h1_with_hand.urdf"
        initial_base_position=[0, 0, 1.4]
        num_dof=19
        initial_joint_positions=[0.0] * 19
        joint_indices=[0, 1, 2, 3, 4, 5, 6, 7, 8, 9,
                10, 11, 12, 13, 14, 29, 30, 31, 32,]
        camera_distance = 1.5
    else:
        print(f"Robot {robot_name} not available!")

    robot = load_robot(
        urdf_path=urdf_path,
        initial_base_position=initial_base_position,
        num_dof=num_dof,
        initial_joint_positions=initial_joint_positions,
        joint_indices=joint_indices,
        camera_distance=camera_distance,
        fix_base=True,
    )

    # Simulate
    ts = time.time()
    k=0
    for i in range(100000):
        joint_pos_targets = initial_joint_positions
        joint_states = p.getJointStates(robot, joint_indices)
        joint_pos = np.array([joint_states[n][0] for n in range(num_dof)])
        joint_vel = np.array([joint_states[n][1] for n in range(num_dof)])

        torques = 40 * (np.array(joint_pos_targets) - joint_pos) - 1.0 * (joint_vel)
        p.setJointMotorControlArray(bodyIndex=robot,
                            jointIndices=joint_indices,
                            controlMode=p.TORQUE_CONTROL,
                            forces=torques)
        p.stepSimulation()
        time.sleep(1./500.)
        if i % 500 == 0:
            print(f"Frequency: {i/(time.time() - ts)}")
    
    # Disconnect from the simulation
    p.disconnect()
