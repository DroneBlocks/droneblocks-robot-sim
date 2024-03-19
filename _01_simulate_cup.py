import pybullet as p
import time

# Start PyBullet in GUI mode
physicsClient = p.connect(p.GUI)
p.setPhysicsEngineParameter(fixedTimeStep=0.002,
                            numSolverIterations=50,#300, 
                            solverResidualThreshold=1e-30, 
                            numSubSteps=1,)
p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)

# Load a plane and a few objects
planeId = p.loadURDF("assets/plane.urdf", useMaximalCoordinates=True)
cubeId = p.loadURDF("assets/mug/mug.urdf", basePosition=[0,0,0.1], baseOrientation=p.getQuaternionFromEuler([0.2, 0, 0]))

p.setGravity(0, 0, -9.81)

# Camera parameters
cameraDistance = 0.5    # How far the camera is from the target
cameraYaw = 45        # Yaw angle in degrees
cameraPitch = -15     # Pitch angle in degrees
cameraTargetPosition = [0, 0, 0]  # Focus on the origin

# Set the camera view
p.resetDebugVisualizerCamera(cameraDistance, cameraYaw, cameraPitch, cameraTargetPosition)

# Simulate
ts = time.time()
for i in range(100000):
    p.stepSimulation()
    time.sleep(1./500.)
    if i % 500 == 0:
        print(f"Frequency: {i/(time.time() - ts)}")

# Disconnect from the simulation
p.disconnect()
