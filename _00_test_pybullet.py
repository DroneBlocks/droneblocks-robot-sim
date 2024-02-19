
try:
    import pybullet as p
    import time

    # Start PyBullet in GUI mode
    physicsClient = p.connect(p.GUI)

    # Simulate for a short time
    for i in range(10):
        p.stepSimulation()
        time.sleep(1./50.)

    # Disconnect from the simulation
    p.disconnect()

    print("=====================\n=====================\nPybullet is successfully installed!\n=====================\n=====================")
except Exception as e:
    print(f"You are missing dependencies: Received exception {e}")