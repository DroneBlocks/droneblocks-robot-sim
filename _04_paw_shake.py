#!/usr/bin/python

import sys
import time
import numpy as np
import math


try:
    # # Add the path to the robot interface library
    # sys.path.append('./unitree_legged_sdk-go1/lib/python/arm64')
    # import robot_interface as sdk
    from sdk_emulator import UnitreeSDKEmulator as sdk
    has_sdk = True
except ModuleNotFoundError:
    print("No robot interface library found. Running in test mode only, the robot will not do anything.")
    has_sdk = False

from keyutils import non_blocking_key_reader

dt = 0.005

# Function for inverse kinematics
def computeStandingAngles(desired_height, qInit, desired_x_off, desired_y_off):
    hip_width = 0.05
    calf_length = 0.18
    thigh_length = 0.18
    
    # compute angles of the triangle between the robot's hip, knee, and foot
    theta1 = math.acos((thigh_length**2 + desired_height**2 - calf_length**2) / (2 * thigh_length * desired_height))
    theta2 = math.acos((thigh_length**2 + calf_length**2 - desired_height**2) / (2 * thigh_length * calf_length))
    
    # offsets
    theta1 = theta1 + 0.8 - 0.59
    theta2 = theta2 - 1.5 - 2.0
    
    # body offset -- hip and thigh
    theta0 = 0 + math.asin(desired_y_off / desired_height)
    theta1 = theta1 + math.asin(desired_x_off / desired_height)
    
    # clip by qInit
    qDesired = [theta0, theta1, theta2, 
            theta0, theta1, theta2,
            theta0, theta1, theta2,
            theta0, theta1, theta2,
            ]
    
    clip_dist = 0.8 * dt # max change per second
    for i in range(12):
        qDesired[i] = min(qInit[i] + clip_dist, max(qInit[i] - clip_dist, qDesired[i]))
        qInit[i] = qDesired[i]
    
    return qDesired, qInit

# Initialize desired height
desired_height = 0.3  # Default height in meters
desired_x_off = 0.0 # Default body x-shift (centered over legs)
desired_y_off = 0.0 # Default body y-shift (centered over legs)
is_shaking_left = False # flag to relax the front left leg

# Display button mapping
print("\n\r\
Use the arrow keys to adjust the body shift:\n\r\
Up arrow: shift forward | \
Down arrow: shift back | \
Left arrow: shift left | \
Right arrow: shift right \n\r\
Press spacebar to shake the dog's front left leg. First, make sure it's not putting any weight on that foot!\n\r\
Press 'q' to quit\n\r")

# Main function
def main(reader):
    # Initialize variables
    global dt
    qDes = [0] * 12
    qInit = [0] * 12
    
    if has_sdk:
        # Initialize UDP and safety modules
        udp = sdk.UDP(0xff, 8080, "192.168.123.10", 8007)
        safe = sdk.Safety(sdk.LeggedType.Go1)

        # Initialize command and state variables
        cmd = sdk.LowCmd()
        state = sdk.LowState()
        udp.InitCmdData(cmd)
    
    motiontime = 0
    freq_counter = 0
    sin_count = 0
    qShakeCenter = []

    global desired_height, desired_x_off, desired_y_off, is_shaking_left

    ts = time.time()
    freq = 0

    # Main loop
    while True:
        time.sleep(dt)
        motiontime += 1
        freq_counter += 1

        if time.time() - ts > 0.5:
            freq = freq_counter / (time.time() - ts)
            ts = time.time()
            freq_counter = 0

        # Capture keypress
        key = next(reader, None)
        if key is not None:
            if 'A' in key:  # Up arrow
                desired_x_off = min(desired_x_off + 0.01, 0.2)
            elif 'B' in key:  # Down arrow
                desired_x_off = max(desired_x_off - 0.01, -0.2)
            elif 'C' in key:  # Right arrow
                desired_y_off = min(desired_y_off + 0.01, 0.2)
            elif 'D' in key:  # Left arrow
                desired_y_off = max(desired_y_off - 0.01, -0.2)
            elif ' ' in key:
                is_shaking_left = not is_shaking_left
                sin_count = 0
                qShakeCenter = [qInit[3], qInit[4], qInit[5]]
            elif key == 'q':
                print("\n\rQuitting.")
                raise Exception
            
        # Display target height (overwrite the current line)
        sys.stdout.write(f"\rTarget offset: (x={desired_x_off:.2f}, y={desired_y_off:.2f}) meters | Shaking mode: {is_shaking_left} | Control Frequency: {int(freq)} Hz")
        sys.stdout.flush()

        if has_sdk:
            # Receive state data
            udp.Recv()
            udp.GetRecv(state)
            
            # Read initial positions if not already read
            if motiontime < 10:
                for joint_idx in range(12):
                    qInit[joint_idx] = state.motorState[joint_idx].q
            
            if motiontime >= 10:
                # Compute joint angles based on desired height
                qDes, qInit = computeStandingAngles(desired_height, qInit, desired_x_off, desired_y_off)

                # Shake the paw
                if is_shaking_left:

                     # Perform sinusoidal oscillation
                    freq_Hz = 1
                    freq_rad = freq_Hz * 2 * math.pi
                    t = dt * sin_count
                    sin_count += 1
                    
                    init_time = 0.35
                    
                    if t < init_time: # first, move to initial pos
                        qShakeCenter[1] = qShakeCenter[1] - 0.01
                        # qShakeCenter[2] = qShakeCenter[2] - 0.004
                        qDes[3] = qShakeCenter[0]
                        qDes[4] = qShakeCenter[1]
                        qDes[5] = qShakeCenter[2]
                    else: # then shake
                        sin_joint1 = 0.0 #0.6 * math.sin((t - init_time) * freq_rad)
                        sin_joint2 = -0.2 * math.sin((t - init_time) * freq_rad * 2)
                        qDes[3] = qShakeCenter[0]
                        qDes[4] = qShakeCenter[1] + sin_joint1
                        qDes[5] = qShakeCenter[2] + sin_joint2

                    qInit[3] = qDes[3]
                    qInit[4] = qDes[4]
                    qInit[5] = qDes[5]

                # Set joint commands
                for joint_idx in range(12):
                    cmd.motorCmd[joint_idx].q = qDes[joint_idx]
                    cmd.motorCmd[joint_idx].dq = 0
                    cmd.motorCmd[joint_idx].Kp = 60
                    cmd.motorCmd[joint_idx].Kd = 1.5
                    cmd.motorCmd[joint_idx].tau = 0.0

                

            # Apply safety measures
            safe.PowerProtect(cmd, state, 1)

            # Send the command data
            udp.SetSend(cmd)
            udp.Send()

# Run the main function
if __name__ == '__main__':
    reader = non_blocking_key_reader()
    main(reader)