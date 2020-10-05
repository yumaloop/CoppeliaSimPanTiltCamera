# Make sure to have the server side running in CoppeliaSim:
# in a child script of a CoppeliaSim scene, add following command
# to be executed just once, at simulation start:
#
# simRemoteApi.start(19999)
#
# then start simulation, and run this program.
#
# IMPORTANT: for each successful call to simxStart, there
# should be a corresponding call to simxFinish at the end!

import os
import sys
import time
import math
import random
import numpy as np
from operator import add
import matplotlib.pyplot as plt
import sim


def remoteAPI_InitTest(clientID):
    if clientID == -1:
        sys.exit("Error: Failed connecting to remote API server.")
    else:
        print("Successfully connected to remote API server.")
        # Now try to retrieve data in a blocking fashion (i.e. a service call):
        res, objs = sim.simxGetObjects(
            clientID, sim.sim_handle_all, sim.simx_opmode_blocking
        )
        if res == sim.simx_return_ok:
            print("Number of objects in the scene: ", len(objs))
            print("Client ID: ", clientID)
        else:
            print("Remote API function call returned with error code: ", res)
            sys.exit("Could not connect")


def move(clientID, targetHandle):
    ret = sim.simxSetObjectPosition(
        clientID, target, -1, (0.1, 0.2, 0.005), sim.simx_opmode_oneshot
    )
    pass


def run_simulation(clientID):
    """
    """
    # http://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctionsPython.htm#simxGetObjectHandle
    ret1, joint_yaw = sim.simxGetObjectHandle(
        clientID, "joint_yaw", sim.simx_opmode_blocking
    )
    ret2, joint_pitch = sim.simxGetObjectHandle(
        clientID, "joint_pitch", sim.simx_opmode_blocking
    )
    ret3, vision_sensor = sim.simxGetObjectHandle(
        clientID, "vision_sensor", sim.simx_opmode_blocking
    )
    ret4, target = sim.simxGetObjectHandle(clientID, "target", sim.simx_opmode_blocking)

    if not ret1 == 0 and ret2 == 0 and ret3 == 0 and ret4 == 0 and ret5 == 0:
        sys.exit("Could not get object handles.")

    # Start simulation
    sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot)
    time.sleep(1)

    target_pos = (0, 0, 0)

    for j in range(100):
        print(f"[{j}]")

        # Generating random positions for the motors
        angle_yaw = random.randint(-30, 30)
        angle_pitch = random.randint(-30, 30)

        # Get current positions
        ret1, yaw_position = sim.simxGetJointPosition(
            clientID, joint_yaw, sim.simx_opmode_blocking
        )
        ret2, pitch_position = sim.simxGetJointPosition(
            clientID, joint_pitch, sim.simx_opmode_blocking
        )

        # Set new positions
        ret = sim.simxSetJointPosition(
            # clientID, joint_yaw, math.radians(angle_yaw), sim.simx_opmode_oneshot
            clientID, joint_yaw, math.radians(angle_yaw), sim.simx_opmode_blocking
        )
        ret = sim.simxSetJointPosition(
            # clientID, joint_pitch, math.radians(angle_pitch), sim.simx_opmode_oneshot
            clientID, joint_pitch, math.radians(angle_pitch), sim.simx_opmode_blocking
        )

        # Update target's position
        diff_pos = (0.05, 0.05, 0.0)
        target_pos = tuple(map(add, target_pos, diff_pos))
        ret = sim.simxSetObjectPosition(
            # clientID, target, -1, target_pos, sim.simx_opmode_oneshot
            clientID, target, -1, target_pos, sim.simx_opmode_blocking
        )

        # Get images from vision sensors
        ret, resolution, image = sim.simxGetVisionSensorImage(
            # clientID, vision_sensor, 0, sim.simx_opmode_streaming
            clientID, vision_sensor, 0, sim.simx_opmode_blocking
        )
        print("ret", ret, "resolution", resolution, "type(image)", type(image))

        if len(resolution) != 0:
            img = np.array(image, dtype=np.uint8)
            img.resize([resolution[0], resolution[1], 3])
            print("img.shape", img.shape)

        # plt.imshow(img, origin="lower")
        """
        r, resolution, image = sim.simxGetVisionSensorImage(clientID, vision_sensor, 0, sim.simx_opmode_streaming)
        img = np.array(image, dtype = np.uint8)
        img.resize([resolution[0], resolution[1], 3])
        # plt.imshow(img, origin="lower")
        """
        time.sleep(0.1)

    # End simulation
    sim.simxStopSimulation(clientID, opmode)


def main(
    address="127.0.0.1",
    port=19999,
    waitUntilConnected=True,
    doNotReconnectOnceDisconnected=True,
    timeOutInMs=5000,
    commThreadCycleInMs=5,
):
    print("Program started.")
    # just in case, close all opened connections
    sim.simxFinish(-1)

    # Connect to CoppeliaSim
    # https://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctionsPython.htm#simxStart
    # For example, clientID is setted as 0
    clientID = sim.simxStart(
        address,
        port,
        waitUntilConnected,
        doNotReconnectOnceDisconnected,
        timeOutInMs,
        commThreadCycleInMs,
    )

    # Check if clientID is valid
    remoteAPI_InitTest(clientID)

    # Execute simulation loop
    run_simulation(clientID)
    
    # Now close the connection to CoppeliaSim:
    sim.simxFinish(clientID)

    print("Program ended.")


if __name__ == "__main__":
    main()
