# Imports from libraries
import matplotlib.pyplot as plt
import pybullet as p
import pybullet_data
import numpy as np
import time
import csv

"""
LINKS VERSIÓN ANTIGUA
0 - link_esfera
1 - link_1
2 - link_2
3 - link_3
4 - link_pinza
5 - link_dedo1
6 - link_dedo2
7 - link_mono_1
8 - link_mono_2
9 - link_rueda_1.001
10 - link_rueda_1
11 - link_7
12 - link_rueda_2.001
13 - link_rueda_2
14 - link_6
15 - link_rueda_3.001
16 - link_rueda_3
17 - link_8
18 - link_rueda_4.001
19 - link_rueda_4
20 - link_5
21 - pared_1_link
22 - pared_2_link
23 - pared_3_link
24 - pared_4_link
"""

"""
LINKS VERSIÓN ACTUALIZADA
0 - arm_base_link_joint
1 - arm_column_link_joint
2 - arm_1_link_joint
3 - arm_2_link_joint
4 - arm_gripper_link_joint
5 - gripper_left_link_joint
6 - gripper_right_link_joint
7 - box_down_side_link_joint
8 - box_left_side_link_joint
9 - box_right_side_link_joint
10 - box_up_side_link_joint
11 - subjection_down_left_wheel_link_joint
12 - wheel_down_left_link_joint
13 - subjection_down_right_wheel_link_joint
14 - wheel_down_right_link_joint
15 - subjection_up_left_wheel_link_joint
16 - wheel_up_left_link_joint
17 - subjection_up_right_wheel_link_joint
18 - wheel_up_right_link_joint
"""

# States
STARTING_POSITION = 1
APPROXIMATION_TO_THE_CUBE = 2
MOVE_ARM_DOWN = 3
PICK_UP_CUBE = 4
CLOSE_GRIPPER = 5
MOVE_ARM_UP = 6
ROTATE_ARM = 7
POINT_ARM_IN_THE_TANK = 8
DROP_CUBE = 9
OPEN_GRIPPER = 10
TASK_COMPLETED = 11
RETURN_TO_STARTING_POSITION = 12

# set_gripper_state() function
# Controls the opening and closing of the robot's gripper
def set_gripper_state(gripper_state):
    if gripper_state == OPEN_GRIPPER:
        grippers_positions = [0, 0]
    elif gripper_state == CLOSE_GRIPPER:
        grippers_positions = [-0.4, 0.4]
    else:
        print("error while operating gripper")
        return
    p.setJointMotorControlArray(bodyIndex=robotId,
                                jointIndices=[5, 6],
                                controlMode=p.POSITION_CONTROL,
                                targetPositions=grippers_positions,
                                forces=[500, 500],
                                positionGains=[0.1, 0.1],
                                velocityGains=[3, 3])
    return

# set_arm_pose() function
# Calculate the inverse kinematics and move the robot arm to the target position
def set_arm_pose(target_position, gripper_state, arm_force, arm_position_gain, arm_velocity_gain):
    arm_joints_positions = p.calculateInverseKinematics(robotId, 4, target_position)
    arm_joints_forces = 0
    for i in range(len(arm_joints_positions)):
        set_gripper_state(gripper_state)
        p.setJointMotorControl2(bodyIndex=robotId,
                                jointIndex=i,
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=arm_joints_positions[i],
                                targetVelocity=0,
                                force=arm_force,
                                positionGain=arm_position_gain,
                                velocityGain=arm_velocity_gain)
        arm_joints_forces += abs(p.getJointState(robotId, i)[2][0]) + abs(p.getJointState(robotId, i)[2][1]) + abs(p.getJointState(robotId, i)[2][2])
    return [len(arm_joints_positions), arm_joints_forces]

# We connect engine with GUI
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# We set gravity (X,Y,Z)
p.setGravity(0, 0, -9.8)

# We load a model (plane)
planeID = p.loadURDF("plane.urdf")

# We load new objects, with a position (x,y,z)
# and an orientation given in quaternion (C,X,Y,Z)
robotId = p.loadURDF("./urdf/urdf/robot_p2.urdf", [0, -3, 2])
cubeId = p.loadURDF("./urdf/cube.urdf", [0, 4, 0.5])

"""
PARÁMETROS DE LA NUEVA VERSIÓN
startOrientation = p.getQuaternionFromEuler([0, 0, 3.14])
robotId = p.loadURDF("robot_model/urdf/robot_model.urdf", [0, 0, 1.2], startOrientation)

startOrientation = p.getQuaternionFromEuler([0, 0, 0])
cubeId = p.loadURDF("robot_model/cube.urdf", [0, 4, 0], startOrientation)

wheels_joints = [8, 10, 12, 14]
gripper_left_joint = 5
gripper_right_joint = 6
arm_gripper_joint = 4

# gripper_left_movement = p.addUserDebugParameter("gripper_left_movement", 0, 5, 0)
# gripper_right_movement = p.addUserDebugParameter("gripper_right_movement", 0, 5, 0)
# arm_gripper_movement = p.addUserDebugParameter("arm_gripper_movement", -30, 25, 0)

p.changeDynamics(robotId, gripper_left_joint, jointLowerLimit=0, jointUpperLimit=0.25)
p.changeDynamics(robotId, gripper_right_joint, jointLowerLimit=0, jointUpperLimit=0.25)
p.changeDynamics(robotId, arm_gripper_joint, jointLowerLimit=-1.25, jointUpperLimit=1)
"""

# Constants
data_saved = False
arm_total_force = 0
counter = 0
measure = 0
arm_movement_parameters = [0, 0]
arm_force_parameters = []
state = STARTING_POSITION

for i in range(10):
	p.enableJointForceTorqueSensor(robotId, i, enableSensor=True)
	
for i in range (p.getNumJoints(robotId)):
    print("%d - %s" % (p.getJointInfo(robotId, i)[0], p.getJointInfo(robotId, i)[1].decode("utf-8")))

with open('Fase3_Alberto_León.csv', mode='w', newline='') as file:

    writer = csv.writer(file)
    writer.writerow(['Tiempo', 'NúmeroJoints', 'G_Parcial'])

    while(1):
        
        # Adjust the camera view in PyBullet to follow the Husky robot
        p.resetDebugVisualizerCamera(cameraDistance=5, 
                                     cameraYaw=p.getDebugVisualizerCamera()[8], 
                                     cameraPitch=p.getDebugVisualizerCamera()[9], 
                                     cameraTargetPosition=p.getBasePositionAndOrientation(robotId)[0])

        if state == STARTING_POSITION:
            p.setJointMotorControlArray(bodyIndex=robotId,
                                        jointIndices=[10, 13, 16, 19],
                                        controlMode=p.VELOCITY_CONTROL,
                                        targetVelocities=[-2, -2, -2, -2],
                                        forces=[3, 3, 3, 3])
            if (p.getLinkState(robotId, 10)[0][1] > -0.5):
                p.setJointMotorControlArray(bodyIndex=robotId,
                                            jointIndices=[10, 13, 16, 19],
                                            controlMode=p.VELOCITY_CONTROL,
                                            targetVelocities=[0, 0, 0, 0],
                                            forces=[0, 0, 0, 0])
                if (p.getLinkState(robotId, 10)[0][1] > 1.3):
                    data_saved = True
                    state = APPROXIMATION_TO_THE_CUBE

        elif state == APPROXIMATION_TO_THE_CUBE:
            arm_movement_parameters = set_arm_pose([0, 4, 2], OPEN_GRIPPER, 100, 0.01, 1)
            arm_force_parameters.append(arm_movement_parameters[1])
            if (p.getLinkState(robotId, 4)[0][2] < 2.6):
                state = MOVE_ARM_DOWN
        
        elif state == MOVE_ARM_DOWN:
            arm_movement_parameters = set_arm_pose([0, 4, 0], OPEN_GRIPPER, 500, 0.01, 2)
            arm_force_parameters.append(arm_movement_parameters[1])
            if (p.getLinkState(robotId, 4)[0][2] < 0.6):
                state = PICK_UP_CUBE

        elif state == PICK_UP_CUBE:
            set_gripper_state(CLOSE_GRIPPER)
            if ((p.getLinkState(robotId, 5)[0][0] > 0.26) and (p.getLinkState(robotId, 6)[0][0] < -0.26)):
                state = MOVE_ARM_UP

        elif state == MOVE_ARM_UP:
            arm_movement_parameters = set_arm_pose([0, 2, 3.7], CLOSE_GRIPPER, 450, 0.01, 1.5)
            arm_force_parameters.append(arm_movement_parameters[1])
            if (p.getLinkState(robotId, 4)[0][2] > 3):
                state = ROTATE_ARM
        
        elif state == ROTATE_ARM:
            p.setJointMotorControl2(robotId, 
                                    1, 
                                    p.VELOCITY_CONTROL, 
                                    targetVelocity=1)
            if (p.getJointState(robotId, 1)[0] > 3.2):
                state = POINT_ARM_IN_THE_TANK

        elif state == POINT_ARM_IN_THE_TANK:
            arm_movement_parameters = set_arm_pose([0, -0.7, 3], CLOSE_GRIPPER, 100, 0.01, 1)
            arm_force_parameters.append(arm_movement_parameters[1])
            if (p.getLinkState(robotId, 4)[0][2] < 3.3):
                state = DROP_CUBE
            
        elif state == DROP_CUBE:
            set_gripper_state(OPEN_GRIPPER)
            if ((p.getLinkState(robotId, 5)[0][0] < -0.15) and (p.getLinkState(robotId, 6)[0][0] > 0.3)):
                state = TASK_COMPLETED
        
        elif state == TASK_COMPLETED:
            arm_movement_parameters = set_arm_pose([0, -0.7, 4], OPEN_GRIPPER, 500, 0.01, 1)
            arm_force_parameters.append(arm_movement_parameters[1])
            if (p.getLinkState(robotId, 4)[0][2] > 3.7):
                state = RETURN_TO_STARTING_POSITION
        
        else:
            p.setJointMotorControl2(robotId, 
                                    1, 
                                    p.VELOCITY_CONTROL, 
                                    targetVelocity=2)
            if (p.getJointState(robotId, 1)[0] > 6.2):
                arm_movement_parameters = set_arm_pose([-1.1709028830654165e-05, 1.9037138825406625, 3.6910210603311135], OPEN_GRIPPER, 100, 0.01, 1)
                arm_force_parameters.append(arm_movement_parameters[1])
                if (p.getLinkState(robotId, 4)[0][1] < 2) and (p.getLinkState(robotId, 4)[0][2] < 3.7) and (p.getLinkState(robotId, 4)[0][0] > -1.2):
                    standard_deviation = np.std(arm_force_parameters)
                    for i in arm_force_parameters:
                        arm_total_force += i
                    break
      
        if counter == 2:
            counter = 0
            measure += 0.01
            if arm_movement_parameters[0] != 0:
                writer.writerow([measure, arm_movement_parameters[0], arm_movement_parameters[1]])
        
        counter += 1
        p.stepSimulation()
        time.sleep(1./240.)

plt.figure(figsize=(10, 6))
with open('Fase3_Alberto_León.csv', 'r') as file:
    lines = file.readlines()
    headers = lines[0].strip().split(',')
    data = np.genfromtxt(lines[1:], delimiter=',', dtype=float)
force_index = headers.index('G_Parcial')
time_index = headers.index('Tiempo')
robot_force = data[:, force_index]
time_elapsed = data[:, time_index]
plt.plot(time_elapsed, robot_force, marker='', linestyle='-', label='Fase3_Alberto_León.csv')
plt.xlabel('tiempo')
plt.ylabel('G-parcial')
plt.title("G-Total: " + str(arm_total_force) + " " + "Desviación Estándar de G-parcial: " + str(standard_deviation))
plt.axis('auto')
plt.grid(True)
plt.legend()
plt.show()