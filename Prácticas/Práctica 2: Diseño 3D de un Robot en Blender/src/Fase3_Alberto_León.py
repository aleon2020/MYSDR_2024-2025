# Imports from libraries
import matplotlib.pyplot as plt
import pybullet as p
import pybullet_data
import numpy as np
import threading
import time
import csv

# Variable initialization
simulation_active = True
G_partial_values_list = []

# save_data() function
# Saves the 'Time', 'Number of Joints' and 'G-partial' data to a CSV file.
def save_data(robotId, joint_indexes, G_partial_values_list, sample_rate=0.01):
    global simulation_active
    elapsed_time = 0.0
    with open("Fase3_Alberto_León.csv", "w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(['Tiempo', 'NúmeroJoints', 'G_parcial'])
        while simulation_active:
            G_partial = sum(abs(p.getJointState(robotId, i)[3]) for i in joint_indexes)
            G_partial_values_list.append(G_partial)
            writer.writerow([round(elapsed_time, 3), len(joint_indexes), round(G_partial, 4)])
            time.sleep(sample_rate)
            elapsed_time += sample_rate

# We connect engine with GUI
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# We set gravity (X,Y,Z)
p.setGravity(0, 0, -9.8)

p.setTimeStep(0.005)

# We load a model (plane)
planeID = p.loadURDF("plane.urdf")

# We load new objects, with a position (x,y,z)
# and an orientation given in quaternion (C,X,Y,Z)
startOrientation = p.getQuaternionFromEuler([0, 0, 3.14])
robotId = p.loadURDF("robot_model/urdf/robot_model.urdf", [0, -3, 1.2], startOrientation)
startOrientation = p.getQuaternionFromEuler([0, 0, 0])
cubeId = p.loadURDF("robot_model/cube.urdf", [0, 4, 0], startOrientation)

for i in range(p.getNumJoints(robotId)):
    print("%d - %s" % (p.getJointInfo(robotId, i)[0], p.getJointInfo(robotId, i)[1].decode("utf-8")))
    p.enableJointForceTorqueSensor(robotId, i, enableSensor=True)

measured_joint_indexes = [2, 3, 4]

data_thread = threading.Thread(target=save_data, args=(robotId, measured_joint_indexes, G_partial_values_list))
data_thread.start()

wheel_joint_indexes = [12, 14, 16, 18]

end_effector_joint_index = 4

cube_target_position = p.getBasePositionAndOrientation(cubeId)[0][0]

while True:

    robot_current_position = p.getLinkState(robotId, end_effector_joint_index)[0][0]
    position_error = cube_target_position - robot_current_position
    if position_error < 0.01:
        break
    p.setJointMotorControlArray(
        bodyUniqueId=robotId,
        jointIndices=wheel_joint_indexes,
        controlMode=p.VELOCITY_CONTROL,
        targetVelocities=[5, 5, -5, -5])

    p.stepSimulation()

for i in wheel_joint_indexes:
    p.setJointMotorControl2(robotId, i, p.VELOCITY_CONTROL, targetVelocity=0)

arm_joint_indexes = [2, 3, 4]
prismatic_arm_joint_index = 4
gripper_joint_indexes = [6, 5]
end_effector_joint_index = 4

cube_current_position, _ = p.getBasePositionAndOrientation(cubeId)
cube_target_position = [cube_current_position[0], cube_current_position[1], 0.5]
arm_joint_angles = p.calculateInverseKinematics(robotId, end_effector_joint_index, cube_target_position)

# CALCULATION OF INVERSE KINEMATICS
for _ in range(400):
    for i in arm_joint_indexes:
        angle = arm_joint_angles[i]
        p.setJointMotorControl2(robotId,
                                i,
                                p.POSITION_CONTROL,
                                targetPosition=angle,
                                maxVelocity=1)
    p.stepSimulation()

# STEP 1: LOWER ARM
for _ in range(800):
    p.setJointMotorControl2(robotId,
                            prismatic_arm_joint_index,
                            p.POSITION_CONTROL,
                            targetPosition=-1.45,
                            maxVelocity=1)
    p.stepSimulation()

# STEP 2: PICK UP CUBE
for _ in range(600):
    p.setJointMotorControl2(robotId,
                            gripper_joint_indexes[0],
                            p.POSITION_CONTROL,
                            targetPosition=-0.2,
                            maxVelocity=1)
    p.setJointMotorControl2(robotId,
                            gripper_joint_indexes[1],
                            p.POSITION_CONTROL,
                            targetPosition=0.2,
                            maxVelocity=1)
    p.stepSimulation()

# STEP 3: RAISE ARM
for _ in range(1000):
    p.setJointMotorControl2(robotId,
                            prismatic_arm_joint_index,
                            p.POSITION_CONTROL,
                            targetPosition=0.0,
                            maxVelocity=1)
    p.stepSimulation()

# STEP 4: TURN ARM
for _ in range(1200):
    p.setJointMotorControl2(robotId,
                            arm_joint_indexes[0],
                            p.POSITION_CONTROL,
                            targetPosition=np.pi,
                            maxVelocity=1)
    p.stepSimulation()

# STEP 5: RELEASE CUBE
for _ in range(600):
    for i in gripper_joint_indexes:
        p.setJointMotorControl2(robotId,
                                i,
                                p.POSITION_CONTROL,
                                targetPosition=0.0,
                                maxVelocity=1)
    p.stepSimulation()

simulation_active = False
data_thread.join()

G_total = sum(G_partial_values_list)
standard_deviation = np.std(G_partial_values_list)
time_elapsed = np.arange(0, len(G_partial_values_list)) * 0.01
time_elapsed = time_elapsed[:len(G_partial_values_list)]
plt.plot(time_elapsed, G_partial_values_list, marker='', linestyle='-', label='Fase3_Alberto_León.csv')
plt.xlabel('tiempo')
plt.ylabel('G-parcial')
plt.title(f'G-Total: {round(G_total, 2)}, Desviación Estándar de G-parcial: {round(standard_deviation, 2)}')
plt.axis('auto')
plt.grid(True)
plt.legend()
plt.show()

p.disconnect()