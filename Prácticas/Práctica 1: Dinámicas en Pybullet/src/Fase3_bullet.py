# Imports from libraries
import pybullet as p
import pybullet_data
import time
import csv

# We connect engine with GUI
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# We set gravity (X,Y,Z)
p.setGravity(0, 0, -9.8)

# We load a model (plane)
planeID = p.loadURDF("plane.urdf")

euler_angles = [0, 0, 0]
startOrientation = p.getQuaternionFromEuler(euler_angles)

# We load new objects, with a position (x,y,z)
# and an orientation given in quaternion (C,X,Y,Z)
huskyId = p.loadURDF("husky/husky.urdf", [0, 0, 0], startOrientation)
rampId = p.loadURDF("urdf/ramp.urdf", [10, 0, 0], startOrientation)
barrierId = p.loadURDF("urdf/barrier.urdf", [17, 1.5, 0], startOrientation)
finishlineId = p.loadURDF("urdf/finish_line.urdf", [20, 0, 0], startOrientation)

joints = [2, 3, 4, 5]

# Set the realtime mode of the simulator
p.setRealTimeSimulation(1)

last_advance = -1
initial_time = time.time()
data_records = []

while (1):

    # p.stepSimulation()
    # time.sleep(1./240.)

    # Scenario 3.1: Assignment of speeds and forces
    speed = 11
    torque = 25

    # Scenario 3.2: Assignment of speeds and forces + friction
    # for i in joints:
    #     p.changeDynamics(huskyId, i, lateralFriction=0.93, spinningFriction=0.005, rollingFriction=0.003)
    #     p.changeDynamics(huskyId, i, spinningFriction=0.005)
    #     p.changeDynamics(huskyId, i, rollingFriction=0.003)

    # Scenario 3.3: Assignment of speeds and forces + friction + Inertia
    # p.changeDynamics(barrierId, 0, localInertiaDiagonal=[20/3, 0.0, 20/3])

    # Relates movement to the simultaneous activation of several joints
    p.setJointMotorControlArray(huskyId,
                                joints,
                                p.VELOCITY_CONTROL,
                                targetVelocities=[speed, speed, speed, speed],
                                forces=[torque, torque, torque, torque])
    
    # Adjust the camera view in PyBullet to follow the Husky robot
    p.resetDebugVisualizerCamera(cameraDistance=5, 
                                 cameraYaw=p.getDebugVisualizerCamera()[8], 
                                 cameraPitch=p.getDebugVisualizerCamera()[9], 
                                 cameraTargetPosition=p.getBasePositionAndOrientation(huskyId)[0])

    # Collects and records information every 0.01 meters that the robot advances
    current_advance = p.getBasePositionAndOrientation(huskyId)[0][0]
    if (current_advance != last_advance):
        data_records.append([time.time() - initial_time, current_advance, p.getBaseVelocity(huskyId)[0][0], speed, torque])
        last_advance = current_advance

    # The execution stops when the robot reaches the end (finish_line)
    if p.getBasePositionAndOrientation(huskyId)[0][0] >= 20:
        break

p.disconnect()

# All data is saved in a file in CSV format and exits PyBullet
with open('Fase3.1.csv', mode = 'w', newline = '') as file:
    writer = csv.writer(file)
    writer.writerow(['tiempo', 'posicion_robot[Y]', 'velocidad_robot[Y]', 'velocidad_ruedas', 'fuerza_ruedas'])
    for i in data_records:
    	writer.writerow([i[0], i[1], i[2], i[3], i[4]])