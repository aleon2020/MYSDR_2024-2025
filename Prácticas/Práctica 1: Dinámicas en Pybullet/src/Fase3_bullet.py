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

p.setRealTimeSimulation(1)

last_distance = -1
start_time = time.time()
csv_values = []

while (1):

    # p.stepSimulation()
    # time.sleep(1./240.)

    # Escenario 3.1: Asignación velocidades y fuerzas
    speed = 15
    torque = 50

    # Escenario 3.2: Asignación velocidades y fuerzas + fricción
    # for i in joints:
    #     p.changeDynamics(huskyId, i, lateralFriction=0.93)
    #     p.changeDynamics(huskyId, i, spinningFriction=0.005)
    #     p.changeDynamics(huskyId, i, rollingFriction=0.003)

    # Escenario 3.3: Asignación velocidades y fuerzas + fricción + Inercia
    # p.changeDynamics(barrierId, 0, localInertiaDiagonal=[20/3, 0.0, 20/3])

    p.setJointMotorControlArray(huskyId,
                                joints,
                                p.VELOCITY_CONTROL,
                                targetVelocities=[speed, speed, speed, speed],
                                forces=[torque, torque, torque, torque])
    
    p.resetDebugVisualizerCamera(cameraDistance=5, 
                                 cameraYaw=p.getDebugVisualizerCamera()[8], 
                                 cameraPitch=p.getDebugVisualizerCamera()[9], 
                                 cameraTargetPosition=p.getBasePositionAndOrientation(huskyId)[0])

    distance = p.getBasePositionAndOrientation(huskyId)[0][0]
    if (distance != last_distance):
        csv_values.append([time.time() - start_time, distance, p.getBaseVelocity(huskyId)[0][0], speed, torque])
        last_distance = distance

    if p.getBasePositionAndOrientation(huskyId)[0][0] >= 20:
        break

p.disconnect()

with open('Fase2.csv', mode = 'w', newline = '') as file:
    writer = csv.writer(file)
    writer.writerow(['tiempo', 'posicion_robot[Y]', 'velocidad_robot[Y]', 'velocidad_ruedas', 'fuerza_ruedas'])
    for i in csv_values:
    	writer.writerow([i[0], i[1], i[2], i[3], i[4]])