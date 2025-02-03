# Imports de las librerías
import pybullet as p
import pybullet_data
import argparse
import time
import numpy as np
import math

parser = argparse.ArgumentParser(description="URDF viewer example")
parser.add_argument("--urdf", type=str, required=True, help="Ruta al archivo URDF.")
args = parser.parse_args()
urdf_path = "urdf/my_arm.urdf"

# Conectamos motor con GUI
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Establecemos gravedad (X,Y,Z)
p.setGravity(0, 0, -9.8)

# p.setRealTimeSimulation()
# El motor de físicas no realiza pausas en la simulación.
# Ejecuta en tiempo real acorde al RTC del sistema.
# Depende del rendimiento del sistema.
p.setRealTimeSimulation(1)

# Cargamos un/unos modelo/s
planeId = p.loadURDF("plane.urdf")

# Cargamos un nuevo objeto, con una posición (x,y,z)
# y una orientación dada en cuaternión (X,Y,Z)
robotId = p.loadURDF(urdf_path, [0, 0, 1.0])
# robotId = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0], useFixedBase=True)

numJoints = p.getNumJoints(robotId)
print("NumJoints: " + str(numJoints))

for j in range (numJoints):
    print("%d - %s" % (p.getJointInfo(robotId,j)[0], p.getJointInfo(robotId,j)[1].decode("utf-8")))

# Solo queremos que la cinemática inversa actue sobre los primeros 3 JOINTS
robotEndEffectorIndex=3

pos_target = [1, 1, 1]
jointPoses = p.calculateInverseKinematics(robotId, robotEndEffectorIndex, pos_target)
print(jointPoses)
p.addUserDebugText("X", pos_target, [1, 0, 0], 1)


for i in range(len(jointPoses)):
    p.setJointMotorControl2(bodyIndex=robotId,
                                    jointIndex=i,
                                    controlMode=p.POSITION_CONTROL,
                                    targetPosition=jointPoses[i],
                                    targetVelocity=0,
                                    force=500,
                                    positionGain=0.03,
                                    velocityGain=1)
while(1):

    # No es necesario, solo para ver más despacio el movimiento.
    time.sleep(1./20.)
    pass
