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
#robotId = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0], useFixedBase=True)

numJoints = p.getNumJoints(robotId)
print("NumJoints: " + str(numJoints))

for j in range (numJoints):
    print("%d - %s" % (p.getJointInfo(robotId, j)[0], p.getJointInfo(robotId, j)[1].decode("utf-8")))

trailDuration = 5
prevPose = [0, 0, 0]
prevPose1 = [0, 0, 0]
hasPrevPose = 1

# Solo queremos que la cinemática inversa actue sobre los primeros 3 JOINTS
robotEndEffectorIndex=4
going = True

# Vamos a posicionar el brazo por una diagonal X,Y usando la misma altura (Z)
# El rango máximo/minimo definido
range_m=[0.3, 2.0]

c=range_m[0]

while (1):

    # No es necesario, solo para ver más despacio el movimiento.
    time.sleep(1./20.)

    # Asignamos nueva posición en cada iteración
    pos_target = [c,c,1]

    if (going):
        c=c+0.01
    else:
        c=c-0.01

    if (c>range_m[1]): going=False
    elif (c<range_m[0]): going=True

    # Calculamos los angulos/posiciones de los joints dependiendo del punto objetivo 'pos_target'

    jointPoses = p.calculateInverseKinematics(robotId, robotEndEffectorIndex, pos_target)

    for i in range(0,robotEndEffectorIndex):

        # setJointMotorControl2()
        # Permite configurar diferentes velocidades y fuerzas a las articulaciones.
        # POSITION_CONTROL
        # - Mueve el joint a una posición determinada con una velocidad fijada.
        # - Es necesario pasar targetPosition y targetVelocity.
        p.setJointMotorControl2(bodyIndex=robotId,
                                    jointIndex=i,
                                    controlMode=p.POSITION_CONTROL,
                                    targetPosition=jointPoses[i],
                                    targetVelocity=0,
                                    force=500,
                                    positionGain=0.03,
                                    velocityGain=1)


    # Pintamos la trayectoria que debería siguiendo nuestro brazo mecánico
    ls = p.getLinkState(robotId, robotEndEffectorIndex)
    if (hasPrevPose):
        p.addUserDebugLine(prevPose, pos_target, [0, 0, 0.3], 1, trailDuration)
        p.addUserDebugLine(prevPose1, ls[4], [1, 0, 0], 1, trailDuration)

    prevPose = pos_target
    prevPose1 = ls[4]
    hasPrevPose = 1

p.disconnect()