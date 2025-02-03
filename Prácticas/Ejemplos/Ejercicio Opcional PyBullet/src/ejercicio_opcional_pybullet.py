# Imports de las librerías
import pybullet as p
import pybullet_data
import argparse
import time

parser = argparse.ArgumentParser(description="URDF viewer example")
parser.add_argument("--urdf", type=str, required=True, help="Ruta al archivo URDF.")
args = parser.parse_args()
urdf_path = "urdf/ejercicio_opcional_pybullet.urdf"

# Conectamos motor con GUI
physicsClient = p.connect (p.GUI)
p.setAdditionalSearchPath (pybullet_data.getDataPath())

# Establecemos gravedad (X,Y,Z)
p.setGravity (0, 0, -9.8)

# Cargamos un/unos modelo/s
planeId = p.loadURDF ("plane.urdf")

startPosition = [0, 0, 1]
startOrientation = p.getQuaternionFromEuler([0, 0, 0])

# Cargamos un nuevo objeto, con una posición (x,y,z)
# y una orientación dada en cuaternión (X,Y,Z)
robotId = p.loadURDF (urdf_path, startPosition, startOrientation)

frictionIdBase = p.addUserDebugParameter("BASE_jointFriction", 0, 100, 10)
torqueIdBase = p.addUserDebugParameter("BASE_joint torque", -20, 20, -9)
frictionId = p.addUserDebugParameter("ARM_jointFriction", 0, 100, 10)
torqueId = p.addUserDebugParameter("ARM_joint torque", -20, 20, -9)

numJoints = p.getNumJoints(robotId)
print("NumJoints: " + str(numJoints))

for j in range (numJoints):
    print("%d - %s" % (p.getJointInfo(robotId,j)[0], p.getJointInfo(robotId,j)[1].decode("utf-8")))

for i in range (10000):

    frictionForceBase = p.readUserDebugParameter(frictionIdBase)
    jointTorqueBase = p.readUserDebugParameter(torqueIdBase)
    frictionForce = p.readUserDebugParameter(frictionId)
    jointTorque = p.readUserDebugParameter(torqueId)

    # setJointMotorControl2()
    # Permite configurar diferentes velocidades y fuerzas a las articulaciones.
    # VELOCITY_CONTROL
    # - Establece una velocidad fija al joint.
    # - Es necesario pasar targetVelocity y opcionalmente force.
    # - force define la fuerza motor (par motor, o momento de fuerza).
    p.setJointMotorControl2(robotId, 0, p.VELOCITY_CONTROL, targetVelocity=0, force=frictionForceBase)
    p.setJointMotorControl2(robotId, 1, p.VELOCITY_CONTROL, targetVelocity=0, force=frictionForce)

    # TORQUE_CONTROL
    # - Establece el torque o momento de fuerza instantáneamente sobre un joint.
    # - Es necesario pasar force que establece la fuerza.
    p.setJointMotorControl2(robotId, 0, p.TORQUE_CONTROL, force=jointTorqueBase)
    p.setJointMotorControl2(robotId, 1, p.TORQUE_CONTROL, force=jointTorque)

    p.stepSimulation()
    time.sleep(1./240.)

p.disconnect()