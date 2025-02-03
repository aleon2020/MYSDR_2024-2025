# Imports de las librerías
import pybullet as p
import pybullet_data
import argparse
import time

parser = argparse.ArgumentParser(description="URDF viewer example")
parser.add_argument("--urdf", type=str, required=True, help="Ruta al archivo URDF.")
args = parser.parse_args()
urdf_path = "urdf/door.urdf"

# Conectamos motor con GUI
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Establecemos gravedad (X,Y,Z)
p.setGravity(0, 0, -9.8)

# Cargamos un/unos modelo/s
planeId = p.loadURDF("plane.urdf")
robotId = p.loadURDF("r2d2.urdf",[0.5, -1, 1])
doorId = p.loadURDF(urdf_path)

# p.changeDynamics()
# Cambia la diagonal inercial de la matriz programáticamente.
p.changeDynamics(doorId, -1, linearDamping=0, angularDamping=0)
for j in range(p.getNumJoints(doorId)):
  p.changeDynamics(doorId, j, linearDamping=0, angularDamping=0)
  print(p.getJointInfo(doorId, j))

# Añadir parámetros en la pestaña "params".
# Útiles para cambiar dinámicas en tiempo real.
frictionId = p.addUserDebugParameter("DOOR_jointFriction", 0, 100, 10)
torqueId = p.addUserDebugParameter("DOOR_joint torque", -20, 20, -9)

numJoints = p.getNumJoints(robotId)
print("NumJoints: " + str(numJoints))

for j in range (numJoints):
    print("%d - %s" % (p.getJointInfo(robotId,j)[0], p.getJointInfo(robotId,j)[1].decode("utf-8")))

joints = [2, 3, 6, 7]

speedId = p.addUserDebugParameter("R2D2_speed", 0, 40, 5)
forceId = p.addUserDebugParameter("R2D2_force", 0, 40, 5)

# p.stepSimulation()
# La simulación avanza solo un "step" (paso) de acuerdo con los pasos 
# establecidos en setTimeStep. Es últil cuando se quiere controlar la
# simulación donde se necesita procesar antes de avanzar al siguiente.

while (1):

  # Desde el bucle de control, puedes obtener los cambios 
  # del parámetro de depuración de la siguiente manera.
  speed = p.readUserDebugParameter(speedId)
  torque = p.readUserDebugParameter(forceId)

  # setJointMotorControlArray()
  # Usado cuando el movimiento está relacionado con la activación
  # de varios joints simultáneamente.
  p.setJointMotorControlArray(robotId,
                              joints,
                              p.VELOCITY_CONTROL,
                              targetVelocities=[-speed, -speed, -speed, -speed],
                              forces=[torque, torque, torque, torque])

  frictionForce = p.readUserDebugParameter(frictionId)
  jointTorque = p.readUserDebugParameter(torqueId)

  # setJointMotorControl2()
  # Permite configurar diferentes velocidades y fuerzas a las articulaciones.
  # VELOCITY_CONTROL
  # - Establece una velocidad fija al joint.
  # - Es necesario pasar targetVelocity y opcionalmente force.
  # - force define la fuerza motor (par motor, o momento de fuerza).
  p.setJointMotorControl2(doorId, 
                          1, 
                          p.VELOCITY_CONTROL, 
                          targetVelocity=0, 
                          force=frictionForce)
  
  # TORQUE_CONTROL
  # - Establece el torque o momento de fuerza instantáneamente sobre un joint.
  # - Es necesario pasar force que establece la fuerza.
  p.setJointMotorControl2(doorId, 
                          1, 
                          p.TORQUE_CONTROL, 
                          force=jointTorque)

  p.stepSimulation()
  time.sleep(0.01)
