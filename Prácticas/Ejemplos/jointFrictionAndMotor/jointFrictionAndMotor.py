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
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Cargamos un/unos modelo/s
door = p.loadURDF(urdf_path)

# p.changeDynamics()
# Cambia la diagonal inercial de la matriz programáticamente.
p.changeDynamics(door, -1, linearDamping=0, angularDamping=0)
for j in range(p.getNumJoints(door)):
  p.changeDynamics(door, j, linearDamping=0, angularDamping=0)
  print(p.getJointInfo(door, j))

frictionId = p.addUserDebugParameter("jointFriction", 0, 20, 10)
torqueId = p.addUserDebugParameter("joint torque", 0, 20, 5)

while (1):
  frictionForce = p.readUserDebugParameter(frictionId)
  jointTorque = p.readUserDebugParameter(torqueId)

  # setJointMotorControl2()
  # Permite configurar diferentes velocidades y fuerzas a las articulaciones.
  # VELOCITY_CONTROL
  # - Establece una velocidad fija al joint.
  # - Es necesario pasar targetVelocity y opcionalmente force.
  # - force define la fuerza motor (par motor, o momento de fuerza).
  p.setJointMotorControl2(door, 
                          1, 
                          p.VELOCITY_CONTROL, 
                          targetVelocity=0, 
                          force=frictionForce)
  
  # TORQUE_CONTROL
  # - Establece el torque o momento de fuerza instantáneamente sobre un joint.
  # - Es necesario pasar force que establece la fuerza.
  p.setJointMotorControl2(door, 
                          1, 
                          p.TORQUE_CONTROL, 
                          force=jointTorque)

  p.stepSimulation()
  time.sleep(0.01)
