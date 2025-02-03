# Imports de las librerías
import pybullet as p
import pybullet_data
import argparse
import time

# Conectamos motor con GUI
cid = p.connect(p.SHARED_MEMORY)
if (cid < 0):
  cid = p.connect(p.GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath())
restitutionId = p.addUserDebugParameter("restitution", 0, 1, 1)
restitutionVelocityThresholdId = p.addUserDebugParameter("res. vel. threshold", 0, 3, 0.2)

# Añadir parámetros en la pestaña "params".
# Útiles para cambiar dinámicas en tiempo real.
lateralFrictionId = p.addUserDebugParameter("lateral friction", 0, 1, 0.5)
spinningFrictionId = p.addUserDebugParameter("spinning friction", 0, 1, 0.03)
rollingFrictionId = p.addUserDebugParameter("rolling friction", 0, 1, 0.03)

# Cargamos un/unos modelo/s
plane = p.loadURDF("plane_transparent.urdf")
sphere = p.loadURDF("sphere_with_restitution.urdf", [0, 0, 2])

# p.setRealTimeSimulation()
# El motor de físicas no realiza pausas en la simulación.
# Ejecuta en tiempo real acorde al RTC del sistema.
# Depende del rendimiento del sistema.
p.setRealTimeSimulation(1)

# Establecemos gravedad (X,Y,Z)
p.setGravity(0, 0, -9.8)

while (1):

  # Desde el bucle de control, puedes obtener los cambios 
  # del parámetro de depuración de la siguiente manera.
  restitution = p.readUserDebugParameter(restitutionId)
  restitutionVelocityThreshold = p.readUserDebugParameter(restitutionVelocityThresholdId)

  p.setPhysicsEngineParameter(restitutionVelocityThreshold=restitutionVelocityThreshold)

  # Desde el bucle de control, puedes obtener los cambios 
  # del parámetro de depuración de la siguiente manera.
  lateralFriction = p.readUserDebugParameter(lateralFrictionId)
  spinningFriction = p.readUserDebugParameter(spinningFrictionId)
  rollingFriction = p.readUserDebugParameter(rollingFrictionId)
  
  p.changeDynamics(plane, -1, lateralFriction=1)
  p.changeDynamics(sphere, -1, lateralFriction=lateralFriction)
  p.changeDynamics(sphere, -1, spinningFriction=spinningFriction)
  p.changeDynamics(sphere, -1, rollingFriction=rollingFriction)

  p.changeDynamics(plane, -1, restitution=restitution)
  p.changeDynamics(sphere, -1, restitution=restitution)
  pos, orn = p.getBasePositionAndOrientation(sphere)
  # print("pos=")
  # print(pos)
  time.sleep(0.01)
