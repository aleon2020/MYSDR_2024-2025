# Imports de las librerías
import pybullet as p
import pybullet_data
import argparse
import time

parser = argparse.ArgumentParser(description="URDF viewer example")
parser.add_argument("--urdf", type=str, required=True, help="Ruta al archivo URDF.")
args = parser.parse_args()
urdf_path = "urdf/example_continuous.urdf"

# Conectamos motor con GUI
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Establecemos gravedad (X,Y,Z)
p.setGravity(0, 0, -9.8)

# Cargamos un/unos modelo/s
planeId = p.loadURDF("plane_transparent.urdf")

startPos = [0, 0, 1]
startOrientation = p.getQuaternionFromEuler([0, 0, -3.15])

# Cargamos un nuevo objeto, con una posición (x,y,z)
# y una orientación dada en cuaternión (X,Y,Z)
robotId = p.loadURDF(urdf_path, startPos, startOrientation)

numJoints = p.getNumJoints(robotId)
print("NumJoints: {}".format(numJoints))

for j in range(numJoints):
     print("{} - {}".format(p.getJointInfo(robotId, j)[0], p.getJointInfo(robotId, j)[1].decode("utf-8")))

# Añadir parámetros en la pestaña "params".
# Útiles para cambiar dinámicas en tiempo real.
right_id = p.addUserDebugParameter("rightMotor", -1, 1, 0)
left_id = p.addUserDebugParameter("leftMotor", -1, 1, 0)

# p.stepSimulation()
# La simulación avanza solo un "step" (paso) de acuerdo con los pasos 
# establecidos en setTimeStep. Es últil cuando se quiere controlar la
# simulación donde se necesita procesar antes de avanzar al siguiente.

try:

    # Bucle principal que ejecuta los pasos de la simulación.
    # Por defecto utilizaremos siempre time step de 1/240 segundos.
    while True:
        p.stepSimulation()
        time.sleep(1./240.)

        # Desde el bucle de control, puedes obtener los cambios 
        # del parámetro de depuración de la siguiente manera.
        right_motor_value = p.readUserDebugParameter(right_id)
        left_motor_value = p.readUserDebugParameter(left_id)
        
        # setJointMotorControl2()
        # Permite configurar diferentes velocidades y fuerzas a las articulaciones.
        p.setJointMotorControl2(robotId, 1, p.VELOCITY_CONTROL, targetVelocity=right_motor_value)
        p.setJointMotorControl2(robotId, 0, p.VELOCITY_CONTROL, targetVelocity=left_motor_value)
        
except KeyboardInterrupt:
      pass
	
p.disconnect()    