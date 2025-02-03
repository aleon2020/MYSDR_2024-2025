# Imports de las librerías
import pybullet as p
import pybullet_data
import argparse
import time

parser = argparse.ArgumentParser(description="URDF viewer example")
parser.add_argument("--urdf", type=str, required=True, help="Ruta al archivo URDF.")
args = parser.parse_args()
urdf_path = args.urdf

# Conectamos motor con GUI
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Establecemos gravedad (X,Y,Z)
p.setGravity(0,0,-9.8)

# Cargamos un/unos modelo/s
planeId = p.loadURDF("plane_transparent.urdf")

startPos = [0,0,1]
startOrientation = p.getQuaternionFromEuler([0,0,-3.15])

# Cargamos un nuevo objeto, con una posición (x,y,z)
# y una orientación dada en cuaternión (X,Y,Z)
robotId = p.loadURDF(urdf_path,startPos, startOrientation)

# p.setRealTimeSimulation()
# El motor de físicas no realiza pausas en la simulación.
# Ejecuta en tiempo real acorde al RTC del sistema.
# Depende del rendimiento del sistema.
# p.setRealTimeSimulation(1)

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
            
except KeyboardInterrupt:
      pass
	
p.disconnect()    
