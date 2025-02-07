# Imports de las librerías.
import pybullet as p
import pybullet_data
import argparse
import time

parser = argparse.ArgumentParser(description="URDF viewer example")
parser.add_argument("--urdf", type=str, required=True, help="Ruta al archivo URDF.")
args = parser.parse_args()

# Datos iniciales.
y_o = 3.0
v = 0.0
a = -9.8
t = 0.01

# Conectamos motor con GUI.
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Establecemos gravedad (X,Y,Z).
p.setGravity(0, 0, 0)

# Cargamos un/unos modelo/s.
planeId = p.loadURDF("plane.urdf")

startPosition = [0, 0, 0.3]
startOrientation = p.getQuaternionFromEuler([0, 0, 0])

# Cargamos un nuevo objeto, con una posición (x,y,z)
# y una orientación dada en cuaternión (X,Y,Z).
sphereId = p.loadURDF("urdf/ejercicio_opcional_gravedad.urdf", startPosition, startOrientation)

# Simulación manual de la caída libre
y = y_o

while (1):

    # Fórmulas MRUA
    y = y + v * t + 0.5 * a * t ** 2
    v = v + a * t
    
    # Evita que la esfera pase por debajo del suelo
    if y <= 0.1:
        y = 0
        v = 0
        y = 0.1
    
    # Actualizar posición de la esfera
    p.resetBasePositionAndOrientation(sphereId, [0, 0, y], [0, 0, 0, 1])
    
    time.sleep(t)

p.disconnect()
