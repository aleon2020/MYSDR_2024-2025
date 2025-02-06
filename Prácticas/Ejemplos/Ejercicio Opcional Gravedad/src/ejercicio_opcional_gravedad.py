# Imports de las librerías.
import pybullet as p
import pybullet_data
import argparse
import time

parser = argparse.ArgumentParser(description="URDF viewer example")
parser.add_argument("--urdf", type=str, required=True, help="Ruta al archivo URDF.")
args = parser.parse_args()

dt = 0.01
altura_inicial = 3.0
velocidad = 0.0

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
esferaId = p.loadURDF("urdf/ejercicio_opcional_gravedad.urdf", startPosition, startOrientation)

# Simulación manual de la caída libre
altura = altura_inicial

while (1):

    # Fórmulas MRUA
    altura = altura + velocidad * dt + 0.5 * (-9.8) * dt ** 2
    velocidad = velocidad + (-9.8) * dt
    
    # Evitar que la esfera pase por debajo del suelo
    if altura <= 0.1:
        altura = 0
        velocidad = 0
        altura = 0.1
    
    # Actualizar posición de la esfera
    p.resetBasePositionAndOrientation(esferaId, [0, 0, altura], [0, 0, 0, 1])
    
    # Pequeña pausa para visualizar la simulación en tiempo real
    time.sleep(dt)

# Desconectar pybullet
p.disconnect()
