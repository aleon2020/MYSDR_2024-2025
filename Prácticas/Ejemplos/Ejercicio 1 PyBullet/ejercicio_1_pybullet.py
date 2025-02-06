# Imports de las librerías.
import pybullet as p
import pybullet_data
import time
import math

# Conectamos motor con GUI.
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Diferente gravedad (X,Y,Z).

# Gravedad de Marte.
p.setGravity(0, 0, -3.721)

# Gravedad de la Luna.
# p.setGravity(0, 0, -1.625)

# Gravedad de la Tierra.
# p.setGravity(0, 0, -9.8)

# Cargamos un modelo (plano).
planeID = p.loadURDF("plane.urdf")

# Diferentes orientaciones utilizando ángulos de Euler.

# Orientaciones iniciales.
# euler_angles = [0, 0, 0]
# startOrientation = p.getQuaternionFromEuler(euler_angles)

# Nuevas orientaciones.
# Orientación del robot 45º a la derecha sobre su eje Y.
# Orientación del robot 90º a la izquierda sobre su eje Z.
euler_angles = [0, 45 * (math.pi / 180), 90 * (math.pi / 180)]
startOrientation = p.getQuaternionFromEuler(euler_angles)

# Diferentes posiciones (x,y,z) del robot.

# Posición inicial.
# startPosition = [0, 0, 1]

# Nueva posición.
startPosition = [1, 1, 1]

# Cargamos un nuevo objeto, con una posición (x,y,z)
# y una orientación dada en cuaternión (C,X,Y,Z).
robotId = p.loadURDF("r2d2.urdf", startPosition, startOrientation)

# Bucle principal que ejecuta los pasos de la simulación.
# Por defecto utilizaremos siempre time step de 1/240 segundos.
for i in range (10000):
    p.stepSimulation()
    time.sleep(1./240.)

p.disconnect()