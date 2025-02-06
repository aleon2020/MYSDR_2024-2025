# Imports de las librerías.
import pybullet as p
import pybullet_data
import time
import math

# Conectamos motor con GUI.
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Establecemos gravedad (X,Y,Z).
p.setGravity(0, 0, -9.8)

# Cargamos un modelo (plano).
planeID = p.loadURDF("plane.urdf")

euler_angles = [0, 0, 0]
startOrientation = p.getQuaternionFromEuler(euler_angles)
startPosition = [0, 0, 1]

# Cargamos un nuevo objeto, con una posición (x,y,z)
# y una orientación dada en cuaternión (C,X,Y,Z).
robotId = p.loadURDF("r2d2.urdf", startPosition, startOrientation)

# Generación de 6 parámetros de depuración para traslación y rotación
# de un objeto de la escena.
tx = p.addUserDebugParameter("x_pos", -5, 5, 0)
ty = p.addUserDebugParameter("y_pos", -5, 5, 0)
tz = p.addUserDebugParameter("z_pos", 0.5, 5, 0.5)
rx = p.addUserDebugParameter("x_euler", 0, 2 * math.pi, 0)
ry = p.addUserDebugParameter("y_euler", 0, 2 * math.pi, 0)
rz = p.addUserDebugParameter("z_euler", 0, 2 * math.pi, 0)

# Bucle principal que ejecuta los pasos de la simulación.
# Por defecto utilizaremos siempre time step de 1/240 segundos.
while True:

    # Al modificar los sliders, el robot cambia en tiempo real
    # su posición y su orientación.
    pos_x = p.readUserDebugParameter(tx)
    pos_y = p.readUserDebugParameter(ty)
    pos_z = p.readUserDebugParameter(tz)
    rot_x = p.readUserDebugParameter(rx)
    rot_y = p.readUserDebugParameter(ry)
    rot_z = p.readUserDebugParameter(rz)

    # Ángulos de Euler en radianes.
    orientation = p.getQuaternionFromEuler([rot_x, rot_y, rot_z])

    # Simulación del Bloqueo de Cardán.
    if abs(rot_y) > math.pi / 2 - 0.01:
        print("¡Bloqueo de Cardán detectado!")

    # Asignación de la posición y la orientación.
    p.resetBasePositionAndOrientation(robotId, [pos_x, pos_y, pos_z], orientation)

    p.stepSimulation()
    time.sleep(1./240.)

p.disconnect()