import pybullet as p
import pybullet_data
import time

def simulate_falling_sphere():
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, 0)  # Desactivamos la gravedad del motor de Bullet
    
    # Cargar el suelo
    plane_id = p.loadURDF("plane.urdf")
    
    # Crear la esfera en 3 metros de altura
    sphere_radius = 0.1
    sphere_start_height = 3.0
    sphere_id = p.loadURDF("sphere_small.urdf", [0, 0, sphere_start_height])
    
    # Parámetros de simulación
    g = -9.81  # Gravedad (m/s^2)
    dt = 1/240  # Paso de tiempo de la simulación
    velocity = 0  # Velocidad inicial
    position = sphere_start_height  # Posición inicial
    
    while position > sphere_radius:
        velocity += g * dt  # v = v0 + a*t
        position += velocity * dt  # x = x0 + v*t + (1/2)*a*t^2 (simplificado por el bucle)
        
        # Actualizar la posición de la esfera en PyBullet
        p.resetBasePositionAndOrientation(sphere_id, [0, 0, position], [0, 0, 0, 1])
        
        p.stepSimulation()
        time.sleep(dt)  # Simular en tiempo real
    
    print("La esfera ha tocado el suelo.")
    time.sleep(2)
    p.disconnect()

if __name__ == "__main__":
    simulate_falling_sphere()

