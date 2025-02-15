import math
import numpy as np
import matplotlib.pyplot as plt

file = ['Fase2.csv']

plt.figure(figsize=(10, 6))

for i in file:
    with open(i, 'r') as file:
        lines = file.readlines()
        headers = lines[0].strip().split(',')
        data = np.genfromtxt(lines[1:], delimiter=',', dtype=float)
    posicion_index = headers.index('posicion_robot[Y]')
    velocidad_index = headers.index('velocidad_robot[Y]')
    posicion_robot_y = data[:, posicion_index]
    velocidad_robot_y = data[:, velocidad_index]
    plt.plot(posicion_robot_y, velocidad_robot_y, marker='', linestyle='-', label=i)

plt.xlabel('POSICIÓN')
plt.ylabel('VELOCIDAD')
plt.title('FASE 2: OBTENCIÓN DE MÉTRICAS')
plt.axis('auto')
plt.grid(True)
plt.legend()
plt.show()