# Imports from libraries
import numpy as np
import matplotlib.pyplot as plt

# Files in CSV format to process
# file = ['Fase2.csv']
# file = ['Fase3.1.csv']
# file = ['Fase3.2.csv']
# file = ['Fase3.3.csv']
# file = ['Fase3.1.csv', 'Fase3.2.csv', 'Fase3.3.csv']
# file = ['Fase3.1.csv', 'Fase3.2.csv']
# file = ['Fase4.csv']
file = ['Fase3.csv', 'Fase4.csv']

# Size of the figure to display the graph
plt.figure(figsize=(8, 4))

# Processing of CSV file data
for i in file:
    with open(i, 'r') as file:
        lines = file.readlines()
        headers = lines[0].strip().split(',')
        data = np.genfromtxt(lines[1:], delimiter=',', dtype=float)

    # Extract data from the robot position and speed columns
    y_robot_position = data[:, headers.index('posicion_robot[Y]')]
    y_robot_speed = data[:, headers.index('velocidad_robot[Y]')]

    # Generate the graph based on the position and speed of the robot
    plt.plot(y_robot_position, y_robot_speed, marker='', linestyle='-', label=i)

# Setting axis and chart titles
plt.xlabel('POSICIÓN (METROS)')
plt.ylabel('VELOCIDAD (METROS / SEGUNDO)')
# plt.title('FASE 2: OBTENCIÓN DE MÉTRICAS')
# plt.title('FASE 3 ESCENARIO 3.1: ASIGNACIÓN VELOCIDADES Y FUERZAS')
# plt.title('FASE 3 ESCENARIO 3.2: ASIGNACIÓN VELOCIDADES Y FUERZAS + FRICCIÓN')
# plt.title('FASE 3 ESCENARIO 3.3: ASIGNACIÓN VELOCIDADES Y FUERZAS + FRICCIÓN + INERCIA')
# plt.title('FASE 3: EVALUACIÓN DE MÉTRICAS')
# plt.title('FASE 4: CONTROLADOR DINÁMICO DEL ROBOT')
plt.title('FASE 3 ESCENARIO 3.3 VS FASE 4')

# Adjustment / Settings of the axes and grid
plt.axis('auto')
plt.grid(True)

# Legend display
plt.legend()

# Graph display
plt.show()
