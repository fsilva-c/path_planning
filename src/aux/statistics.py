import pandas as pd
import numpy as np

df = pd.read_csv('experiments/classical/ex5_cen_1.csv')

# distancia total...
df['delta_x'] = df['x'].diff()
df['delta_y'] = df['y'].diff()
df['delta_z'] = df['z'].diff()
df['distance'] = np.sqrt(df['delta_x']**2 + df['delta_y']**2 + df['delta_z']**2)
distancia_total = df['distance'].sum()

# velocidade média...
mean_velocity = df['distance'].sum() / (df['timestamp'].iloc[-1] - df['timestamp'].iloc[0])

# tempo de voo...
fly_time = df['timestamp'].iloc[-1] - df['timestamp'].iloc[0]

# altitude média...
mean_height = df['z'].mean()

# variação de altitude...
var_height = df['z'].max() - df['z'].min()

# show statistics...
print(f'Distância total percorrida: {distancia_total} m')
print(f'Velocidade média: {mean_velocity} m/s')
print(f'Tempo de vôo: {fly_time} s')
print(f'Altitude média: {mean_height} m')
print(f'Variação de altitude: {var_height} m')

'''
import matplotlib.pyplot as plt

# Visualização 3D do Caminho:
from mpl_toolkits.mplot3d import Axes3D
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(df['x'], df['y'], df['z'])
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
plt.show()
'''
