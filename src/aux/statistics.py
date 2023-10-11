import pandas as pd
import numpy as np

df = pd.read_csv('experiments/classical/Teste 1.csv')

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
print(f'Vriação de altitude: {var_height} m')


# Análise de Velocidade Instantânea:
import matplotlib.pyplot as plt
velocidade_instantanea = ((df['x'].diff())**2 + (df['y'].diff())**2 + (df['z'].diff())**2)**0.5 / df['timestamp'].diff()
# Plote um gráfico da velocidade instantânea em relação ao tempo
plt.plot(df['timestamp'], velocidade_instantanea)
plt.xlabel('Timestamp')
plt.ylabel('Velocidade (m/s)')
plt.title('Velocidade Instantânea ao Longo do Tempo')
plt.show()

# Visualização 3D do Caminho:
from mpl_toolkits.mplot3d import Axes3D
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(df['x'], df['y'], df['z'])
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
plt.show()


