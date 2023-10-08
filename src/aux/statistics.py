import pandas as pd
import numpy as np

df = pd.read_csv('experiments/classical/Teste 1.csv')

# distancia total
df['delta_x'] = df['x'].diff()
df['delta_y'] = df['y'].diff()
df['delta_z'] = df['z'].diff()
df['distance'] = np.sqrt(df['delta_x']**2 + df['delta_y']**2 + df['delta_z']**2)
distancia_total = df['distance'].sum()

# velocidade média...
mean_velocity = df['distance'].sum() / (df['timestamp'].iloc[-1] - df['timestamp'].iloc[0])

# tempo de voo
fly_time = df['timestamp'].iloc[-1] - df['timestamp'].iloc[0]

# show statistics...
print(f'Distância total percorrida: {distancia_total} m')
print(f'Velocidade média: {mean_velocity} m/s')
print(f'Tempo de vôo: {fly_time} s')
