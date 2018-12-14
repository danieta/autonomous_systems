#%%

import numpy as np
from scipy import io as sio
import matplotlib.pyplot as plt

data = sio.loadmat('test')

positions = data['beliefs'][:-1,0:2]

plt.figure()
plt.scatter(positions[:,0], positions[:,1], c=data['matches'][:-1], linewidth=5)
plt.plot(positions[:,0], positions[:,1], 'b,-')
plt.show()