import numpy as np
import matplotlib.pyplot as plt

theta1 = np.linspace(0, np.pi, 20)
theta2 = np.linspace(-np.pi/2, np.pi/2, 20)
x = np.array([])
y = np.array([])

for t1 in theta1:
    for t2 in theta2:
        x = np.append(x, 80*np.cos(t1) + 65.5* np.cos(t1 + t2))
        y = np.append(y, 80*np.sin(t1) + 65.5* np.sin(t1 + t2))

plt.scatter(x,y)
plt.show()