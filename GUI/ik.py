
import math 
from klampt.math import vectorops,so3

import klampt
from klampt.model import ik


def solve_2R_inverse_kinematics(x,y,L1=1,L2=1):
    """For a 2R arm centered at the origin, solves for the joint angles
    (q1,q2) that places the end effector at (x,y).

    The result is a list of up to 2 solutions, e.g. [(q1,q2),(q1',q2')].
    """
    y-=1.08
    D = vectorops.norm((x,y))
    print(f"D value {D}")
    thetades = math.atan2(y,x)
    if D == 0:
        raise ValueError("(x,y) at origin, infinite # of solutions")
    c2 = (D**2-0.70128)/(0.6864)
    q2s = []
    if c2 < -1:
        print("solve_2R_inverse_kinematics: (x,y) inside inner circle")
        return []
    elif c2 > 1:
        print("solve_2R_inverse_kinematics: (x,y) out of reach")
        return []
    else:
        if c2 == 1:
            q2s = [math.acos(c2)]
        else:
            q2s = [math.acos(c2),-math.acos(c2)]
    res = []
    for q2 in q2s:
        thetaactual = math.atan2(math.sin(q2),0.65+0.528*math.cos(q2))
        q1 = thetades - thetaactual
        res.append((q1,q2))
    return res

print(solve_2R_inverse_kinematics(0/127, 280/127))


import numpy as np
import matplotlib.pyplot as plt

# Link lengths
L1 = 80
L2 = 65.5

# Range of angles from 0 to pi for both joints
theta1 = np.linspace(0, np.pi, 120)  # Joint 1 angles
theta2 = np.linspace(-np.pi/2, np.pi/2, 120)  # Joint 2 angles

# Create a meshgrid of joint angles
Theta1, Theta2 = np.meshgrid(theta1, theta2)

# Calculate the x and y coordinates of the end-effector
X = L1 * np.cos(Theta1) + L2 * np.cos(Theta1 + Theta2)
Y = L1 * np.sin(Theta1) + L2 * np.sin(Theta1 + Theta2) + 134.75

# Plotting the reachable workspace

import matplotlib.pyplot as plt

# Plotting the reachable workspace
plt.figure(figsize=(8, 8))
plt.plot(X, Y, '.', markersize=1, color='blue')  # Replace X and Y with your data
plt.xlabel('Eixo Y (mm)')
plt.ylabel('Eixo Z (mm)')
plt.ylim(-20, 320)
plt.xlim(-185, 185)

# Plotting other lines and points
plt.plot([-185, 185], [281, 281], color='g', linestyle=(0, (5, 10)))
plt.plot([80, 80], [-20, 65], color='r', linestyle=(0, (5, 10)))
plt.plot([185, 80], [65, 65], color='r', linestyle=(0, (5, 10)))
plt.plot([-80, -80], [-20, 65], color='r', linestyle=(0, (5, 10)))
plt.plot([-185, -80], [65, 65], color='r', linestyle=(0, (5, 10)))
plt.title("Área de Trabalho Robô didático")
plt.grid(True)

# Highlighted points
highlight_x = -80
highlight_y = 64
plt.scatter(0, 283.25, color='green', s=80, label='Pontos máximo')
plt.text(-3, 283, f'({0}, {280})', fontsize=13, color='green', ha='right', va='bottom')
plt.scatter(-highlight_x, highlight_y+1, color='red', s=80, label='Pontos mínimos')
plt.text(-highlight_x-5, highlight_y-5, f'({-highlight_x}, {69.25})', fontsize=13, color='red', ha='right', va='bottom')
plt.scatter(highlight_x, highlight_y, color='red', s=80)
plt.text(highlight_x+5, highlight_y-5, f'({highlight_x}, {69.25})', fontsize=13, color='red', ha='left', va='bottom')

# Creating and adding circle patches
inner_circle = plt.Circle((0, 134.75), 142.5, color='green', linewidth=4, fill=False, label="Raio Externo ")
outer_circle = plt.Circle((0, 134.75), 14.5, color='yellow', linewidth=4,fill=False, label="Raio Interno")

# Adding the circles to the plot
ax = plt.gca()  # Get the current axes
ax.add_patch(inner_circle)
ax.add_patch(outer_circle)

# Ensure equal scaling and proper limits after adding the patches
ax.set_aspect('equal', adjustable='box')

plt.legend(loc='lower right')
plt.show()
