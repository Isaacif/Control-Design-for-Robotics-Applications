import numpy as np

def find_closest_angles(x1, y1, X, Y, Theta1, Theta2, error_limit=1e-2):
    # Compute the Euclidean distance between (x1, y1) and all points in (X, Y)
    distances = np.sqrt((X - x1)**2 + (Y - y1)**2)
    print(distances)
    # Find the index of the minimum distance
    min_distance_index = np.unravel_index(np.argmin(distances), distances.shape)
    min_distance = distances[min_distance_index]
    
    # Check if the minimum distance is within the error limit
    if min_distance <= error_limit:
        # Return the corresponding (theta1, theta2) values
        return Theta1[min_distance_index], Theta2[min_distance_index]
    else:
        # Return None if no point is within the error limit
        return None

# Example usage
L1 = 80
L2 = 65.5

theta1 = np.linspace(0, np.pi, 120)
theta2 = np.linspace(-np.pi/2, np.pi/2, 120)
Theta1, Theta2 = np.meshgrid(theta1, theta2)

# Calculate the x and y coordinates of the end-effector
X = L1 * np.cos(Theta1) + L2 * np.cos(Theta1 + Theta2)
Y = L1 * np.sin(Theta1) + L2 * np.sin(Theta1 + Theta2) + 134.75

# Find closest angles to a given (x1, y1) with a specified error limit
x1, y1 = 90, 180  # Example target coordinates
angles = find_closest_angles(x1, y1, X, Y, Theta1, Theta2, error_limit=1.0)

if angles:
    print(f"Closest angles: theta1 = {angles[0]}, theta2 = {angles[1]}")
else:
    print("No close point found within the error limit.")
