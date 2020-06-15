import numpy as np

dt = 1

A = np.eye(6)
A[[0, 1, 2], [3, 4, 5]] = dt  # discrete time constant

print(A)

A = np.eye(6)
A[([0, 3], [2, 5])] = dt  # discrete time constant

print(A)

B = np.vstack((0.5*dt**2*np.eye(3), dt*np.eye(3)))

print(B)
