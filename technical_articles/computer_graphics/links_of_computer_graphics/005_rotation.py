import numpy as np
import transform_matrix as tm

rx = tm.rotation_x_matrix(90)
ry = tm.rotation_y_matrix(90)
rz = tm.rotation_z_matrix(90)

px = np.array([
    [1],
    [0],
    [0],
    [1]
])

py = np.array([
    [0],
    [1],
    [0],
    [1]
])

pz = np.array([
    [0],
    [0],
    [1],
    [1]
])

print("rotation x axis rx * px =", np.matmul(rx, px))
print("rotation y axis ry * px =", np.matmul(ry, px))
print("rotation z axis rz * px =", np.matmul(rz, px))


