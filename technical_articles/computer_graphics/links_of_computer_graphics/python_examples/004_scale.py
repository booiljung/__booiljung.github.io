import numpy as np
import transform_matrix as tm

p1 = np.array([
    [1],
    [1],
    [1],
    [1]
])

t1 = tm.scale_matrix(1, 2, 3)

p2 = np.matmul(t1, p1)

print("scale p1 * t1 =", p2)

