import numpy as np
import transform_matrix as tm

p1 = np.array([
    [0],
    [0],
    [0],
    [1]
])

t1 = tm.translation_matrix(1, 2, 3)

p2 = np.matmul(t1, p1)

print("translation p1 * t1 =", p2)

