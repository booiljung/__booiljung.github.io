import numpy as np

a = np.array([
    [1, 2],
    [3, 4],
    [5, 6]
], np.float64)

b = np.array([
    [1, 2, 3],
    [4, 5, 6]
], np.float64)

c = np.matmul(a, b)

print("a =\n", a)
print("b =\n", b)
print("c =\n", c)

"""
a =
[[1. 2.]
 [3. 4.]
 [5. 6.]]

b =
[[1. 2. 3.]
 [4. 5. 6.]]

c =
[[ 9. 12. 15.]
 [19. 26. 33.]
 [29. 40. 51.]]

"""
