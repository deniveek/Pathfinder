import numpy as np


def dist(point, start, end):
    if np.all(np.equal(start, end)):
        return np.linalg.norm(point - start)

    return np.divide(
        np.abs(np.linalg.norm(np.cross(end - start, start - point))),
        np.linalg.norm(end - start))


def DouglasPeucker(array, epsilon=5):
    d_max = 0
    idx_d_max = 0
    for i in range(1, len(array)-1):
        d = dist(array[i], array[0], array[-1])
        if d > d_max:
            d_max = d
            idx_d_max = i

    if d_max > epsilon:
        res_1 = DouglasPeucker(array[:idx_d_max+1], epsilon)
        res_2 = DouglasPeucker(array[idx_d_max:], epsilon)
        res = np.concatenate([res_1[:-1], res_2])
    else:
        res = np.array([array[0], array[-1]])
    return res