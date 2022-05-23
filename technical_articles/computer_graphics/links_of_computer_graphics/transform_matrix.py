from math import sin, cos, pi
import numpy as np

def identity_matrix():
    return np.array([
        [ 1, 0, 0, 0 ],
        [ 0, 1, 0, 0 ],
        [ 0, 0, 1, 0 ],
        [ 0, 0, 0, 1 ]
    ], np.float64)


def scale_matrix(sx, sy, sz):
    return np.array([
        [ sx,  0,  0, 0 ],
        [  0, sy,  0, 0 ],
        [  0,  0, sz, 0 ],
        [  0,  0,  0, 1 ]
    ], np.float64)


def translation_matrix(tx, ty, tz):
    return np.array([
        [ 1, 0, 0, tx ],
        [ 0, 1, 0, ty ],
        [ 0, 0, 1, tz ],
        [ 0, 0, 0, 1 ]
    ], np.float64)


def rotation_x_matrix(theta):
    return np.array([
        [ 1,                 0,                  0, 0 ],
        [ 0, cos(theta*pi/180),  sin(theta*pi/180), 0 ],
        [ 0, -sin(theta*pi/180), cos(theta*pi/180), 0 ],
        [ 0,                 0,                  0, 1 ]
    ], np.float64)


def rotation_y_matrix(theta):
    return np.array([
        [ cos(theta*pi/180), 0, -sin(theta*pi/180), 0 ],
        [                 0, 1,                  0, 0 ],
        [ sin(theta*pi/180), 0,  cos(theta*pi/180), 0 ],
        [                 0, 0,                  0, 1 ]
    ], np.float64)


def rotation_z_matrix(theta):
    return np.array([
        [  cos(theta*pi/180), sin(theta*pi/180), 0, 0 ],
        [ -sin(theta*pi/180), cos(theta*pi/180), 0, 0 ],
        [                  0,                 0, 1, 0 ],
        [                  0,                 0, 0, 1 ]
    ], np.float64)


def rotation_xyz_matrix(ex, ey, ez):
    rx = rotation_x_matrix(ex)
    ry = rotation_y_matrix(ey)
    rz = rotation_z_matrix(ez)
    return np.matmul(np.matmul(rx, ry), rz)


