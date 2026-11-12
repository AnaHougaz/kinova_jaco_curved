import numpy as np

from .parameters import D2, e2, WRIST_LENGTH


def T01(theta1):
    c = np.cos(theta1)
    s = np.sin(theta1)

    return np.array([
        [c, -s, 0, 0],
        [s,  c, 0, 0],
        [0,  0, 1, 0],
        [0,  0, 0, 1]
    ])


def T12(theta2):
    c = np.cos(theta2)
    s = np.sin(theta2)

    return np.array([
        [c, -s, 0, 0],
        [0,  0, -1, 0],
        [s,  c, 0, 0],
        [0,  0, 0, 1]
    ])


def T23(theta3):
    c = np.cos(theta3)
    s = np.sin(theta3)

    return np.array([
        [c, -s, 0, D2],
        [s,  c, 0, 0],
        [0,  0, 1, e2],
        [0,  0, 0, 1]
    ])


def T34(theta4):
    c = np.cos(theta4)
    s = np.sin(theta4)

    return np.array([
        [c, -s, 0, 0],
        [0,  0, 1, WRIST_LENGTH],
        [-s, -c, 0, 0],
        [0,  0, 0, 1]
    ])


def T45(theta5):
    c = np.cos(theta5)
    s = np.sin(theta5)

    return np.array([
        [c, -s, 0, 0],
        [0,  0, -1, 0],
        [s,  c, 0, 0],
        [0,  0, 0, 1]
    ])


def T56(theta6):
    c = np.cos(theta6)
    s = np.sin(theta6)

    return np.array([
        [c, -s, 0, 0],
        [0,  0, 1, 0],
        [-s, -c, 0, 0],
        [0,  0, 0, 1]
    ])