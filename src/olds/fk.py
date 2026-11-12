from .transforms import (
    T01,
    T12,
    T23,
    T34,
    T45,
    T56
)


def forward_kinematics(q):

    theta1, theta2, theta3, theta4, theta5, theta6 = q

    T = (
        T01(theta1)
        @ T12(theta2)
        @ T23(theta3)
        @ T34(theta4)
        @ T45(theta5)
        @ T56(theta6)
    )

    return T

# @ é o operador de multiplicação de matrizes do numpy