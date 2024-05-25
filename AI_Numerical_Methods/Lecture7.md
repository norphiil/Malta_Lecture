# Lecture 7

A = [
    1, 2, 4
    3, 8, 14
    2, 6, 13
]

Where
L = [
    1, 0, 0
    L21, 1, 0
    L31, L32, 1
]
and
U = [
    U11, U12, U13
    0, U22, U23
    0, 0, U33
]

A = LU

[
    1, 2, 4
    3, 8, 14
    2, 6, 13
] = [
    1, 0, 0
    L21, 1, 0
    L31, L32, 1
] [
    U11, U12, U13
    0, U22, U23
    0, 0, U33
]

[
    1, 2, 4
    3, 8, 14
    2, 6, 13
] = [
    U11, U12, U13
    L21U11, L21U12 + U22, L21U13 + U23
    L31U11, L31U12 + L32U22, L31U13 + L32U23 + U33
]

L = [
    1, 0, 0
    3, 1, 0
    2, 1, 1
]

U = [
    1, 2, 4
    0, 2, 2
    0, 0, 3
]
solve for Ax = b
Where
b = [
    1
    3
    2
]

LUx = b

LY = b

LY = [
    1, 0, 0
    3, 1, 0
    2, 1, 1
] [
    Y1
    Y2
    Y3
] = [
    1
    3
    2
]

Y1 = 1
Y2 = 3 - 3 = 0
Y3 = 2 - 2 = 0

Ux = Y

Y = [
    1
    3 - 3
    2 - 2
]  = [
    1
    0
    0
]

## A = LL^T

Are these matrices positive definite ?

S = [
    3, 4
    4, 5
]

the determinant of S is 3 \* 5 - 4 \* 4 = 15 - 16 = -1
Is not positive definite because the determinant is negative

S = [
    3, 4
    4, 6
]

the determinant of S is 3 \* 6 - 4 \* 4 = 18 - 16 = 2
Is positive definite

A = LL^T
Where A is symmetric positive definite and L is lower triangular

A = [
    a11, a22, a33
    a21, a22, a23
    a31, a32, a33
]

L = [
    l11, 0, 0
    l21, l22, 0
    l31, l32, l33
]

L^T = [
    l11, l21, l31
    0, l22, l32
    0, 0, l33
]

A = LL^T

[
    a11, a22, a33
    a21, a22, a23
    a31, a32, a33
] = [
    l11, 0, 0
    l21, l22, 0
    l31, l32, l33
] [
    l11, l21, l31
    0, l22, l32
    0, 0, l33
]

[
    a11, a22, a33
    a21, a22, a23
    a31, a32, a33
] = [
    l11^2, l11l21, l11l31
    l11l21, l21^2 + l22^2, l21l31 + l22l32
    l11l31, l21l31 + l22l32, l31^2 + l32^2 + l33^2
]

A = [
    25, 15, -5
    15, 18, 0
    -5, 0, 11
] = LL^T = [
    l11^2, l11l21, l11l31
    l11l21, l21^2 + l22^2, l21l31 + l22l32
    l11l31, l21l31 + l22l32, l31^2 + l32^2 + l33^2
]

A = [
    25, 15, -5
    15, 18, 0
    -5, 0, 11
] = [
    5, 0, 0
    3, 3, 0
    -1, 0, 1
] [
    5, 3, -1
    0, 3, 0
    0, 0, 1
]
