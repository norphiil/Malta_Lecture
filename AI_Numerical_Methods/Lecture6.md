# Lecture 6

## Orthogonalization

v1 = [
    1,
    1,
    1
]

v2 = [
    0,
    1,
    1
]

v3 = [
    0,
    0,
    1
]

u1 = v1

u2 = v2 - (v2 . u1) / |u1|² \* u1

v2.u1 = [0, 1, 1] . [1, 1, 1] = 0 + 1 + 1 = 2

|u1|² = [1, 1, 1] . [1, 1, 1] = 1 + 1 + 1 = 3

u2 = [0, 1, 1] - 2/3 \* [1, 1, 1] = [0, 1, 1] - [2/3, 2/3, 2/3] = [-2/3, 1/3, 1/3]

u3 = v3 - (v3 . u1) / |u1|² \* u1 - (v3 . u2) / |u2|² \* u2

v3.u1 = [0, 0, 1] . [1, 1, 1] = 0 + 0 + 1 = 1

v3.u2 = [0, 0, 1] . [-2/3, 1/3, 1/3] = 0 + 0 + 1/3 = 1/3

|u2|² = [-2/3, 1/3, 1/3] . [-2/3, 1/3, 1/3] = 4/9 + 1/9 + 1/9 = 2/3

u3 = [0, 0, 1] - 1/3 \* [1, 1, 1] - 1/3 \* 3/2 \* [-2/3, 1/3, 1/3]