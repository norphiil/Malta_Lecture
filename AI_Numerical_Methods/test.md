# Quiz 2

## Exercise 1

With the matrix

A = [
    4, 1
    0, 4
]

for check if is diagonalisable

the eigenvalues of A are

det(A-λI) = 0

det([4-λ, 1
    0, 4-λ]) = 0

(4-λ)(4-λ) = 0

λ = 4

the eigenvectors of A are

A - 4I = [
    0, 1
    0, 0
]

v1 = [1, 0]

So, A is diagonalisable

## Exercise 2

The transformation matrix
A=[
    1,2
    3,2
]

the eigenvalues of A:

det(A-λI) = 0

det([1-λ, 2
    3, 2-λ]) = 0

(1-λ)(2-λ) - 6 = 0

λ² - 3λ - 4 = 0

(λ - 4)(λ + 1) = 0

λ = 4, λ = -1

the eigenvalues of A are λ = 4, λ = -1

the eigenvectors of A:

for λ = 4

A - 4I = [
    -3, 2
    3, -2
]

v1 = [2, 3]

for λ = -1

A + I = [
    2, 2
    3, 3
]

v2 = [1, -1]

can be diagonalised into A = CDC^(−1)

Given that C=[
    2,1
    3,−1
]

D=[
    4,0
    0,−1
]

## Exercise 3

The transformation matrix
A=[
    4,1
    2,3
]

the eigenvalues of A:

det(A-λI) = 0

det([4-λ, 1
    2, 3-λ]) = 0

(4-λ)(3-λ) - 2 = 0

λ² - 7λ + 10 = 0

(λ - 5)(λ - 2) = 0

λ = 5, λ = 2

the eigenvalues of A are λ = 5, λ = 2

the eigenvectors:

for λ = 5

A - 5I = [
    -1, 1
    2, -2
]

v1 = [1, 2]

for λ = 2

A - 2I = [
    2, 1
    2, 1
]

v2 = [1, -2]

can be diagonalised into A = CDC−1

Given that
D=[
    5,0
    0,2
]

C=[
    1,1
    2,−2
]

## Exercise 4

The transformation matrix
A=[
    1,−2
    0, −1
]

can be diagonalised into A = CDC−1, where

C=[
    0, -1/sqrt(2)
    1, 1/sqrt(2)
]

D=[
    -1, 0
    0, 1
]

C^(-1)=[
    1, 1
    sqrt(2), 0
]

Calculate efficiently A^(100)

A^(100) = CD^(100)C^(-1)

D^(100)=[
    (-1)^(100), 0
    0, 1^(100)
]

D^(100)=[
    1, 0
    0, 1
]
