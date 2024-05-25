# Quiz 3

Given the following set of vectors:
\[
    v1 = \begin{pmatrix} 1 \\ 3 \end{pmatrix}
\]


\[
    v2 = \begin{pmatrix} 2 \\ 1 \end{pmatrix}
\]
Apply the Gram-Schmidt orthogonalisation process to these vectors to obtain a set of orthonormal vectors u1, u2.

\[
    u1 = v1
    u1 = \begin{pmatrix} 1 \\ 3 \end{pmatrix}
\]

\[
    u2 = v2 - \frac{v2 \cdot u1}{u1 \cdot u1} u1
\]

\[
    v2 \cdot u1 = 2 \times 1 + 1 \times 3 = 5
\]
\[
    u1 \cdot u1 = 1 \times 1 + 3 \times 3 = 10
\]
\[
    u2 = \begin{pmatrix} 2 \\ 1 \end{pmatrix} - \frac{5}{10} \begin{pmatrix} 1 \\ 3 \end{pmatrix} = \begin{pmatrix} 2 \\ 1 \end{pmatrix} - \begin{pmatrix} 1/2 \\ 3/2 \end{pmatrix} = \begin{pmatrix} 3/2 \\ -1/2 \end{pmatrix}
\]

The matrix A is given by:
\[
    A = \begin{pmatrix} 16 & -8 \\ -8 & 5 \end{pmatrix}
\]
is a positive definite matrix.

True or False?

\[
   \text{det}(A) = 16 \times 5 - (-8) \times (-8) = 80 - 64 = 16
\]

\( 16 > 0 \) so the matrix is positive definite. True.


The matrix C is given by:
\[
    C = \begin{pmatrix} 16 & -8 \\ -8 & 5 \end{pmatrix}
\]
can be decomposed into \[C=LL^T\] where L is a lower triangular matrix.
Use Cholesky decomposition to work out the matrix L

\( L = \begin{pmatrix} l_{11} & 0 \\ l_{21} & l_{22} \end{pmatrix} \)


\(
l_{11} = \sqrt{a_{11}} = \sqrt{16} = 4
\)
\(
l_{21} = \frac{a_{12}}{l_{11}}
\)
\( a_{12} = -8 \)
\( l_{11} = 4 \)
\(
l_{21} = \frac{-8}{4} = -2
\)
\(
l_{22} = \sqrt{a_{22} - l_{21}^2}
\)
\( a_{22} = 5 \)
\( l_{21} = -2 \)
\(
l_{22} = \sqrt{5 - (-2)^2} = \sqrt{5 - 4} = \sqrt{1} = 1
\)


\[
    L = \begin{pmatrix} a & b \\ c & d \end{pmatrix}
\]
where \( a = 4 \), \( b = 0 \), \( c = -2 \), \( d = 1\) so
\[
L = \begin{pmatrix} 4 & 0 \\ -2 & 1 \end{pmatrix}
\]

Given the matrix;
\[
    A = \begin{pmatrix} 1 & 1 & 1 \\ 4 & 3 & -1 \\ 3 & 5 & 3 \end{pmatrix}
\]
the LU decomposition of A can be written as the product of a lower unit triangular matrix L and an upper triangular matrix U.

Fill in the blanks for L and U.
\[
    L = \begin{pmatrix} l_{11} & l_{12} & l_{13} \\ l_{21} & l_{22} & l_{23} \\ l_{31} & l_{32} & l_{33} \end{pmatrix}
\]
\[
    U = \begin{pmatrix} u_{11} & u_{12} & u_{13} \\ u_{21} & u_{22} & u_{23} \\ u_{31} & u_{32} & u_{33} \end{pmatrix}
\]
\[
L = \begin{pmatrix} 1 & 0 & 0 \\ 4 & 1 & 0 \\ 3 & -2 & 1 \end{pmatrix}
\]
\[
U = \begin{pmatrix} 1 & 1 & 1 \\ 0 & -1 & -5 \\ 0 & 0 & 10 \end{pmatrix}
\]

Given that a matrix A can be decomposed into LU, Where

\[
    L = \begin{pmatrix} 1 & 0 & 0 \\ 2 & 1 & 0 \\ 4 & 3 & 1 \end{pmatrix}
\]
\[
    U = \begin{pmatrix} 1 & 2 & 4 \\ 0 & 1 & 5 \\ 0 & 0 & 1 \end{pmatrix}
\]
Solve the system of equations represented by \(Ax=b\) where

\[
    b = \begin{pmatrix} 20 \\ 61 \\ 104 \end{pmatrix}
\]

The solution vector:

\[
    x = \begin{pmatrix} x1 \\ x2 \\ x3 \end{pmatrix}
\]

\[
L = \begin{pmatrix} 1 & 0 & 0 \\ 2 & 1 & 0 \\ 4 & 3 & 1 \end{pmatrix}, \quad b = \begin{pmatrix} 20 \\ 61 \\ 104 \end{pmatrix}
\]
\( y_1 = 20 \)
\( 2y_1 + y_2 = 61 \)
\( 4y_1 + 3y_2 + y_3 = 104 \)

\[
y_1 = 20
\]
\[
2 \cdot 20 + y_2 = 61 \implies y_2 = 61 - 40 = 21
\]
\[
4 \cdot 20 + 3 \cdot 21 + y_3 = 104 \implies 80 + 63 + y_3 = 104 \implies y_3 = 104 - 143 = -39
\]
\( y = \begin{pmatrix} 20 \\ 21 \\ -39 \end{pmatrix} \)

\[
U = \begin{pmatrix} 1 & 2 & 4 \\ 0 & 1 & 5 \\ 0 & 0 & 1 \end{pmatrix}, \quad y = \begin{pmatrix} 20 \\ 21 \\ -39 \end{pmatrix}
\]
\( x_1 + 2x_2 + 4x_3 = 20 \)
\( x_2 + 5x_3 = 21 \)
\( x_3 = -39 \)
\[
x_2 + 5(-39) = 21 \implies x_2 - 195 = 21 \implies x_2 = 216
\]
\[
x_1 + 2(216) + 4(-39) = 20 \implies x_1 + 432 - 156 = 20 \implies x_1 = 20 - 276 = -256
\]

\[ x = \begin{pmatrix} -256 \\ 216 \\ -39 \end{pmatrix} \]