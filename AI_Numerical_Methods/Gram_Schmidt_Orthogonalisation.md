# Exercises

## Gram Schmidt Orthogonalisation

\[
V_1 = \begin{pmatrix} 1 \\ -1 \\ 1 \end{pmatrix}
\]
\[
V_2 = \begin{pmatrix} 1 \\ 0 \\ 1 \end{pmatrix}
\]
\[
V_3 = \begin{pmatrix} 1 \\ 1 \\ 2 \end{pmatrix}
\]

\[
   U_1 = V_1 = \begin{pmatrix} 1 \\ -1 \\ 1 \end{pmatrix}
\]

\[
   U_2 = V_2 - \frac{V_2 \cdot U_1}{U_1 \cdot U_1} U_1
\]
\[
     V_2 \cdot U_1 = \begin{pmatrix} 1 \\ 0 \\ 1 \end{pmatrix} \cdot \begin{pmatrix} 1 \\ -1 \\ 1 \end{pmatrix} = 1 \cdot 1 + 0 \cdot (-1) + 1 \cdot 1 = 2
     \]
\[
     U_1 \cdot U_1 = \begin{pmatrix} 1 \\ -1 \\ 1 \end{pmatrix} \cdot \begin{pmatrix} 1 \\ -1 \\ 1 \end{pmatrix} = 1^2 + (-1)^2 + 1^2 = 3
\]
\[
    U_2 = \begin{pmatrix} 1 \\ 0 \\ 1 \end{pmatrix} - \frac{2}{3} \begin{pmatrix} 1 \\ -1 \\ 1 \end{pmatrix} = \begin{pmatrix} 1 \\ 0 \\ 1 \end{pmatrix} - \begin{pmatrix} 2/3 \\ -2/3 \\ 2/3 \end{pmatrix} = \begin{pmatrix} 1/3 \\ 2/3 \\ 1/3 \end{pmatrix}
\]

\[
   U_3 = V_3 - \frac{V_3 \cdot U_1}{U_1 \cdot U_1} U_1 - \frac{V_3 \cdot U_2}{U_2 \cdot U_2} U_2
\]
\[
     V_3 \cdot U_1 = \begin{pmatrix} 1 \\ 1 \\ 2 \end{pmatrix} \cdot \begin{pmatrix} 1 \\ -1 \\ 1 \end{pmatrix} = 1 \cdot 1 + 1 \cdot (-1) + 2 \cdot 1 = 2
\]
\[
     V_3 \cdot U_2 = \begin{pmatrix} 1 \\ 1 \\ 2 \end{pmatrix} \cdot \begin{pmatrix} \frac{1}{3} \\ \frac{2}{3} \\ \frac{1}{3} \end{pmatrix} = 1 \cdot \frac{1}{3} + 1 \cdot \frac{2}{3} + 2 \cdot \frac{1}{3} = \frac{5}{3}
\]
\[
     U_2 \cdot U_2 = \begin{pmatrix} \frac{1}{3} \\ \frac{2}{3} \\ \frac{1}{3} \end{pmatrix} \cdot \begin{pmatrix} \frac{1}{3} \\ \frac{2}{3} \\ \frac{1}{3} \end{pmatrix} = \left(\frac{1}{3}\right)^2 + \left(\frac{2}{3}\right)^2 + \left(\frac{1}{3}\right)^2 = \frac{6}{9} = \frac{2}{3}
\]
\[
\frac{V_3 \cdot U_1}{U_1 \cdot U_1} U_1 = \frac{2}{3} \begin{pmatrix} 1 \\ -1 \\ 1 \end{pmatrix} = \begin{pmatrix} \frac{2}{3} \\ -\frac{2}{3} \\ \frac{2}{3} \end{pmatrix}
\]
\[
\frac{V_3 \cdot U_2}{U_2 \cdot U_2} U_2 = \frac{\frac{5}{3}}{\frac{2}{3}} \begin{pmatrix} \frac{1}{3} \\ \frac{2}{3} \\ \frac{1}{3} \end{pmatrix} = \begin{pmatrix} \frac{5}{6} \\ \frac{5}{3} \\ \frac{5}{6} \end{pmatrix}
\]
\[
   U_3 = \begin{pmatrix} 1 \\ 1 \\ 2 \end{pmatrix} - \begin{pmatrix} \frac{2}{3} \\ -\frac{2}{3} \\ \frac{2}{3} \end{pmatrix} - \begin{pmatrix} \frac{5}{9} \\ \frac{10}{9} \\ \frac{5}{9} \end{pmatrix} = \begin{pmatrix} -\frac{1}{2} \\ 0 \\ \frac{1}{2} \end{pmatrix}
\]

## LU Decomposition

\[ A = \begin{pmatrix} x & y \\ z & w \end{pmatrix} = \begin{pmatrix} 1 & 0 \\ a & 1 \end{pmatrix} \begin{pmatrix} b & c \\ 0 & d \end{pmatrix} = \begin{pmatrix} b & c \\ ab & ac + d \end{pmatrix} \]

1. factor these matrices into A = LU

a.
\[ A =
\begin{pmatrix} 2 & 1 \\ 6 & 7 \end{pmatrix}
\]
\[ LU = \begin{pmatrix} 1 & 0 \\ 3 & 1 \end{pmatrix} \begin{pmatrix} 2 & 1 \\ 0 & 4 \end{pmatrix} = \begin{pmatrix} 2 & 1 \\ 6 + 0 & 3 + 4 \end{pmatrix} = \begin{pmatrix} 2 & 1 \\ 6 & 7 \end{pmatrix} \]

\( A = \begin{pmatrix} 2 & 1 \\ 6 & 7 \end{pmatrix} \)
\( L = \begin{pmatrix} 1 & 0 \\ 3 & 1 \end{pmatrix} \)
\( U = \begin{pmatrix} 2 & 1 \\ 0 & 4 \end{pmatrix} \)

b.
\[ A =
\begin{pmatrix} 3 & 1 \\ -6 & -4 \end{pmatrix}
\]
\[ LU = \begin{pmatrix} 1 & 0 \\ -2 & 1 \end{pmatrix} \begin{pmatrix} 3 & 1 \\ 0 & -2 \end{pmatrix} = \begin{pmatrix} 3 & 1 \\ -6 + 0 & -2 + -2 \end{pmatrix} = \begin{pmatrix} 3 & 1 \\ -6 & -4 \end{pmatrix} \]
\( A = \begin{pmatrix} 3 & 1 \\ -6 & -4 \end{pmatrix} \)
\( L = \begin{pmatrix} 1 & 0 \\ -2 & 1 \end{pmatrix} \)
\( U = \begin{pmatrix} 3 & 1 \\ 0 & -2 \end{pmatrix} \)

c. A = \[
\begin{pmatrix} 1 & 1 & 1 \\ 1 & 1 & 1 \\ 1 & 1 & 1 \end{pmatrix}
\]

\[ LU = \begin{pmatrix} 1 & 0 & 0 \\ 1 & 1 & 0 \\ 1 & 0 & 1 \end{pmatrix} \begin{pmatrix} 1 & 1 & 1 \\ 0 & 0 & 0 \\ 0 & 0 & 0 \end{pmatrix} = \begin{pmatrix} 1 & 1 & 1 \\ 1 & 1 & 1 \\ 1 & 1 & 1 \end{pmatrix} \]
\( A = \begin{pmatrix} 1 & 1 & 1 \\ 1 & 1 & 1 \\ 1 & 1 & 1 \end{pmatrix} \)
\( L = \begin{pmatrix} 1 & 0 & 0 \\ 1 & 1 & 0 \\ 1 & 0 & 1 \end{pmatrix} \)
\( U = \begin{pmatrix} 1 & 1 & 1 \\ 0 & 0 & 0 \\ 0 & 0 & 0 \end{pmatrix} \)

d. A = \[
\begin{pmatrix} 2 & -1 & 0 \\ -1 & 2 & -1 \\ 0 & -1 & 2 \end{pmatrix}
\]
\[ LU = \begin{pmatrix} 1 & 0 & 0 \\ -1/2 & 1 & 0 \\ 0 & -2/3 & 1 \end{pmatrix} \begin{pmatrix} 2 & -1 & 0 \\ 0 & 3/2 & -1 \\ 0 & 0 & 4/3 \end{pmatrix} = \begin{pmatrix} 2 & -1 & 0 \\ -1 & 2 & -1 \\ 0 & -1 & 2 \end{pmatrix} \]
\( A = \begin{pmatrix} 2 & -1 & 0 \\ -1 & 2 & -1 \\ 0 & -1 & 2 \end{pmatrix} \)
\( L = \begin{pmatrix} 1 & 0 & 0 \\ -1/2 & 1 & 0 \\ 0 & -2/3 & 1 \end{pmatrix} \)
\( U = \begin{pmatrix} 2 & -1 & 0 \\ 0 & 3/2 & -1 \\ 0 & 0 & 4/3 \end{pmatrix} \)

## 𝑄𝑅 Decomposition and Gram-Schmidt Orthogonalisation

1. Consider the matrix 𝐴:

\[
A = \begin{pmatrix} 2 & -2 & 18 \\ 2 & 1 & 0 \\ 1 & 2 & 0 \end{pmatrix}
\]

a. Orthogonalise the matrix 𝐴 using the Gram-Schmidt process.

\[
\mathbf{v}_1 = \begin{pmatrix} 2 \\ 2 \\ 1 \end{pmatrix}, \quad \mathbf{v}_2 = \begin{pmatrix} -2 \\ 1 \\ 2 \end{pmatrix}, \quad \mathbf{v}_3 = \begin{pmatrix} 18 \\ 0 \\ 0 \end{pmatrix}
\]
\[
\mathbf{u}_1 = \begin{pmatrix} 2 \\ 2 \\ 1 \end{pmatrix}
\]
\[
\text{proj}_{\mathbf{u}_1} \mathbf{v}_2 = \frac{\mathbf{v}_2 \cdot \mathbf{u}_1}{\mathbf{u}_1 \cdot \mathbf{u}_1} \mathbf{u}_1
\]
\[
\mathbf{u}_2 = \mathbf{v}_2 - \text{proj}_{\mathbf{u}_1} \mathbf{v}_2
\]
\[
\mathbf{u}_3 = \mathbf{v}_3 - \text{proj}_{\mathbf{u}_1} \mathbf{v}_3 - \text{proj}_{\mathbf{u}_2} \mathbf{v}_3
\]
\[
\mathbf{u}_1 = \begin{pmatrix} 2 \\ 2 \\ 1 \end{pmatrix}, \quad \mathbf{u}_2 = \begin{pmatrix} -2 \\ 1 \\ 2 \end{pmatrix}, \quad \mathbf{u}_3 = \begin{pmatrix} 2 \\ -4 \\ 4 \end{pmatrix}
\]

b. Using your answer to (a), find the QR decomposition of the matrix 𝐴.

