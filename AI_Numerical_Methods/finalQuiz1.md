# Final Quiz

\[
    u2 = v2 - \frac{v2 \cdot u1}{u1 \cdot u1} u1
\]

\[
    Var(X) = E[(X - μ)^2] = 1/n \sum_{i=1}^{n} (x_i - μ)^2
\]

\[
    Cov(X, Y) = E[(XY)] - E[X]E[Y] \\
    = 1/n \sum_{i=1}^{n} (x_i - E[X])(y_i - E[Y]) \\
    Q = 1/n \sum_{i=1}^{n} (u_i - Q)(u_i - Q)^T
\]

# Question

Work out the covariance matrix from this data:

\[
\begin{array}{cc}
\text{Feature 1} & \text{Feature 2} \\
1 & 3 \\
4 & 6 \\
\end{array}
\]

\[
\mu_1 = \frac{1 + 4}{2} = 2.5
\]

\[
\mu_2 = \frac{3 + 6}{2} = 4.5
\]

\[
    \begin{array}{cc}
    \text{Feature 1} & \text{Feature 2} \\
    1 - 2.5 & 3 - 4.5 \\
    4 - 2.5 & 6 - 4.5 \\
    \end{array}
    =
    \begin{array}{cc}
    -1.5 & -1.5 \\
    1.5 & 1.5 \\
    \end{array}
\]

\[
    Q = 1/n \sum_{i=1}^{n} (u_i - Q)(u_i - Q)^T
\]

\[
    X = \begin{pmatrix}
    -1.5 & -1.5 \\
    1.5 & 1.5 \\
    \end{pmatrix}
\]

\[
    X^T = \begin{pmatrix}
    -1.5 & 1.5 \\
    -1.5 & 1.5 \\
    \end{pmatrix}
\]

\[
    X^T X = \begin{pmatrix}
    -1.5 & 1.5 \\
    -1.5 & 1.5 \\
    \end{pmatrix}
    \begin{pmatrix}
    -1.5 & -1.5 \\
    1.5 & 1.5 \\
    \end{pmatrix}
    =
    \begin{pmatrix}
    (-1.5 \cdot -1.5 + 1.5 \cdot 1.5) & (-1.5 \cdot -1.5 + 1.5 \cdot 1.5) \\
    (-1.5 \cdot -1.5 + 1.5 \cdot 1.5) & (-1.5 \cdot -1.5 + 1.5 \cdot 1.5) \\
    \end{pmatrix}
    =
    \begin{pmatrix}
    4.5 & 4.5 \\
    4.5 & 4.5 \\
    \end{pmatrix}
\]

Finally, the covariance matrix is:

\[
    Q = \frac{1}{2}
    \begin{pmatrix}
    4.5 & 4.5 \\
    4.5 & 4.5 \\
    \end{pmatrix}
\]

\[
    Q = \begin{pmatrix}
    2.25 & 2.25 \\
    2.25 & 2.25 \\
    \end{pmatrix}
\]
