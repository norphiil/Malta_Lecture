# Final Quiz

\[
    proj_v x = \frac{x \cdot v}{v \cdot v} v
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

Compute the projection of the vector v=⟨3,1⟩ onto w=⟨1,2⟩.

\[
    v = \begin{pmatrix} 3 \\ 1 \end{pmatrix}
\]


\[
    w = \begin{pmatrix} 1 \\ 2 \end{pmatrix}
\]

\[
\text{proj}_{w} \mathbf{v} = \frac{\mathbf{v} \cdot \mathbf{w}}{\mathbf{w} \cdot \mathbf{w}} \mathbf{w}
\]

\[
v \cdot w = 3 \cdot 1 + 1 \cdot 2 = 3 + 2 = 5
\]

\[
w \cdot w = 1 \cdot 1 + 2 \cdot 2 = 1 + 4 = 5
\]

\[
    \text{proj}_{\mathbf{w}} \mathbf{v} = 5/5 \begin{pmatrix} 1 \\ 2 \end{pmatrix} = \begin{pmatrix} 1 \\ 2 \end{pmatrix}
\]
