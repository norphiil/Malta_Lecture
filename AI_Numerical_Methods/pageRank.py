# Lecture 4 and Lecture 5
import numpy as np

L = np.array([
    [0,     0,   1/3, 0,   0],
    [1/3,   0,   1/3, 1/2, 0],
    [0,     0,   0,   1/2, 1],
    [1/3,   0,   1/3, 0,   0],
    [1/3,   1,   0,   0,   0],
])

# L = np.array([
#     [0, 0, 1/2, 1/3],
#     [1/2, 0, 1/2, 1/3],
#     [0, 0, 0, 1/3],
#     [1/2, 1, 0, 0],
# ])

# L = np.array([
#     [0, 1/3,    0,      1/2,    1/2],
#     [0, 0,      0,      0,      1/2],
#     [1, 1/3,    0,      1/2,    0],
#     [0, 1/3,    1/2,    0,      0],
#     [0, 0,      1/2,    0,      0],
# ])

n_pages = len(L)
r = np.ones([n_pages, 1])*1 / n_pages
# r = [
#     [1/5],
#     [1/5],
#     [1/5],
#     [1/5],
#     [1/5],
# ]

# print("L: ", L)
# print("r: ", r)
# diff_norm = 999
# i = 0
# while diff_norm > 0.005:
#     i += 1
#     print("iteration: ", i)
#     r_new = np.dot(L, r)
#     diff_norm = np.linalg.norm(np.subtract(r_new, r))
#     r = r_new
#     print("r: ", r)
#     print("diff_norm: ", diff_norm)

# eval, evec = np.linalg.eig(L)
# print("eval: ", eval)
# print("evec: ", evec)

# for i in range(len(eval)):
#     if np.isclose(eval[i], 1):
#         print("i: ", i)
#         print("eval: ", eval[i])
#         eval_0 = eval[i]
#         evec_0 = evec[:, i]
#         real_evec_0 = np.real(evec_0)
#         print("real_evec_0: ", real_evec_0)

#         norm_evec_0 = real_evec_0/np.linalg.norm(real_evec_0, 1)
#         print("norm_evec_0: ", norm_evec_0)


def gen_net(n_pages):
    link_mat = np.random.randint(2, size=(n_pages, n_pages))
    print(link_mat)
    link_mat = link_mat / np.sum(link_mat, axis=0)
    return link_mat


def page_rank(link_matrix, d=0.85):
    n_pages = len(link_matrix)
    r = np.ones([n_pages, 1]) * 1 / n_pages

    diff_norm = 999
    i = 0
    while diff_norm > 0.005 and i < 1000:
        print("iteration: ", i + 1)
        i += 1
        M = (d * link_matrix) + (1 - d) / n_pages * np.ones((n_pages, n_pages))
        r_new = np.dot(M, r)

        diff_norm = np.linalg.norm(np.subtract(r_new, r))
        r = r_new

    return r


def get_eigenvector(link_matrix):
    # Compute Eigenvalues and Eigenvectors
    evals, evecs = np.linalg.eig(link_matrix)

    # Identify the Eigenvalue 1
    index = np.where(np.isclose(evals, 1))[0][0]

    # Extract the Corresponding Eigenvector
    eigenvector = np.real(evecs[:, index])

    # Normalize the Eigenvector
    eigenvector /= np.sum(eigenvector)

    return eigenvector


print(np.round(get_eigenvector(L) * 100, 2))
print(np.round(page_rank(L, d=1) * 100, 2))
print(gen_net(5))

A = np.array([
    [3, 1],
    [0, 3],
])


eigenvalues, eigenvectors = np.linalg.eig(A)
print("eigenvalues: ", eigenvalues)
print("eigenvectors: ", eigenvectors)
