#
#  Copyright : Copyright (c) MOSEK ApS, Denmark. All rights reserved.
#
#  File :      sdo2.py
#
#  Purpose :   Solves the semidefinite problem with two symmetric variables:
#
#                 min   <C1,X1> + <C2,X2>
#                 st.   <A1,X1> + <A2,X2> = b
#                             (X2)_{1,2} <= k
#                
#                 where X1, X2 are symmetric positive semidefinite,
#
#                 C1, C2, A1, A2 are assumed to be constant symmetric matrices,
#                 and b, k are constants.

from mosek.fusion import *
import sys
import numpy as np

# Sample data in sparse, symmetric triplet format
C1_k = [0, 2]
C1_l = [0, 2]
C1_v = [1, 6]
A1_k = [0, 2, 0, 2]
A1_l = [0, 0, 2, 2]
A1_v = [1, 1, 1, 2]
C2_k = [0, 1, 0, 1, 2]
C2_l = [0, 0, 1, 1, 2]
C2_v = [1, -3, -3, 2, 1]
A2_k = [1, 0, 1, 3]
A2_l = [0, 1, 1, 3]
A2_v = [1, 1, -1, -3]
b = 23
k = -3

# Convert input data into Fusion sparse matrices
C1 = Matrix.sparse(3, 3, C1_k, C1_l, C1_v)
C2 = Matrix.sparse(4, 4, C2_k, C2_l, C2_v)
A1 = Matrix.sparse(3, 3, A1_k, A1_l, A1_v)
A2 = Matrix.sparse(4, 4, A2_k, A2_l, A2_v)

# Define the model
with Model('sdo2') as M:
    # Two semidefinite variables
    X1 = M.variable(Domain.inPSDCone(3))
    X2 = M.variable(Domain.inPSDCone(4))

    # Objective
    M.objective(ObjectiveSense.Minimize, Expr.add(Expr.dot(C1,X1), Expr.dot(C2,X2)))

    # Equality constraint
    M.constraint(Expr.add([Expr.dot(A1,X1), Expr.dot(A2,X2)]), Domain.equalsTo(b))

    # Inequality constraint
    M.constraint(X2.index([0,1]), Domain.lessThan(k))

    # Solve
    M.setLogHandler(sys.stdout)
    M.solve()

    # Retrieve result
    print("X1:\n{0}".format(np.reshape(X1.level(), (3,3))))
    print("X2:\n{0}".format(np.reshape(X2.level(), (4,4))))