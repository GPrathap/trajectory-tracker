#
#  Copyright : Copyright (c) MOSEK ApS, Denmark. All rights reserved.
#
#  File :      sdo3.py
#
#  Purpose :   Solves the semidefinite problem:
#
#                 min   tr(X_1) + ... + tr(X_n)
#                 st.   <A_11,X_1> + ... + <A_1n,X_n> >= b_1
#                       ...
#                       <A_k1,X_1> + ... + <A_kn,X_n> >= b_k
#                
#                 where X_i are symmetric positive semidefinite of dimension d,
#
#                 A_ji are constant symmetric matrices and b_i are constant.
#
#              This example is to demonstrate creating and using 
#              many matrix variables of the same dimension.

from mosek.fusion import *
import sys
import numpy as np 

# Sample input data
n = 100
d = 4
k = 3
b = [9,10,11]
A = list(map(lambda x: x+np.transpose(x),
             [np.random.normal(0, 5, size=(d, d)) for _ in range(k*n)]))

# Create a model with n semidefinite variables od dimension d x d
with Model("sdo3") as M:
    X = M.variable(Domain.inPSDCone(d, n))

    # Pick indexes of diagonal entries for the objective
    alldiag = [[j,s,s] for j in range(n) for s in range(d)]
    M.objective(ObjectiveSense.Minimize, Expr.sum( X.pick(alldiag) ))

    # Each constraint is a sum of inner products
    # Each semidefinite variable is a slice of X
    for i in range(k):
        M.constraint(Expr.add([Expr.dot(A[i*n+j], 
                                        X.slice([j,0,0], [j+1,d,d]).reshape([d,d])) 
                               for j in range(n)]), 
                     Domain.greaterThan(b[i]))

    # Solve
    M.setLogHandler(sys.stdout)            # Add logging
    M.writeTask("sdo3.ptf")                # Save problem in readable format
    M.solve()

    # Get results. Each variable is a slice of X
    print("Contributing variables:")
    for j in range(n):
        Xj = X.slice([j,0,0],[j+1,d,d]).level()
        if any(Xj[s]>1e-6 for s in range(d*d)):
            print("X{0}=\n{1}".format(j, np.reshape(Xj,(d,d))))