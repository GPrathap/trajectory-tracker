# Copyright: Copyright (c) MOSEK ApS, Denmark. All rights reserved.
#
# File:      elastic.py
#
# Purpose: Demonstrates model parametrization on the example of an elastic net linear regression:
#
#          min_x  |Ax-b|_2 + lambda1*|x|_1 + lambda2*|x|_2

from mosek.fusion import *

# Construct the model with parameters b, lambda1, lambda2
# and with prescribed matrix A
def initializeModel(m, n, A):
    M = Model()
    x = M.variable("x", n)

    # t >= |Ax-b|_2 where b is a parameter
    b = M.parameter("b", m)
    t = M.variable()
    M.constraint(Expr.vstack(t, Expr.sub(Expr.mul(A, x), b)), Domain.inQCone())

    # p_i >= |x_i|, i=1..n
    p = M.variable(n)
    M.constraint(Expr.hstack(p, x), Domain.inQCone())

    # q >= |x|_2
    q = M.variable()
    M.constraint(Expr.vstack(q, x), Domain.inQCone())

    # Objective, parametrized with lambda1, lambda2
    # t + lambda1*sum(p) + lambda2*q
    lambda1 = M.parameter("lambda1")
    lambda2 = M.parameter("lambda2")
    obj = Expr.add([t, Expr.mul(lambda1, Expr.sum(p)), Expr.mul(lambda2, q)])
    M.objective(ObjectiveSense.Minimize, obj)

    # Return the ready model
    return M

def smallExample():
    # Create a small example
    m, n = 4, 2
    A = [ [1.0, 2.0],
          [3.0, 4.0],
          [-2.0, -1.0],
          [-4.0, -3.0] ]
    M = initializeModel(m, n, A)

    # For convenience retrieve some elements of the model
    b = M.getParameter("b")
    lambda1 = M.getParameter("lambda1")
    lambda2 = M.getParameter("lambda2")
    x = M.getVariable("x")

    # First solve
    b.setValue([0.1, 1.2, -1.1, 3.0])
    lambda1.setValue(0.1)
    lambda2.setValue(0.01)

    M.solve()
    print("Objective {0}, solution x = {1}".format(M.primalObjValue(), x.level()))
    
    # Increase lambda1
    lambda1.setValue(0.5)
    
    M.solve()
    print("Objective {0}, solution x = {1}".format(M.primalObjValue(), x.level()))

    # Now change the data completely
    b.setValue([1.0, 1.0, 1.0, 1.0])
    lambda1.setValue(0.0)
    lambda2.setValue(0.0)
    
    M.solve()
    print("Objective {0}, solution x = {1}".format(M.primalObjValue(), x.level()))

    # And increase lamda2
    lambda2.setValue(1.4145)
    
    M.solve()
    print("Objective {0}, solution x = {1}".format(M.primalObjValue(), x.level()))

    M.dispose()

smallExample()