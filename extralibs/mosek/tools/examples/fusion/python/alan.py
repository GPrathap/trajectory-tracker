##
# Copyright: Copyright (c) MOSEK ApS, Denmark. All rights reserved.
#
#  File:      alan.py
#
#  Purpose: This file contains an implementation of the alan.gms (as
#  found in the GAMS online model collection) using Fusion.
#
#  The model is a simple portfolio choice model. The objective is to
#  invest in a number of assets such that we minimize the risk, while
#  requiring a certain expected return.
#
#  We operate with 4 assets (hardware,software, show-biz and treasure
#  bill). The risk is defined by the covariance matrix
#    Q = [[  4.0, 3.0, -1.0, 0.0 ],
#         [  3.0, 6.0,  1.0, 0.0 ],
#         [ -1.0, 1.0, 10.0, 0.0 ],
#         [  0.0, 0.0,  0.0, 0.0 ]]
#
#
#  We use the form Q = U^T * U, where U is a Cholesky factor of Q.
##

import sys
from mosek.fusion import *

#####################################################
### Problem data:
# Security names
securities = ["hardware", "software", "show-biz", "t-bills"]
# Two examples of mean returns on securities
mean1 = [9.0, 7.0, 11.0, 5.0]
mean2 = [8.0, 9.0, 12.0, 7.0]
# Target mean return
target = 10.0
# Factor of covariance matrix.
U = Matrix.dense([[2.0, 1.5, -0.5, 0.0],
                  [0.0, 1.93649167, 0.90369611, 0.0],
                  [0.0, 0.0, 2.98886824, 0.0],
                  [0.0, 0.0, 0.0, 0.0]])

numsec = len(securities)

###################################################
# Create a model with expected returns as parameter
M = Model('alan')
x = M.variable("x", numsec, Domain.greaterThan(0.0))
t = M.variable("t", 1, Domain.greaterThan(0.0))

M.objective("minvar", ObjectiveSense.Minimize, t)

# sum securities to 1.0
M.constraint("wealth", Expr.sum(x), Domain.equalsTo(1.0))
# define target expected return
mean = M.parameter(numsec)
M.constraint("dmean", Expr.dot(mean, x), Domain.greaterThan(target))

# Bound on risk
M.constraint("t > ||Ux||^2",
             Expr.vstack(0.5,
                         t,
                         Expr.mul(U, x)), Domain.inRotatedQCone())

##################################################
# Define a method that solves a single instance
def solve(meanVal):
    print("Solve using mean={0}".format(meanVal))
    mean.setValue(meanVal)
    M.solve()
    solx = x.level()
    print("Solution: {0}".format(solx))
    return solx

#################################################
# Solve two instances with different means
sol1 = solve(mean1)
sol2 = solve(mean2)

# Finish
M.dispose()