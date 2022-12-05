##
# Copyright: Copyright (c) MOSEK ApS, Denmark. All rights reserved.
#
# File:      portfolio_2_frontier.py
#
#  Purpose :   Implements a basic portfolio optimization model.
#              Computes points on the efficient frontier.
#
##

from mosek.fusion import *
import numpy as np
import sys

"""
Purpose:
    Computes several portfolios on the optimal portfolios by

        for alpha in alphas:
            maximize   expected return - alpha * variance
            subject to the constraints

Input:
    n: Number of assets
    mu: An n dimensional vector of expected returns
    GT: A matrix with n columns so (GT')*GT  = covariance matrix
    x0: Initial holdings
    w: Initial cash holding
    alphas: List of the alphas

Output:
    The efficient frontier as list of tuples (alpha, expected return, variance)
"""
def EfficientFrontier(n,mu,GT,x0,w,alphas):

    with Model("Efficient frontier") as M:
        frontier = []

        # Defines the variables (holdings). Shortselling is not allowed.
        x = M.variable("x", n, Domain.greaterThan(0.0)) # Portfolio variables
        s = M.variable("s", 1, Domain.unbounded())      # Variance variable

        # Total budget constraint
        M.constraint('budget', Expr.sum(x), Domain.equalsTo(w+sum(x0)))

        # Computes the risk
        M.constraint('variance', Expr.vstack(s, 0.5, Expr.mul(GT,x)), Domain.inRotatedQCone())

        # Define objective as a weighted combination of return and variance
        alpha = M.parameter()
        M.objective('obj', ObjectiveSense.Maximize, Expr.sub(Expr.dot(mu,x), Expr.mul(alpha,s)))
        
        # Solve multiple instances by varying the parameter alpha
        for a in alphas:
            alpha.setValue(a);

            M.solve()

            frontier.append((a, np.dot(mu,x.level()), s.level()[0]))

        return frontier


if __name__ == '__main__':

    n      = 3;
    w      = 1.0;
    mu     = [0.1073,0.0737,0.0627]
    x0     = [0.0,0.0,0.0]
    GT     = [
        [ 0.166673333200005, 0.0232190712557243 ,  0.0012599496030238 ],
        [ 0.0              , 0.102863378954911  , -0.00222873156550421],
        [ 0.0              , 0.0                ,  0.0338148677744977 ]
    ]

    # Some predefined alphas are chosen
    alphas = [0.0, 0.01, 0.1, 0.25, 0.30, 0.35, 0.4, 0.45, 0.5, 0.75, 1.0, 1.5, 2.0, 3.0, 10.0]
    frontier= EfficientFrontier(n,mu,GT,x0,w,alphas)
    print("\n-----------------------------------------------------------------------------------");
    print('Efficient frontier')
    print("-----------------------------------------------------------------------------------\n");
    print('%-12s  %-12s  %-12s' % ('alpha','return','risk'))
    for i in frontier:
        print('%-12.4f  %-12.4e  %-12.4e' % (i[0],i[1],i[2]))