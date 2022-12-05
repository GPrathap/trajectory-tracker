##
# Copyright: Copyright (c) MOSEK ApS, Denmark. All rights reserved.
#
# File:      total_variation.py
#
# Purpose:   Demonstrates how to solve a total 
#            variation problem using the Fusion API.
##
import sys
import mosek
from mosek.fusion import *
import numpy as np

def total_var(n,m):
    M = Model('TV')

    u = M.variable("u", [n+1,m+1], Domain.inRange(0.,1.0) )
    t = M.variable("t", [n,m], Domain.unbounded() )

    # In this example we define sigma and the input image f as parameters
    # to demonstrate how to solve the same model with many data variants.
    # Of course they could simply be passed as ordinary arrays if that is not needed.
    sigma = M.parameter("sigma")
    f = M.parameter("f", [n,m])

    ucore = u.slice( [0,0], [n,m] )

    deltax = Expr.sub( u.slice( [1,0], [n+1,m] ), ucore)
    deltay = Expr.sub( u.slice( [0,1], [n,m+1] ), ucore)

    M.constraint( Expr.stack(2, t, deltax, deltay), Domain.inQCone().axis(2) )

    M.constraint( Expr.vstack(sigma, Expr.flatten( Expr.sub( f, ucore ) ) ),
                  Domain.inQCone() )

    M.objective( ObjectiveSense.Minimize, Expr.sum(t) )

    return M

#Display
def show(n,m,grid):
    try:
        import matplotlib
        import matplotlib.pyplot as plt
        import matplotlib.cm as cm
        plt.imshow(grid, extent=(0,m,0,n),
                   interpolation='nearest', cmap=cm.jet,
                   vmin=0, vmax=1)
        plt.show()
    except:
        print (grid)

if __name__ == '__main__':
    np.random.seed(0)
    
    n, m = 100, 200

    # Create a parametrized model with given shape
    M = total_var(n, m)
    sigma = M.getParameter("sigma")
    f     = M.getParameter("f")
    ucore = M.getVariable("u").slice([0,0], [n,m])

    # Example: Linear signal with Gaussian noise    
    signal = np.reshape([[1.0*(i+j)/(n+m) for i in range(m)] for j in range(n)], (n,m))
    noise  = np.random.normal(0., 0.08, (n,m))
    fVal   = signal + noise

    # Uncomment to get graphics:
    # show(n, m, signal)
    # show(n, m, fVal)
   
    # Set value for f
    f.setValue(fVal)

    for sigmaVal in [0.0004, 0.0005, 0.0006]:
        # Set new value for sigma and solve
        sigma.setValue(sigmaVal*n*m)

        M.solve()

        sol = np.reshape(ucore.level(), (n,m))
        # Now use the solution
        # ...

        # Uncomment to get graphics:
        # show(n, m, np.reshape(ucore.level(), (n,m)))

        print("rel_sigma = {sigmaVal}  total_var = {var}".format(sigmaVal=sigmaVal,
                                                                 var=M.primalObjValue() ))


    M.dispose()