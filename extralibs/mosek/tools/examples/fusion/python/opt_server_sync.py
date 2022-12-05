##
#  Copyright : Copyright (c) MOSEK ApS, Denmark. All rights reserved.
#
#  File :      opt_server_sync.py
#
#  Purpose :   Demonstrates how to use MOSEK OptServer
#              to solve optimization problem synchronously
##
from mosek.fusion import *
import sys

if len(sys.argv) < 1:
    print("Missing argument, syntax is:")
    print("  python opt_server_sync.py serveraddr")
    sys.exit(1)

addr = sys.argv[1]

with Model('testOptServer') as M:
    # Setup a simple test problem
    x = M.variable('x', 3, Domain.greaterThan(0.0))
    M.constraint("lc", Expr.dot([1.0, 1.0, 2.0], x), Domain.equalsTo(1.0))
    M.objective("obj", ObjectiveSense.Minimize, Expr.sum(x))

    # Attach log handler
    M.setLogHandler(sys.stdout)

    # Set OptServer URL
    M.optserverHost(addr)

    # Solve the problem on the OptServer
    M.solve()

    # Get the solution
    print('x1,x2,x3 = %s' % str(x.level()))