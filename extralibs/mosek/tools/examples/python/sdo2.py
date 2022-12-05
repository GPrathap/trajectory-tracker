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

from mosek import *
import sys

# Since the value of infinity is ignored, we define it solely
# for symbolic purposes
inf = 0.0

# Define a stream printer to grab output from MOSEK
def streamprinter(text):
    sys.stdout.write(text)
    sys.stdout.flush()
                
# Sample data in sparse lower-triangular triplet form
C1_k = [0, 2]
C1_l = [0, 2]
C1_v = [1, 6]
A1_k = [0, 2, 2]
A1_l = [0, 0, 2]
A1_v = [1, 1, 2]
C2_k = [0, 1, 1, 2]
C2_l = [0, 0, 1, 2]
C2_v = [1, -3, 2, 1]
A2_k = [1, 1, 3]
A2_l = [0, 1, 3]
A2_v = [1, -1, -3]
b = 23
k = -3

# Make mosek environment
with Env() as env:

    # Create a task object and attach log stream printer
    with env.Task(0, 0) as task:
        # Set log handler for debugging ootput
        task.set_Stream(streamtype.log, streamprinter)

        # Append two symmetric variables of dimension 3, 4
        barvardims = [3, 4]
        task.appendbarvars(barvardims)

        # Semidefinite part of objective function
        task.putbarcblocktriplet(
            len(C1_v)+len(C2_v),           # Number of entries
            [0]*len(C1_v) + [1]*len(C2_v), # Which SDP variable (j)
            C1_k + C2_k,                   # Entries: (k,l)->v
            C1_l + C2_l,
            C1_v + C2_v,
            )

        # Append two constraints
        task.appendcons(2)

        # First constraint (equality)
        task.putbarablocktriplet(
            len(A1_v)+len(A2_v),           # Number of entries
            [0]*(len(A1_v)+len(A2_v)),     # Which constraint (i = 0)
            [0]*len(A1_v) + [1]*len(A2_v), # Which SDP variable (j)
            A1_k + A2_k,                   # Entries: (k,l)->v
            A1_l + A2_l,
            A1_v + A2_v,
            )

        # Second constraint (X2)_{1,2} <= k
        task.putbarablocktriplet(
            1,                             # Number of entries
            [1],                           # Which constraint (i = 1)
            [1],                           # Which SDP variable (j = 1)
            [1], [0], [0.5]                # Entries: (k,l)->v
            )

        # Set bounds for constraints
        task.putconboundlist([0,1], [boundkey.fx, boundkey.up],
                                    [b, -inf],
                                    [b, k])

        # Write the problem for human inspection
        task.writedata("test.ptf")

        # Optimize
        task.optimize()
        task.solutionsummary(streamtype.msg)

        # Get status information about the solution
        solsta = task.getsolsta(soltype.itr)

        if solsta == solsta.optimal:
            # Assuming the optimization succeeded read solution
            print("Solution (lower-triangular part vectorized): ")
            for i in range(2):
                dim = int(barvardims[i]*(barvardims[i]+1)/2)
                X = [0.0] * dim
                task.getbarxj(soltype.itr, i, X)
                print("X{i} = {X}".format(i=i, X=X))

        elif (solsta == solsta.dual_infeas_cer or
              solsta == solsta.prim_infeas_cer):
            print("Primal or dual infeasibility certificate found.\n")
        elif solsta == solsta.unknown:
            print("Unknown solution status")
        else:
            print("Other solution status")