##
#  Copyright : Copyright (c) MOSEK ApS, Denmark. All rights reserved.
#
#  File :      opt_server_sync.py
#
#  Purpose :   Demonstrates how to use MOSEK OptServer
#              to solve optimization problem synchronously
##
import mosek
import sys

def streamprinter(msg):
    sys.stdout.write(msg)
    sys.stdout.flush()

if len(sys.argv) <= 2:
    print("Missing argument, syntax is:")
    print("  opt_server_sync inputfile serveraddr")
else:

    inputfile = sys.argv[1]
    addr = sys.argv[2]

    # Create the mosek environment.
    with mosek.Env() as env:

        # Create a task object linked with the environment env.
        # We create it with 0 variables and 0 constraints initially,
        # since we do not know the size of the problem.
        with env.Task(0, 0) as task:
            task.set_Stream(mosek.streamtype.log, streamprinter)

            # We assume that a problem file was given as the first command
            # line argument (received in `argv')
            task.readdata(inputfile)

            # Set OptServer URL
            task.putoptserverhost(addr)
            
            # Solve the problem remotely
            trm = task.optimize()

            # Print a summary of the solution
            task.solutionsummary(mosek.streamtype.log)