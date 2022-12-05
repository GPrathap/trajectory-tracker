//
// Copyright: Copyright (c) MOSEK ApS, Denmark. All rights reserved.
//
//  File:      opt_server_sync.java
//
//  Purpose :   Demonstrates how to use MOSEK OptServer
//              to solve optimization problem synchronously

package com.mosek.fusion.examples;
import mosek.fusion.*;

public class opt_server_sync {

  public static void main(String[] args) throws SolutionError {
    if (args.length<1) {
        System.out.println("Missing argument, syntax is:");
        System.out.println("   java com.mosek.fusion.examples.opt_server_sync serveraddr");
        return;
    }

    String addr = args[0];

    // Setup a simple test problem
    Model M = new Model("testOptServer");
    Variable x = M.variable("x", 3, Domain.greaterThan(0.0));
    M.constraint("lc", Expr.dot(new double[] {1.0, 1.0, 2.0}, x), Domain.equalsTo(1.0));
    M.objective("obj", ObjectiveSense.Minimize, Expr.sum(x));

    // Attach log handler
    M.setLogHandler(new java.io.PrintWriter(System.out));

    // Set OptServer URL
    M.optserverHost(addr);

    // Solve the problem on the OptServer
    M.solve();

    // Get the solution
    double[] solx = x.level();
    System.out.printf("x1,x2,x3 = %e, %e, %e\n", solx[0], solx[1], solx[2]);
  }
}