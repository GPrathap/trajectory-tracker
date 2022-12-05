/*
   Copyright : Copyright (c) MOSEK ApS, Denmark. All rights reserved.
 
   File :      sdo2.java
 
   Purpose :   Solves the semidefinite problem with two symmetric variables:
 
                  min   <C1,X1> + <C2,X2>
                  st.   <A1,X1> + <A2,X2> = b
                              (X2)_{1,2} <= k
                 
                  where X1, X2 are symmetric positive semidefinite,
 
                  C1, C2, A1, A2 are assumed to be constant symmetric matrices,
                  and b, k are constants.
*/
package com.mosek.fusion.examples;
import mosek.fusion.*;

public class sdo2 {
  public static void main(String[] args) throws SolutionError {

    // Sample data in sparse, symmetric triplet format
    int[]    C1_k = {0, 2};
    int[]    C1_l = {0, 2};
    double[] C1_v = {1, 6};
    int[]    A1_k = {0, 2, 0, 2};
    int[]    A1_l = {0, 0, 2, 2};
    double[] A1_v = {1, 1, 1, 2};
    int[]    C2_k = {0, 1, 0, 1, 2};
    int[]    C2_l = {0, 0, 1, 1, 2};
    double[] C2_v = {1, -3, -3, 2, 1};
    int[]    A2_k = {1, 0, 1, 3};
    int[]    A2_l = {0, 1, 1, 3};
    double[] A2_v = {1, 1, -1, -3};
    double b = 23;
    double k = -3;

    // Convert input data into Fusion sparse matrices
    Matrix C1 = Matrix.sparse(3, 3, C1_k, C1_l, C1_v);
    Matrix C2 = Matrix.sparse(4, 4, C2_k, C2_l, C2_v);
    Matrix A1 = Matrix.sparse(3, 3, A1_k, A1_l, A1_v);
    Matrix A2 = Matrix.sparse(4, 4, A2_k, A2_l, A2_v);

    Model M  = new Model("sdo2");
    try {
      // Two semidefinite variables
      Variable X1 = M.variable(Domain.inPSDCone(3));
      Variable X2 = M.variable(Domain.inPSDCone(4));

      // Objective
      M.objective(ObjectiveSense.Minimize, Expr.add(Expr.dot(C1,X1), Expr.dot(C2,X2)));

      // Equality constraint
      M.constraint(Expr.add(Expr.dot(A1,X1), Expr.dot(A2,X2)), Domain.equalsTo(b));

      // Inequality constraint
      M.constraint(X2.index(new int[] {0,1}), Domain.lessThan(k));

      // Solve
      M.setLogHandler(new java.io.PrintWriter(System.out));
      M.solve();

      // Print solution
      System.out.println("Solution (vectorized):");
      System.out.println(java.util.Arrays.toString( X1.level() ));
      System.out.println(java.util.Arrays.toString( X2.level() ));
    } finally {
      M.dispose();
    }
  }
}