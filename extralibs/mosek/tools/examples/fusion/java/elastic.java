// Copyright: Copyright (c) MOSEK ApS, Denmark. All rights reserved.
//
// File:      elastic.java
//
// Purpose: Demonstrates model parametrization on the example of an elastic net linear regression:
//
//          min_x  |Ax-b|_2 + lambda1*|x|_1 + lambda2*|x|_2

package com.mosek.fusion.examples;
import mosek.fusion.*;

public class elastic {

  // Construct the model with parameters b, lambda1, lambda2
  // and with prescribed matrix A
  public static Model initializeModel(int m, int n, double[][] A) {
    Model M = new Model();
    Variable x = M.variable("x", n);

    // t >= |Ax-b|_2 where b is a parameter
    Parameter b = M.parameter("b", m);
    Variable  t = M.variable();
    M.constraint(Expr.vstack(t, Expr.sub(Expr.mul(A, x), b)), Domain.inQCone());

    // p_i >= |x_i|, i=1..n
    Variable p = M.variable(n);
    M.constraint(Expr.hstack(p, x), Domain.inQCone());

    // q >= |x|_2
    Variable q = M.variable();
    M.constraint(Expr.vstack(q, x), Domain.inQCone());

    // Objective, parametrized with lambda1, lambda2
    // t + lambda1*sum(p) + lambda2*q
    Parameter lambda1 = M.parameter("lambda1");
    Parameter lambda2 = M.parameter("lambda2");
    Expression obj = Expr.add(new Expression[] {t, Expr.mul(lambda1, Expr.sum(p)), Expr.mul(lambda2, q)});
    M.objective(ObjectiveSense.Minimize, obj);

    // Return the ready model
    return M;
  }

  public static void smallExample() throws SolutionError {
    //Create a small example
    int m = 4;
    int n = 2;
    double[][] A = { {1.0,   2.0},
                     {3.0,   4.0},
                     {-2.0, -1.0},
                     {-4.0, -3.0} };
    double[] sol;
    Model M = initializeModel(m, n, A);

    // For convenience retrieve some elements of the model
    Parameter b = M.getParameter("b");
    Parameter lambda1 = M.getParameter("lambda1");
    Parameter lambda2 = M.getParameter("lambda2");
    Variable x = M.getVariable("x");

    try {
      // First solve
      b.setValue(new double[]{0.1, 1.2, -1.1, 3.0});
      lambda1.setValue(0.1);
      lambda2.setValue(0.01);

      M.solve();
      sol = x.level();
      System.out.printf("Objective %.5f, solution %.3f, %.3f\n", M.primalObjValue(), sol[0], sol[1]);

      // Increase lambda1
      lambda1.setValue(0.5);
      
      M.solve();
      sol = x.level();
      System.out.printf("Objective %.5f, solution %.3f, %.3f\n", M.primalObjValue(), sol[0], sol[1]);

      // Now change the data completely
      b.setValue(new double[] {1.0, 1.0, 1.0, 1.0});
      lambda1.setValue(0.0);
      lambda2.setValue(0.0);
      
      M.solve();
      sol = x.level();
      System.out.printf("Objective %.5f, solution %.3f, %.3f\n", M.primalObjValue(), sol[0], sol[1]);

      // And increase lamda2
      lambda2.setValue(1.4145);
      
      M.solve();
      sol = x.level();
      System.out.printf("Objective %.5f, solution %.3f, %.3f\n", M.primalObjValue(), sol[0], sol[1]);
    } 
    catch (SolutionError e) {
      System.out.println("Solution not available");
      throw e;
    }
    finally {
      M.dispose();
    }
  }

  public static void main(String[] argv) throws SolutionError {
    smallExample();
  }
}
