/*
   Copyright : Copyright (c) MOSEK ApS, Denmark. All rights reserved.

   File :      opt_server_sync.java

   Purpose :   Demonstrates how to use MOSEK OptServer
               to solve optimization problem synchronously

*/
package com.mosek.example;
import mosek.*;

public class opt_server_sync {
  public static void main (String[] args) {
    if (args.length < 2) {
      System.out.println ("Missing argument, syntax is:");
      System.out.println ("  opt_server_sync inputfile serveraddr");
    } else {

      String inputfile = args[0];
      String addr      = args[1];

      rescode trm[]  = new rescode[1];

      try (Env  env  = new Env();
           Task task = new Task(env, 0, 0)) {
        task.set_Stream (mosek.streamtype.log,
        new mosek.Stream() {
          public void stream(String msg) { System.out.print(msg); }
        });

        task.readdata (inputfile);

        // Set OptServer URL
        task.putoptserverhost(addr);

        // Optimize
        task.optimize (trm);

        task.solutionsummary (mosek.streamtype.log);
      }
    }
  }
}