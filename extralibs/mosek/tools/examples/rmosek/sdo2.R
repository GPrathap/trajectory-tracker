#
#  Copyright : Copyright (c) MOSEK ApS, Denmark. All rights reserved.
#
#  File :      sdo2.R
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
library("Rmosek")

getbarvarMatrix <- function(barvar, bardim)
{
    N <- as.integer(bardim)
    new("dspMatrix", x=barvar, uplo="L", Dim=c(N,N))
}

sdo2 <- function()
{
    # Sample data in sparse lower-triangular triplet form
    C1_k <- c(1, 3);
    C1_l <- c(1, 3);
    C1_v <- c(1, 6);
    A1_k <- c(1, 3, 3);
    A1_l <- c(1, 1, 3);
    A1_v <- c(1, 1, 2);
    C2_k <- c(1, 2, 2, 3);
    C2_l <- c(1, 1, 2, 3);
    C2_v <- c(1, -3, 2, 1);
    A2_k <- c(2, 2, 4);
    A2_l <- c(1, 2, 4);
    A2_v <- c(1, -1, -3);
    b <- 23.0;
    k <- -3.0;
 
    # Specify the dimensions of the problem
    prob <- list(sense="min");
    # Two constraints
    prob$A <- Matrix(nrow=2, ncol=0);                   # 2 constraints
    prob$c <- numeric(0);
    prob$bx <- rbind(blc=numeric(0), 
                     buc=numeric(0));

    # Dimensions of semidefinite matrix variables 
    prob$bardim <- c(3, 4);
    # Constraint bounds    
    prob$bc <- rbind(blc=c(b, -Inf), 
                     buc=c(b, k));

    # Block triplet format specifying the lower triangular part 
    # of the symmetric coefficient matrix 'barc':
    prob$barc$j <- c(rep(1, length(C1_v)), 
                     rep(2, length(C2_v)));         # Which PSD variable (j)
    prob$barc$k <- c(C1_k, C2_k);                   # Entries: (k,l)->v
    prob$barc$l <- c(C1_l, C2_l);
    prob$barc$v <- c(C1_v, C2_v);

    # Block triplet format specifying the lower triangular part 
    # of the symmetric coefficient matrix 'barA':
    prob$barA$i <- c(rep(1, length(A1_v)+length(A2_v)), 2);   # Which constraint (i)
    prob$barA$j <- c(rep(1, length(A1_v)),
                     rep(2, length(A2_v)),
                     2);                                      # Which PSD variable (j)
    prob$barA$k <- c(A1_k, A2_k, 2);                          # Entries: (k,l)->v
    prob$barA$l <- c(A1_l, A2_l, 1);
    prob$barA$v <- c(A1_v, A2_v, 0.5);

    # Solve the problem
    r <- mosek(prob);

    # Print matrix variable and return the solution
    stopifnot(identical(r$response$code, 0));
    print( list(X1=getbarvarMatrix(r$sol$itr$barx[[1]], prob$bardim[1])) );
    print( list(X2=getbarvarMatrix(r$sol$itr$barx[[2]], prob$bardim[2])) );

}

sdo2();