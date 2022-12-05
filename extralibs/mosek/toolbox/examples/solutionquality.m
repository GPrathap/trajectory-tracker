%
%  Copyright : Copyright (c) MOSEK ApS, Denmark. All rights reserved.
%
%  File :      solutionquality.m
%
%  Purpose :   To demonstrate how to examine the quality of a solution. 
%
 
function solutionquality(data)

    cmd      = sprintf('read(%s)',data)
    % Read the problem from file
    [r, res] = mosekopt(cmd)

    % Perform the optimization.
    [r, res] = mosekopt('minimize', res.prob); 

    solsta = strcat('MSK_SOL_STA_', res.sol.itr.solsta);
    
    if strcmp(solsta, 'MSK_SOL_STA_OPTIMAL')

      sol = res.sol.itr
      primalobj= sol.pobjval
      dualobj= sol.dobjval
      
      abs_obj_gap     = abs(dualobj - primalobj);
      rel_obj_gap     = abs_obj_gap/(1.0 + min( abs(primalobj), abs(dualobj)));

      
      % Assume the application needs the solution to be within
      % 1e-6 optimality in an absolute sense. Another approach
      % would be looking at the relative objective gap */
      
      fprintf('\n\n');
      fprintf('Customized solution information.\n');
      fprintf('  Absolute objective gap: %e\n',abs_obj_gap);
      fprintf('  Relative objective gap: %e\n',rel_obj_gap);
          
      if ( rel_obj_gap>1e-6 )
          fprintf('Warning: The relative objective gap is LARGE.\n');
          % Perhaps we reject the solution here
      else            
          % We accept solution
          res.sol.itr.xx
      end
          
   
    elseif strcmp(solsta, 'MSK_SOL_STA_DUAL_INFEASIBLE_CER') || ...
           strcmp(solsta, 'MSK_SOL_STA_PRIMAL_INFEASIBLE_CER') 
        fprintf('Primal or dual infeasibility certificate found.\n');
   
    elseif strcmp(solsta,  'MSK_SOL_STA_UNKNOWN')
        fprintf('The status of the solution is unknown.\n');

    else
        fprintf('Other solution status');
    end

end