function mosekdiag()
%
% MOSEK diagnostics script for MATLAB.
%

clear all

%
% Basic system information
%
disp(sprintf('Matlab version   : %s', version()));
disp(sprintf('Architecture     : %s', computer()));


%
% Check if version is not too small
%
s = sscanf(version(),'%d.%d');
if s(1) < 7 || (s(1) == 7 && s(2) < 9)
    error(['MOSEK supports MATLAB 7.9 or newer. Installed version: %d.%d'], ...
          s(1), s(2))
end


%
% See if mosekopt is available.
%
c = which('mosekopt');
if length(c) == 0
    % This should never happen (mosekopt.m and mosekdiag.m
    % are in the same directory).
    error('mosekopt is not included in the MATLAB path.')
else
    disp(sprintf('mosekopt path    : %s', c))
end


%
% See if mosekopt is a MeX file.
%
if strcmp(c(end-1:end),'.m')
    error(sprintf(['mosekopt.%s not found. Mostly likely the ' ...
                   'architecture of MATLAB and MOSEK does not ' ...
                   'match.'], mexext()))    
end


%
% Get version
%
try
    [r, res] = mosekopt('version echo(0) nokeepenv');
catch ME 
    disp('*******************************************************');    
    disp('Error: could not load mosekopt.');
    disp('Very likely the folder with shared libraries is not in the environment');
    disp('variable PATH or you missed some operating-system-specific installation step.');    
    disp('Below is a more detailed debug info to help you identify the problem.');
    disp('See the interface manual:');
    disp('   https://docs.mosek.com/9.2/toolbox/install-interface.html#troubleshooting');
    disp('for typical troubleshooting tips.');
    disp('*******************************************************');
    disp('Detailed debug info:');
    rethrow(ME);
end

disp(sprintf('MOSEK version    : %d.%d.%d', ...
    [res.version.major, res.version.minor, res.version.revision]))


%
% Testing a simple linear optimization problem
%
prob.c = [1 2 0]';
prob.a = sparse([1 2 2 1], [1 1 2 3], [1.0 1.0 1.0 1.0]);
prob.blc = [4.0 1.0]';
prob.buc = [6.0 inf]';
prob.blx = sparse(3,1);
prob.bux = [];
param.MSK_IPAR_LOG = 0;
[r,res] = mosekopt('minimize echo(0) nokeepenv', prob, param); 

if r
    disp(sprintf('Test linear solve: Error %d (%s)',  res.rcode, res.rcodestr));
    disp(sprintf('Error message    : %s', res.rmsg));
    
    disp('*******************************************************');    
    disp('Below is a more detailed debug info to help you identify the problem.');
    disp('See the interface manual:');
    disp('   https://docs.mosek.com/9.2/toolbox/install-interface.html#troubleshooting');
    disp('for typical troubleshooting tips.');

    if strfind(res.rcodestr, 'LICENSE')
        disp('This error is related to licensing.');
        disp('The line with "License path" below shows where MOSEK is expecting the license.');
        disp('Make sure the license is placed exactly there and is readable and up to date.');
        disp('Try restarting MATLAB if you installed a new license file.');
    end

    disp('*******************************************************');
    disp('Detailed debug info:');
    mosekopt('debug(4)')         
    
    error('Could not solve a test problem');
else
    disp(sprintf('Test linear solve: Success'));
end

%
% All is fine if we got to the end
%
disp('mosekopt works OK. You can use MOSEK in MATLAB.')