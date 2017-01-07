function [sys,x0,str,ts,simStateCompliance] = mav_dynamics(t,x,u,flag,mass,Jx,Jy,Jz,Jxz,pn0,pe0,pd0,u0,v0,w0,phi0,theta0,psi0,p0,q0,r0)

switch flag,

  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0,
    [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(pn0,pe0,pd0,u0,v0,w0,phi0,theta0,psi0,p0,q0,r0);

  %%%%%%%%%%%%%%%
  % Derivatives %
  %%%%%%%%%%%%%%%
  case 1,
    sys=mdlDerivatives(t,x,u,mass,Jx,Jy,Jz,Jxz);

  %%%%%%%%%%
  % Update %
  %%%%%%%%%%
  case 2,
    sys=mdlUpdate(t,x,u);

  %%%%%%%%%%%
  % Outputs %
  %%%%%%%%%%%
  case 3,
    sys=mdlOutputs(t,x,u);

  %%%%%%%%%%%%%%%%%%%%%%%
  % GetTimeOfNextVarHit %
  %%%%%%%%%%%%%%%%%%%%%%%
  case 4,
    sys=mdlGetTimeOfNextVarHit(t,x,u);

  %%%%%%%%%%%%%
  % Terminate %
  %%%%%%%%%%%%%
  case 9,
    sys=mdlTerminate(t,x,u);

  %%%%%%%%%%%%%%%%%%%%
  % Unexpected flags %
  %%%%%%%%%%%%%%%%%%%%
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end

% end sfuntmpl

%
%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
%
function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(pn0,pe0,pd0,u0,v0,w0,phi0,theta0,psi0,p0,q0,r0)

%
% call simsizes for a sizes structure, fill it in and convert it to a
% sizes array.
%
% Note that in this example, the values are hard coded.  This is not a
% recommended practice as the characteristics of the block are typically
% defined by the S-function parameters.
%
sizes = simsizes;

sizes.NumContStates  = 12;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 12;
sizes.NumInputs      = 6;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);

%
% initialize the initial conditions
%
x0  = [pn0;pe0;pd0;u0;v0;w0;phi0;theta0;psi0;p0;q0;r0];

%
% str is always an empty matrix
%
str = [];

%
% initialize the array of sample times
%
ts  = [0 0];

% Specify the block simStateCompliance. The allowed values are:
%    'UnknownSimState', < The default setting; warn and assume DefaultSimState
%    'DefaultSimState', < Same sim state as a built-in block
%    'HasNoSimState',   < No sim state
%    'DisallowSimState' < Error out when saving or restoring the model sim state
simStateCompliance = 'UnknownSimState';

% end mdlInitializeSizes

%
%=============================================================================
% mdlDerivatives
% Return the derivatives for the continuous states.
%=============================================================================
%
function sys=mdlDerivatives(t,x,uu, mass,Jx,Jy,Jz,Jxz)

    pn    = x(1);
    pe    = x(2);
    pe    = x(3);
    u     = x(4);
    v     = x(5);
    w     = x(6);
    phi   = x(7);
    theta = x(8);
    psi   = x(9);
    p     = x(10);
    q     = x(11);
    r     = x(12);
    fx    = uu(1);
    fy    = uu(2);
    fz    = uu(3);
    ell   = uu(4);
    m     = uu(5);
    n     = uu(6);
    
    Gamma  = Jx*Jz-Jxz^2;
    Gamma1 = (Jxz*(Jx-Jy+Jz))/Gamma;
    Gamma2 = (Jz*(Jz-Jy)+Jxz*Jxz)/Gamma;
    Gamma3 = Jx/Gamma;
    Gamma4 = Jxz/Gamma;
    Gamma5 = (Jz-Jx)/Gamma;
    Gamma7 = (Jx*(Jx-Jy)+Jxz*Jxz)/Gamma;
    Gamma8 = Jx/Gamma;
    
    pndot = cos(theta)*cos(psi)*u... 
            + (sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi))*v... 
            + (cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi))*w;
    
    pedot = cos(theta)*sin(psi)*u...
            + (sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi))*v...
            + (cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi))*w;
    
    pddot = -sin(theta)*u...
            + sin(phi)*cos(theta)*v...
            + cos(phi)*cos(theta)*w;
    
    udot = r*v - q*w + (1/mass)*fx;
    
    vdot = p*w - r*u + (1/mass)*fy; 
    
    wdot = q*u - p*v + (1/mass)*fz;
    
    phidot = p + sin(phi)*tan(theta)*q + cos(phi)*tan(theta)*r;
    
    thetadot = cos(phi)*q - sin(phi)*r;
    
    psidot = (sin(phi)/cos(theta))*q + (cos(phi)/cos(theta))*r;
    
    pdot = Gamma1*p*q - Gamma2*q*r + Gamma3*ell + Gamma4*n;
    
    qdot = Gamma5*p*r - Gamma4*(p*p-r*r) + (1/Jy)*m;
    
    rdot = Gamma7*p*q - Gamma1*q*r + Gamma4*ell + Gamma8*n;
        

sys = [pndot; pedot; pddot; udot; vdot; wdot; phidot; thetadot; psidot; pdot; qdot; rdot];

% end mdlDerivatives

%
%=============================================================================
% mdlUpdate
% Handle discrete state updates, sample time hits, and major time step
% requirements.
%=============================================================================
%
function sys=mdlUpdate(t,x,u)

sys = [];

% end mdlUpdate

%
%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%
function sys=mdlOutputs(t,x,u)

sys = x;

% end mdlOutputs

%
%=============================================================================
% mdlGetTimeOfNextVarHit
% Return the time of the next hit for this block.  Note that the result is
% absolute time.  Note that this function is only used when you specify a
% variable discrete-time sample time [-2 0] in the sample time array in
% mdlInitializeSizes.
%=============================================================================
%
function sys=mdlGetTimeOfNextVarHit(t,x,u)

sampleTime = 1;    %  Example, set the next hit to be one second later.
sys = t + sampleTime;

% end mdlGetTimeOfNextVarHit

%
%=============================================================================
% mdlTerminate
% Perform any end of simulation tasks.
%=============================================================================
%
function sys=mdlTerminate(t,x,u)

sys = [];

% end mdlTerminate
