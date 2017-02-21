function [sys,x0,str,ts,simStateCompliance] = mav_dynamics_VTOL(t,x,u,flag,P)

switch flag,

  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0,
    [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(P);

  %%%%%%%%%%%%%%%
  % Derivatives %
  %%%%%%%%%%%%%%%
  case 1,
    sys=mdlDerivatives(t,x,u,P);

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
function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(P)

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
% initialize the initial conditions     ?????? ???????12?????
%
x0  = [...
    P.pn0;...
    P.pe0;...
    P.pd0;...
    P.u0;...
    P.v0;...
    P.w0;...
    P.phi0;...
    P.theta0;...
    P.psi0;...
    P.p0;...
    P.q0;...
    P.r0;...
    ];

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
function sys=mdlDerivatives(t,x,uu,P)

    pn    = x(1);
    pe    = x(2);
    pd    = x(3);
    u     = x(4);
    v     = x(5);
    w     = x(6);
    phi   = x(7);      % roll angle (degrees)   
    theta = x(8);      % pitch angle (degrees)
    psi   = x(9);      % yaw angle (degrees)
    p     = x(10);
    q     = x(11);
    r     = x(12);
    fx    = uu(1);
    fy    = uu(2);
    fz    = uu(3);
    ell   = uu(4);
    m     = uu(5);
    n     = uu(6);
    
% physical parameters of airframe    
    mass= P.mass;
    Jx = P.Jx;
    Jy = P.Jy;
    Jz = P.Jz;
    Jxz = P.Jxz;
    
    G=Jx*Jz-Jxz^2;
    G1 = Jxz*(Jx-Jy+Jz)/G;
    G2 = (Jz*(Jz-Jy)+Jxz^2)/G;
    G3 = Jz/G;
    G4 = Jxz/G;
    G5 = (Jz-Jx)/Jy;
    G6 = Jxz/ Jy;
    G7 = ((Jx-Jy)*Jx+Jxz^2)/G;
    G8 = Jx/G;
% body -> Nav   Cnb
    C11=cos(theta)*cos(psi);
    C12=sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi);
    C13=cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi);

    C21=cos(theta)*sin(psi);
    C22=sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi);
    C23=cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi);

    C31=-sin(theta);
    C32=sin(phi)*cos(theta);
    C33=cos(phi)*cos(theta);

% 6??? 12????????????????

    pndot = C11*u + C12*v + C13*w
    pedot = C21*u + C22*v + C23*w;
    pddot = C31*u + C32*v + C33*w;
    
    udot = r*v - q*w + fx/mass    %mass=1.534kg
    vdot = p*w - r*u + fy/mass
    wdot = q*u - p*v + fz/mass
    
    phidot = p + sin(phi)*tan(theta)*q + cos(phi)*tan(theta)*r;
    thetadot = cos(phi)*q - sin(phi)*r;
    psidot = sin(phi)/cos(theta)*p + cos(phi)/cos(theta)*r;
    
%     pdot = (P.Jz - P.Jy)/P.Jx *q*r + ell/P.Jx ;
%     qdot = (P.Jz - P.Jx)/P.Jy *p*r +   m/P.Jy ;
%     rdot = (P.Jx - P.Jy)/P.Jz *p*q +   n/P.Jz ;
    
    pdot = G1*p*q - G2*q*r + G3*ell + G4 *n; 
    qdot = G5*p*r - G6*(p^2-r^2)     + 1/Jy *m;
    rdot = G7*p*q - G1*q*r + G4*ell  + G8 *n;
 
% P.Jx   = 0.1147 %kg-m^2
% P.Jy   = 0.0576 %kg-m^2
% P.Jz   = 0.1712 %kg-m^2
% P.Jxz  = 0.0015 %kg-m^2
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
