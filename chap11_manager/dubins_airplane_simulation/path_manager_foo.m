% path manager
%
% Modified:  
%   - 3/25/2010 - RWB
%   - 4/3/2013 - RWB
%
% output is:
%   flag - if flag==1, follow waypoint path
%          if flag==2, follow orbit
%   
%   Va^d - desired airspeed
%   c_ell - inertial position of start of line path
%   psi_q - desired course direction of line path
%   gam_q - desired flight path angle of line path
%   c    - center of orbit
%   rho  - radius of orbit
%   lambda = direction of orbit (+1 for CW, -1 for CCW)
%
function out = path_manager_foo(in,P)

  NN = 0;
  waypoints = in(1+NN);
  NN = NN + 1;
  pn        = in(1+NN);
  pe        = in(2+NN);
  h         = in(3+NN);
  Va        = in(4+NN);
  % alpha   = in(5+NN);
  % beta    = in(6+NN);
  % phi     = in(7+NN);
  % theta   = in(8+NN);
  % chi     = in(9+NN);
  % p       = in(10+NN);
  % q       = in(11+NN);
  % r       = in(12+NN);
  % Vg      = in(13+NN);
  % wn      = in(14+NN);
  % we      = in(15+NN);
  % psi     = in(16+NN);
  state     =  in(1+NN:16+NN);  
  NN = NN + 16;
  t         = in(1+NN);
 

  flag = 2;
  switch flag
      case 1, % define straight-line path      
          flag     = 1;              % flag==1 implies straight line path
          Va_d     = 13;             % desired airspeed
          c        = [300; 100; -100];  % point on line
          psi      = 110*pi/180;     % heading of line
          gam      = 10*pi/180;      % flight path angle of line (climbrate)
          rho      = 0;              % not used for straight-line path
          lambda   = 0;
      
      case 2,  % define spiral path
          flag     = 2;             % flag==1 implies spiral path
          Va_d     = 13;            % desired airspeed
          c        = [100; 100; -100];     % center of spiral
          psi      = 0*pi/180;      % start angle for spiral
          gam      = 10*pi/180;     % climbrate of spiral
          rho      = 200;           % radius of spiral
          lambda   = -1;             % direction of spiral (+1->CW, -1->CCW)
   end

   flag_need_new_waypoints=0;
   out = [flag; Va_d; c; psi; gam; rho; lambda; state; flag_need_new_waypoints];    

end