% path planner
%
% Modified:  
%   - 4/06/2010 - RWB
%   - 5/21/2013 - RWB
%
% output is a vector containing P.num_waypoints waypoints
%
% input is the map of the environment
function out = path_planner_foo(in,P,map)

  NN = 0;
  % pn        = in(1+NN);
  % pe        = in(2+NN);
  % h         = in(3+NN);
  % Va      = in(4+NN);
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
  % flag_new_waypoints =  in(17+NN);
  NN = NN + 17;
  % t         =  in(1+NN);


  % format for each point is [pn, pe, pd, chi, Va^d] where the position
  % of the waypoint is (pn, pe, pd), the desired course at the waypoint
  % is chi, and the desired airspeed between waypoints is Va
      
  switch 4,
      case 1,
          num_waypoints = 16;
          wpp = [...
                0,   0,   -100,    0*pi/180,     P.Va0;...
                100, 100, -125,   90*pi/180,     P.Va0;... % short climb RSR
                0,   200, -150,  180*pi/180,     P.Va0;... % short climb RSL
                100, 200, -175,    0*pi/180,     P.Va0;... % short climb LSR
                200, 100, -200,  -90*pi/180,     P.Va0;... % short climb LSL
                200, 200, -350,  180*pi/180,     P.Va0;... % long climb RSR
                100, 100, -550,  180*pi/180,     P.Va0;... % long climb RSL
                0,   200, -350,  180*pi/180,     P.Va0;... % long descend LSR
                100, 200, -150,  -90*pi/180,     P.Va0;... % long descend LSL
                200, 100, -250,    0*pi/180,     P.Va0;... % intermediate climb RLSR (climb at beginning)
                100,   0, -350,  180*pi/180,     P.Va0;... % intermediate climb LRSL (climb at beginning)
                0,   100, -450,  180*pi/180,     P.Va0;... % intermediate climb LRSR (climb at beginning)
                100,  50, -550,  -90*pi/180,     P.Va0;... % intermediate climb RLSL (climb at beginning)
                200,   0, -450,   90*pi/180,     P.Va0;... % intermediate climb RSLR (descend at end)
                100, 100, -350,   90*pi/180,     P.Va0;... % intermediate climb RSRL (descend at end)          
                200, 100, -250,  -90*pi/180,     P.Va0;... % intermediate climb LSRL (descend at end)        
                100, 100, -150,  180*pi/180,     P.Va0;... % intermediate climb LSLR (descend at end)
                ];
      case 2, % simulation - low altitude case
          num_waypoints = 1;
          wpp = [...
                0,   0,   -100,    0*pi/180,     P.Va0;...
                0, 200,   -125,   270*pi/180,     P.Va0;... 
                ];
      case 3, % simulation - high altitude case
          num_waypoints = 1;
          wpp = [...
                0,   0,   -100,    0*pi/180,     P.Va0;...
                0, 200,   -250,  270*pi/180,     P.Va0;... 
                ];           
      case 4,
          num_waypoints = 1;
          wpp = [...
                0,   0,   -100,    0*pi/180,     P.Va0;...
                0, 200,   -200,  270*pi/180,     P.Va0;... 
                ];
  end
 
  for i=(num_waypoints+1):P.size_waypoint_array-1,
      wpp = [...
          wpp;...
          -9999, -9999, -9999, -9999, -9999;...
          ];
  end
  
  out = [num_waypoints; reshape(wpp', 5*P.size_waypoint_array, 1)]; 

end