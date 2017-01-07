% path_manager_dubins 
%   - follow Dubins airplane paths between waypoint configurations
%
% Modified:  
%   - 4/1/2010 - RWB
%   - 5/21/2013 - RWB
%
% input is:
%   num_waypoints - number of waypoint configurations
%   waypoints    - an array of dimension 5 by P.size_waypoint_array.
%                - the first num_waypoints rows define waypoint
%                  configurations
%                - format for each waypoint configuration:
%                  [wn, we, wd, chi_d, Va_d]
%                  where the (wn, we, wd) is the NED position of the
%                  waypoint, chi_d is the desired course at the waypoint,
%                  and Va_d is the desired airspeed along the path. 
%
% output is:
%   flag - if flag==1, follow straight-line path
%          if flag==2, follow spiral path 
%   Va_d - desired airspeed
%   c    - either an inertial point on stright-line or center of the spiral
%   psi  - either heading angle of straight-line or start angle for spiral
%   gam  - flight path angle for straight-line and spiral
%   rho  - radius of spiral
%   lambda = direction of spiral (+1 for CW, -1 for CCW)
%
function out = path_manager_dubins(in,P,start_of_simulation)

  NN = 0;
  num_waypoints = in(1+NN);
  waypoints = reshape(in(2+NN:5*P.size_waypoint_array+1+NN),5,P.size_waypoint_array);
  NN = NN + 1 + 5*P.size_waypoint_array;
  pn        = in(1+NN);
  pe        = in(2+NN);
  h         = in(3+NN);
  % Va      = in(4+NN);
  % alpha   = in(5+NN);
  % beta    = in(6+NN);
  % phi     = in(7+NN);
  % theta   = in(8+NN);
  chi     = in(9+NN);
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
 
  
  p = [pn; pe; -h];

  persistent waypoints_old   % stored copy of old waypoints
  persistent ptr_a           % waypoint pointer
  persistent state_transition % state of transition state machine
  persistent dubinspath
  persistent flag_need_new_waypoints % flag that request new waypoints from path planner
  persistent flag_first_time_in_state
  persistent halfplane
  persistent required_crossing
  
  if start_of_simulation,
      waypoints_old = zeros(5,P.size_waypoint_array);
      flag_need_new_waypoints = 0;
      state_transition = 0;
      flag_first_time_in_state = 1;
  end
  
  
  % if the waypoints have changed, update the waypoint pointer and plan new
  % Dubin's path
  if min(min(waypoints==waypoints_old))==0,
      waypoints_old = waypoints;
      state_transition = 1;
      ptr_a = 1;
      ptr_b = 2;
      start_node = [waypoints(1:4,ptr_a)', 0, 0];
      end_node   = [waypoints(1:4,ptr_b)', 0, 0];  
      dubinspath = dubinsParameters(start_node, end_node, P.R_min, P.gam_max);
      flag_need_new_waypoints = 0;
      flag_first_time_in_state = 1;
  end
  
  % define transition state machine
  switch state_transition,
      case 0, % beginning of simulation
          flag   = 1;
          Va_d   = P.Va0;
          c      = p;
          psi    = chi;
          gam    = 0;
          R      = 0;
          lambda = 0;
          if flag_first_time_in_state,
              flag_first_time_in_state =0;
          end
      
      case 1, % follow start spiral on Dubins path 
          flag   = 2;  % following spiral
          Va_d   = waypoints(5,ptr_a); % desired airspeed along waypoint path
          c      = dubinspath.c_s;
          psi    = dubinspath.psi_s;
          gam    = dubinspath.gam;
          R      = dubinspath.R;
          lambda = dubinspath.lam_s;
          
          if flag_first_time_in_state,
            halfplane = (p(1:2)-dubinspath.w_s(1:2))'*dubinspath.q_s(1:2);
            % determine number of required crossings of half plane
            if halfplane > 0, 
                required_crossing = 2*dubinspath.k_s+2;
            else
                required_crossing = 2*dubinspath.k_s+1;
            end
            flag_first_time_in_state=0;
          end
          
          % check for half plane crossing
          halfplane_tmp = (p(1:2,end)-dubinspath.w_s(1:2))'*dubinspath.q_s(1:2);
          if sign(halfplane)~=sign(halfplane_tmp),
              halfplane = halfplane_tmp;
              required_crossing = required_crossing-1;
          end
          
          % check to see if spiral is completed, transition to next segment
          if (required_crossing<=0) && (halfplane>0)
              switch (dubinspath.case),
                  case 1, state_transition = 3;
                  case 2, state_transition = 2;
                  case 3, state_transition = 3;
              end
              flag_first_time_in_state=1;
          end
                    
      case 2, % follow intermediate-start spiral on Dubins path 
          flag   = 2;  % following spiral
          Va_d   = waypoints(5,ptr_a); % desired airspeed along waypoint path
          c      = dubinspath.c_si;
          psi    = dubinspath.psi_si;
          gam    = dubinspath.gam;
          R      = dubinspath.R;
          lambda = dubinspath.lam_si;
          
          if flag_first_time_in_state,
            halfplane = (p(1:2)-dubinspath.w_si(1:2))'*dubinspath.q_si(1:2);
            % determine number of required crossings of half plane
            if halfplane > 0, 
                required_crossing = 2*dubinspath.k_si+2;
            else
                required_crossing = 2*dubinspath.k_si+1;
            end
            flag_first_time_in_state=0;
          end
          
          % check for half plane crossing
          halfplane_tmp = (p(1:2,end)-dubinspath.w_si(1:2))'*dubinspath.q_si(1:2);
          if sign(halfplane)~=sign(halfplane_tmp),
              halfplane = halfplane_tmp;
              required_crossing = required_crossing-1;
          end
          
          % check to see if spiral is completed, transition to next segment
          if (required_crossing<=0) && (halfplane>0)
              state_transition         = 3;
              flag_first_time_in_state = 1;
          end
          
      case 3, % follow straight line on Dubins path until intersect H2
          flag   = 1;  % line
          Va_d   = waypoints(5,ptr_a); % desired airspeed along waypoint path
          if dubinspath.case==2,
              w = dubinspath.w_si;
              q = dubinspath.q_si;
          else
              w = dubinspath.w_s;
              q  = dubinspath.q_s;
          end
          c      = w;
          psi    = atan2(q(2),q(1));
          gam    = -atan(q(3)/norm(q(1:2)));
          R      = dubinspath.R;
          lambda = 0;
          
          
          % check for half plane crossing
          halfplane = (p(1:2)-dubinspath.w_l(1:2))'*dubinspath.q_l(1:2);
          if halfplane>0,
              switch (dubinspath.case),
                  case 1, state_transition = 5;
                  case 2, state_transition = 5;
                  case 3, state_transition = 4;
              end
              flag_first_time_in_state = 1;
          end
          
      case 4, % follow intermediate-end spiral on Dubins path 
          flag   = 2;  % following spiral
          Va_d   = waypoints(5,ptr_a); % desired airspeed along waypoint path
          c      = dubinspath.c_ei;
          psi    = dubinspath.psi_ei;
          gam    = dubinspath.gam;
          R      = dubinspath.R;
          lambda = dubinspath.lam_ei;
          
          if flag_first_time_in_state,
            halfplane = (p(1:2)-dubinspath.w_ei(1:2))'*dubinspath.q_ei(1:2);
            % determine number of required crossings of half plane
            if halfplane > 0, 
                required_crossing = 2*dubinspath.k_ei+2;
            else
                required_crossing = 2*dubinspath.k_ei+1;
            end
            flag_first_time_in_state=0;
          end
          
          % check for half plane crossing
          halfplane_tmp = (p(1:2,end)-dubinspath.w_ei(1:2))'*dubinspath.q_ei(1:2);
          if sign(halfplane)~=sign(halfplane_tmp),
              halfplane = halfplane_tmp;
              required_crossing = required_crossing-1;
          end
          
          % check to see if spiral is completed, transition to next segment
          if (required_crossing<=0) && (halfplane>0)
              state_transition         = 5;
              flag_first_time_in_state = 1;
          end

      case 5, % follow end spiral on Dubins path 
          flag   = 2;  % following spiral
          Va_d   = waypoints(5,ptr_a); % desired airspeed along waypoint path
          c      = dubinspath.c_e;
          psi    = dubinspath.psi_e;
          gam    = dubinspath.gam;
          R      = dubinspath.R;
          lambda = dubinspath.lam_e;
          
          if flag_first_time_in_state,
            halfplane = (p(1:2)-dubinspath.w_e(1:2))'*dubinspath.q_e(1:2);
            % determine number of required crossings of half plane
            if halfplane > 0, 
                required_crossing = 2*dubinspath.k_e+2;
            else
                required_crossing = 2*dubinspath.k_e+1;
            end
            flag_first_time_in_state=0;
          end
          
          % check for half plane crossing
          halfplane_tmp = (p(1:2,end)-dubinspath.w_e(1:2))'*dubinspath.q_e(1:2);
          if sign(halfplane)~=sign(halfplane_tmp),
              halfplane = halfplane_tmp;
              required_crossing = required_crossing-1;
          end
          
          % check to see if spiral is completed, transition to next segment
          if (required_crossing<=0) && (halfplane>0)
              % increase the waypoint pointer
              if ptr_a==num_waypoints-1,
                  flag_need_new_waypoints = 1;
                else
                  ptr_a = ptr_a+1;
                  ptr_b = ptr_a+1;
                  % plan new Dubin's path to next waypoint configuration
                  start_node = [waypoints(1:4,ptr_a)', 0, 0];
                  end_node   = [waypoints(1:4,ptr_b)', 0, 0];      
                  dubinspath = dubinsParameters(start_node, end_node, P.R_min, P.gam_max);    
                  state_transition = 1;
                  flag_first_time_in_state = 1;
              end

          end
  end
  
  out = [flag; Va_d; c; psi; gam; R; lambda; state; flag_need_new_waypoints];
  
end

