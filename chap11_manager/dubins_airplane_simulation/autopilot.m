function y = autopilot(uu,P)
%
% autopilot for mavsim
% 
% Modification History:
%   2/11/2010 - RWB
%   5/14/2010 - RWB
%   4/1/2013  - RWB
%   

    % process inputs
    NN = 0;
%    pn       = uu(1+NN);  % inertial North position
%    pe       = uu(2+NN);  % inertial East position
    h        = uu(3+NN);  % altitude
    Va       = uu(4+NN);  % airspeed
%    alpha    = uu(5+NN);  % angle of attack
%    beta     = uu(6+NN);  % side slip angle
    phi      = uu(7+NN);  % roll angle
    theta    = uu(8+NN);  % pitch angle
    chi      = uu(9+NN);  % course angle
    p        = uu(10+NN); % body frame roll rate
    q        = uu(11+NN); % body frame pitch rate
    r        = uu(12+NN); % body frame yaw rate
%    Vg       = uu(13+NN); % ground speed
%    wn       = uu(14+NN); % wind North
%    we       = uu(15+NN); % wind East
%    psi      = uu(16+NN); % heading
%    bx       = uu(17+NN); % x-gyro bias
%    by       = uu(18+NN); % y-gyro bias
%    bz       = uu(19+NN); % z-gyro bias
    NN = NN+19;
    Va_c     = uu(1+NN);  % commanded airspeed (m/s)
    gamma_c  = uu(2+NN);  % commanded flight path angle (rad)
    phi_c    = uu(3+NN);  % commanded roll angle (rad)
    NN = NN+3;
    t        = uu(1+NN);   % time
    

    %----------------------------------------------------------
    % lateral autopilot
    
    
    if t==0,
        % assume no rudder, therefore set delta_r=0
        delta_r = 0;%coordinated_turn_hold(beta, 1, P);
        % use commanded roll angle to regulate heading
%        phi_c   = course_hold(chi_c, chi, r, 1, P);
        % use aileron to regulate roll angle
        delta_a = roll_hold(phi_c, phi, p, 1, P);     

    else
        delta_r = 0;%coordinated_turn_hold(beta, 0, P);
%        phi_c   = course_hold(chi_c, chi, r, 0, P);
        delta_a = roll_hold(phi_c, phi, p, 0, P);
    end
  
    
    %----------------------------------------------------------
    % longitudinal autopilot
    
%     % define persistent variable for state of altitude state machine
%     persistent altitude_state;
%     persistent initialize_integrator;
%     % initialize persistent variable
%     if t==0,
%         if h<=P.altitude_take_off_zone,     
%             altitude_state = 1;
%         elseif h<=h_c-P.altitude_hold_zone, 
%             altitude_state = 2;
%         elseif h>=h_c+P.altitude_hold_zone, 
%             altitude_state = 3;
%         else
%             altitude_state = 4;
%         end
%         initialize_integrator = 1;
%     end
%     
%     % implement state machine
%     switch altitude_state,
%         case 1,  % in take-off zone
%             delta_t = 1;
%             theta_c = 30*pi/180;
%             if h>=P.altitude_take_off_zone,
%                 altitude_state = 2;
%                 initialize_integrator = 1;
%             else
%                 initialize_integrator = 0;
%             end
%             
%         case 2,  % climb zone
%             delta_t = 1;
%             theta_c = airspeed_with_pitch_hold(Va_c, Va, initialize_integrator, P);
%             if h>=h_c-P.altitude_hold_zone,
%                 altitude_state = 4;
%                 initialize_integrator = 1;
%             elseif h<=P.altitude_take_off_zone,
%                 altitude_state = 1;
%                 initialize_integrator = 1;
%             else
%                 initialize_integrator = 0;
%             end
%             
%         case 3, % descend zone
%             delta_t = 0;
%             theta_c = airspeed_with_pitch_hold(Va_c, Va, initialize_integrator, P);
%             if h<=h_c+P.altitude_hold_zone,
%                 altitude_state = 4;
%                 initialize_integrator = 1;
%             else
%                 initialize_integrator = 0;
%             end
%         case 4, % altitude hold zone
%             delta_t = airspeed_with_throttle_hold(Va_c, Va, initialize_integrator, P);
%             theta_c = altitude_hold(h_c, h, initialize_integrator, P);
%             if h<=h_c-P.altitude_hold_zone,
%                 altitude_state = 2;
%                 initialize_integrator = 1;
%             elseif h>=h_c+P.altitude_hold_zone,
%                 altitude_state = 3;
%                 initialize_integrator = 1;
%             else
%                 initialize_integrator = 0;
%             end
%     end
%     
    hdot_c  = Va*sin(gamma_c);
    if t==0,
        [theta_c,hdot] = climbrate_with_pitch_hold(hdot_c, h, 1, P);
        delta_e = pitch_hold(theta_c, theta, q, 1, P);
        delta_t = airspeed_with_throttle_hold(Va_c, Va, 1, P);             
    else
        [theta_c,hdot] = climbrate_with_pitch_hold(hdot_c, h, 0, P);
        delta_e = pitch_hold(theta_c, theta, q, 0, P);
        delta_t = airspeed_with_throttle_hold(Va_c, Va, 0, P);             
    end
        
    
    
    %----------------------------------------------------------
    % create outputs
    
    % control outputs
    delta = [delta_e; delta_a; delta_r; delta_t];
    % commanded (desired) states
    x_command = [...
        0;...                    % pn
        0;...                    % pe
        0;...                    % h
        Va_c;...                 % Va
        0;...                    % alpha
        0;...                    % beta
        phi_c;...                % phi
        theta_c;...              % theta
        hdot_c;...               % hdot
        0;...                    % p
        0;...                    % q
        hdot;...                 % hdot
        ];
            
    y = [delta; x_command];
 
end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Autopilot functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% climbrate_with_pitch_hold
%   - regulate climbrate using pitch angle
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [theta_c,hdot_] = climbrate_with_pitch_hold(hdot_c, h, flag, P)
  persistent integrator;
  persistent differentiator;
  persistent differentiator_d1;
  persistent error_d1;
  persistent hdot;
  persistent h_d1;
  % initialize persistent variables at beginning of simulation
  if flag==1,
      integrator        = 0; 
      differentiator    = 0;
      differentiator_d1 = 0;
      error_d1          = 0; 
      hdot              = 0;
      h_d1              = h;
  end
  
  % compute current climbrate using dirty derivative
  hdot = ((2*P.tau-P.Ts)*hdot + 2*(h-h_d1))/(2*P.tau+P.Ts);
  
 
  % compute the current error
  error = hdot_c - hdot;
  
  % update the integrator
  integrator = integrator + (P.Ts/2)*(error + error_d1); % trapazoidal rule
  
  % update the differentiator
  differentiator = (2*P.tau-P.Ts)/(2*P.tau+P.Ts)*differentiator_d1...
      + (2/(2*P.tau+P.Ts))*(error - error_d1);
  
  % proportional term
  up = P.climbrate_pitch_kp * error;
  
  % integral term
  ui = P.climbrate_pitch_ki * integrator;
  
  % derivative term
  ud = P.climbrate_pitch_kd * differentiator;
  
  
  % implement PID control
  theta_c = sat(up + ui + ud, 30*pi/180, -30*pi/180);
  
  % implement integrator antiwindup
  if P.climbrate_pitch_ki~=0,
    theta_c_unsat = up + ui + ud;
    k_antiwindup = P.Ts/P.climbrate_pitch_ki;
    integrator = integrator + k_antiwindup*(theta_c-theta_c_unsat);
  end

  % update persistent variables
  error_d1 = error;
  differentiator_d1 = differentiator;
  h_d1 = h;

  hdot_ = hdot;
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% course_hold
%   - regulate heading using the roll command
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function phi_c = course_hold(chi_c, chi, r, flag, P)
  persistent integrator;
  persistent error_d1;
  % initialize persistent variables at beginning of simulation
  if flag==1,
      integrator = 0; 
      error_d1   = 0; % error at last sample (d1-delayed by one sample)
  end
 
  % compute the current error
  error = chi_c - chi;
  
  % update the integrator
  if abs(error)>15*pi/180,
      integrator = 0;
  else
      integrator = integrator + (P.Ts/2)*(error + error_d1); % trapazoidal rule
  end
  
  % proportional term
  up = P.course_kp * error;
  
  % integral term
  ui = P.course_ki * integrator;
  
  % derivative term
  ud = -P.course_kd*r;
  
  
  % implement PID control
  phi_c = sat(up + ui + ud, 45*pi/180, -45*pi/180);
  
  % implement integrator anti-windup
  if P.course_ki~=0,
    phi_c_unsat = up+ui+ud;
    k_antiwindup = P.Ts/P.course_ki;
    integrator = integrator + k_antiwindup*(phi_c-phi_c_unsat);
  end

  % update persistent variables
  error_d1 = error;

end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% roll_hold
%   - regulate roll using aileron
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function delta_a = roll_hold(phi_c, phi, p, flag, P)
  persistent integrator;
  persistent error_d1;
  % initialize persistent variables at beginning of simulation
  if flag==1,
      integrator = 0; 
      error_d1   = 0; % error at last sample (d1-delayed by one sample)
  end
 
  % compute the current error
  error = phi_c - phi;
  
  % update the integrator
  integrator = integrator + (P.Ts/2)*(error + error_d1); % trapazoidal rule
  
  % proportional term
  up = P.roll_kp * error;
  
  % integral term
  ui = P.roll_ki * integrator;
  
  % derivative term
  ud = -P.roll_kd*p;
  
  
  % implement PID control
  delta_a = sat(up + ui + ud, 45*pi/180, -45*pi/180);
  
  % implement integrator anti-windup
  if P.roll_ki~=0,
    delta_a_unsat = up + ui + ud;
    k_antiwindup=P.Ts/P.roll_ki;
    integrator = integrator + k_antiwindup*(delta_a - delta_a_unsat);
  end
  
  % update persistent variables
  error_d1 = error;

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% pitch_hold
%   - regulate pitch using elevator
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function delta_e = pitch_hold(theta_c, theta, q, flag, P)
  persistent integrator;
  persistent error_d1;
  % initialize persistent variables at beginning of simulation
  if flag==1,
      integrator = 0; 
      error_d1   = 0; % error at last sample (d1-delayed by one sample)
  end
 
  % compute the current error
  error = theta_c - theta;
  
  % update the integrator
  integrator = integrator + (P.Ts/2)*(error + error_d1); % trapazoidal rule
  
  % proportional term
  up = P.pitch_kp * error;
  
  % integral term
  ui = P.pitch_ki * integrator;
  
  % derivative term
  ud = -P.pitch_kd * q;
  
  
  % implement PID control
  delta_e = sat(up + ui + ud, 45*pi/180, -45*pi/180);
  
  % implement integrator anti-windup
  if P.pitch_ki~=0,
    delta_e_unsat = up + ui + ud;
    k_antiwindup = P.Ts/P.pitch_ki;
    integrator = integrator + k_antiwindup*(delta_e-delta_e_unsat);
  end
  
  % update persistent variables
  error_d1 = error;

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% airspeed_with_pitch_hold
%   - regulate airspeed using pitch angle
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function theta_c = airspeed_with_pitch_hold(Va_c, Va, flag, P)
  persistent integrator;
  persistent differentiator;
  persistent differentiator_d1;
  persistent error_d1;
  % initialize persistent variables at beginning of simulation
  if flag==1,
      integrator = 0; 
      differentiator = 0;
      differentiator_d1 = 0;
      error_d1   = 0; 
  end
 
  % compute the current error
  error = Va_c - Va;
  
  % update the integrator
  integrator = integrator + (P.Ts/2)*(error + error_d1); % trapazoidal rule
  
  % update the differentiator
  differentiator = (2*P.tau-P.Ts)/(2*P.tau+P.Ts)*differentiator_d1...
      + (2/(2*P.tau+P.Ts))*(error - error_d1);
  
  % proportional term
  up = P.airspeed_pitch_kp * error;
  
  % integral term
  ui = P.airspeed_pitch_ki * integrator;
  
  % derivative term
  ud = P.airspeed_pitch_kd * differentiator;
  
  
  % implement PID control
  theta_c = sat(up + ui + ud, 30*pi/180, -30*pi/180);
  
  % implement integrator antiwindup
  if P.airspeed_pitch_ki~=0,
    theta_c_unsat = up + ui + ud;
    k_antiwindup = P.Ts/P.airspeed_pitch_ki;
    integrator = integrator + k_antiwindup*(theta_c-theta_c_unsat);
  end

  % update persistent variables
  error_d1 = error;
  differentiator_d1 = differentiator;

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% airspeed_with_throttle_hold
%   - regulate airspeed using throttle
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function delta_t = airspeed_with_throttle_hold(Va_c, Va, flag, P)
  persistent integrator;
  persistent differentiator;
  persistent differentiator_d1;
  persistent error_d1;
  % initialize persistent variables at beginning of simulation
  if flag==1,
      integrator = 0; 
      differentiator = 0;
      differentiator_d1 = 0;
      error_d1   = 0; 
  end
 
  % compute the current error
  error = Va_c - Va;
  
  % update the integrator
  integrator = integrator + (P.Ts/2)*(error + error_d1); % trapazoidal rule
  
  % update the differentiator
  differentiator = (2*P.tau-P.Ts)/(2*P.tau+P.Ts)*differentiator_d1...
      + (2/(2*P.tau+P.Ts))*(error - error_d1);
  
  % proportional term
  up = P.airspeed_throttle_kp * error;
  
  % integral term
  ui = P.airspeed_throttle_ki * integrator;
  
  % derivative term
  ud = P.airspeed_throttle_kd * differentiator;  
  
  % implement PID control
  delta_t = sat(P.u_trim(4)+up + ui + ud, 1, 0);
  
  % implement integrator anti-windup
  if P.airspeed_throttle_ki~=0,
    delta_t_unsat = P.u_trim(4) + up + ui + ud;
    k_antiwindup = P.Ts/P.airspeed_throttle_ki;
    integrator = integrator + k_antiwindup*(delta_t-delta_t_unsat);
  end
  
  % update persistent variables
  error_d1 = error;
  differentiator_d1 = differentiator;

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% altitude_hold
%   - regulate altitude using pitch angle
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function theta_c = altitude_hold(h_c, h, flag, P)
  persistent integrator;
  persistent differentiator;
  persistent differentiator_d1;
  persistent error_d1;
  % initialize persistent variables at beginning of simulation
  if flag==1,
      integrator = 0; 
      differentiator = 0;
      differentiator_d1 = 0;
      error_d1   = 0; 
  end
 
  % compute the current error
  error = h_c - h;
  
  % update the integrator
  integrator = integrator + (P.Ts/2)*(error + error_d1); % trapazoidal rule
  
  % update the differentiator
  differentiator = (2*P.tau-P.Ts)/(2*P.tau+P.Ts)*differentiator_d1...
      + (2/(2*P.tau+P.Ts))*(error - error_d1);
  
  % proportional term
  up = P.altitude_kp * error;
  
  % integral term
  ui = P.altitude_ki * integrator;
  
  % derivative term
  ud = P.altitude_kd * differentiator;
  
  
  % implement PID control
  theta_c = sat(up + ui + ud, 30*pi/180, -30*pi/180);
  
  % implement integrator anti-windup
  if P.altitude_ki~=0,
    theta_c_unsat = up + ui + ud;
    k_antiwindup = P.Ts/P.altitude_ki;
    integrator = integrator + k_antiwindup*(theta_c-theta_c_unsat);
  end
  
  % update persistent variables
  error_d1 = error;
  differentiator_d1 = differentiator;

end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% coordinated_turn_hold
%   - sideslip with rudder
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function theta_r = coordinated_turn_hold(v, flag, P)
  persistent integrator;
  persistent error_d1;
  % initialize persistent variables at beginning of simulation
  if flag==1,
      integrator = 0; 
      error_d1   = 0; 
  end
 
  % compute the current error
  error = -v;
  
  % update the integrator
  integrator = integrator + (P.Ts/2)*(error + error_d1); % trapazoidal rule
  
  % proportional term
  up = P.sideslip_kp * error;
  
  % integral term
  ui = P.sideslip_ki * integrator;
  
  % derivative term
  ud = 0;%-P.sideslip_kd * r;
  
  
  % implement PID control
  theta_r = sat(up + ui + ud, 30*pi/180, -30*pi/180);
  
  % implement integrator antiwindup
  if P.sideslip_ki~=0,
    theta_r_unsat = up + ui + ud;
    k_antiwindup = P.Ts/P.sideslip_ki;
    integrator = integrator + k_antiwindup*(theta_r-theta_r_unsat);
  end
  
  % update persistent variables
  error_d1 = error;
 
end

  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% sat
%   - saturation function
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function out = sat(in, up_limit, low_limit)
  if in > up_limit,
      out = up_limit;
  elseif in < low_limit;
      out = low_limit;
  else
      out = in;
  end
end
  
 