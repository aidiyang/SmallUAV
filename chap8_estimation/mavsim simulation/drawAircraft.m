function drawAircraft(uu)

    % process inputs to function
    pn       = uu(1);       % inertial North position     
    pd       = uu(2);       % inertial Down position
    u        = uu(3);       % body frame velocities
    w        = uu(4);       
    theta    = uu(5);       % pitch angle     
    q        = uu(6);      % pitch rate     
    t        = uu(7);      % time
    
    % define persistent variables 
    persistent aircraft_handle;  % figure handle for UAV
    persistent Vertices
    persistent Faces
    persistent facecolors
    persistent S


    % first time function is called, initialize plot and persistent vars
    if t==0,
        figure(1), clf
        scale = 1;
        [Vertices,Faces,facecolors] = defineAircraftBody(scale);
        aircraft_handle = drawBody(Vertices,Faces,facecolors,...
                                   pn,pd,theta,...
                                   [], 'normal');
        S = 200;
        title('UAV')
        xlabel('North')
        zlabel('Altitude')
        axis([pn-S/2, pn+S/2, 0, S]);
        grid on
        
    % at every other time step, redraw quadrotor and target
    else 
        drawBody(Vertices,Faces,facecolors,...
                     pn,pd,theta,...
                     aircraft_handle);
        aircraft_handle.Parent.XLim=[pn-S/2, pn+S/2];
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function handle = drawBody(V,F,colors,...
                               pn, pd, theta,...
                               handle, mode)
    V = rotate(V, theta);  % rotate rigid body  
    V = translate(V, pn, pd);  % translate after rotation

  % transform vertices from NED to XYZ (for matlab rendering)
  R = [...
      1, 0;...
      0, -1;...
      ];
  V = R*V;

  if isempty(handle),
    handle = patch('Vertices', V', 'Faces', F,...
                 'FaceVertexCData',colors,...
                 'FaceColor','flat',...
                 'EraseMode', mode);
  else
    set(handle,'Vertices',V','Faces',F);
    drawnow
  end
  
end 

%%%%%%%%%%%%%%%%%%%%%%%
function pts=rotate(pts,theta)

  % define rotation matrix (right handed)
  R_pitch = [...
          cos(theta), -sin(theta);...
          sin(theta), cos(theta)];
  R = R_pitch;  
    % note that R above either leaves the vector alone or rotates
    % a vector in a left handed rotation.  We want to rotate all
    % points in a right handed rotation, so we must transpose
  R = R';

  % rotate vertices
  pts = R*pts;
  
end
% end rotateVert

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% translate vertices by pn, pe, pd
function pts = translate(pts,pn,pd)

  pts = pts + repmat([pn;pd],1,size(pts,2));
  
end

% end translate


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% define aircraft vertices and faces
function [V,F,colors] = defineAircraftBody(scale)

    % parameters for drawing aircraft
    % colors
    %red     = [1, 0, 0];
    %green   = [0, 1, 0];
    blue    = [0, 0, 1];
    %yellow  = [1,1,0];
    %magenta = [0, 1, 1];
  

    % define vertices and faces for aircraft
    V = [...
        6, 1;...    % point 1
        7, 0;...    % point 2
        3, -2;...   % point 3
        -5, -1;...  % point 4
        -9, -3;...  % point 5
        -9, 1;...   % point 6
        ]';
  
    F = [...
        1, 2, 3, 4, 5, 6;... % nose-top
        ];  
  
  colors = [...
        blue;... 
        ];

  V = scale*V;   % rescale vertices
  end
  