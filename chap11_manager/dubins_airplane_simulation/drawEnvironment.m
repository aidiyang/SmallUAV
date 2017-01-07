function drawEnvironment(uu,V,F,colors,map,R_min)

    % process inputs to function
    NN = 0;
    pn       = uu(1+NN);       % inertial North position     
    pe       = uu(2+NN);       % inertial East position
    pd       = uu(3+NN);       % inertial Down position
    u        = uu(4+NN);       % body frame velocities
    v        = uu(5+NN);       
    w        = uu(6+NN);       
    phi      = uu(7+NN);       % roll angle         
    theta    = uu(8+NN);       % pitch angle     
    psi      = uu(9+NN);       % yaw angle     
    p        = uu(10+NN);      % roll rate
    q        = uu(11+NN);      % pitch rate     
    r        = uu(12+NN);      % yaw rate    
    t        = uu(13+NN);      % time
    
    NN = NN + 13;
    path     = uu(1+NN:9+NN); 
    NN = NN + 9;
    num_waypoints = uu(1+NN);
    waypoints     = reshape(uu(2+NN:5*num_waypoints+1+NN),5,num_waypoints)'; 


    % define persistent variables 
    persistent aircraft_handle;  % figure handle for MAV
    persistent path_handle;      % handle for straight-line or orbit path
    persistent waypoint_handle;  % handle for waypoints

    S = 500; % plot size
    
    % first time function is called, initialize plot and persistent vars
    if t==0,

        figure(1), clf
                
        aircraft_handle = drawBody(V,F,colors,...
                                   pn,pe,pd,phi,theta,psi,...
                                   [], 'normal');
        hold on
%        waypoint_handle = drawWaypoints(waypoints, R_min, [], 'normal');
        path_handle = drawPath(path, S/2, [], 'normal');
        drawMap(map);
        
        title('UAV')
        xlabel('East')
        ylabel('North')
        zlabel('-Down')
 %       axis([-map.width/5,map.width,-map.width/5,map.width,0,3*map.MaxHeight]);
        axis([-map.width/5,map.width,-map.width/5,map.width,0,map.width]);
        view(-40,70)  % set the view angle for figure
        grid on
        
        
    % at every other time step, redraw MAV
    else 
        drawBody(V,F,colors,...
                     pn,pe,pd,phi,theta,psi,...
                     aircraft_handle);
%        drawWaypoints(waypoints, R_min, waypoint_handle);
        drawPath(path, S, path_handle);

    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function handle = drawBody(V,F,colors,...
                               pn, pe, pd, phi, theta, psi,...
                               handle, mode)
  V = rotate(V', phi, theta, psi)';  % rotate rigid body  
  V = translate(V', pn, pe, pd)';  % translate after rotation

  % transform vertices from NED to XYZ (for matlab rendering)
  R = [...
      0, 1, 0;...
      1, 0, 0;...
      0, 0, -1;...
      ];
  V = V*R;

  if isempty(handle),
    handle = patch('Vertices', V, 'Faces', F,...
                 'FaceVertexCData',colors,...
                 'FaceColor','flat',...
                 'EraseMode', mode);
  else
    set(handle,'Vertices',V,'Faces',F);
    drawnow
  end
  
end 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function handle = drawPath(path, S, handle, mode)
    flag = path(1); 
    c   = [path(3); path(4); path(5)];
    psi = path(6);
    gam = path(7);
    rho  = path(8);
    lam  = path(9);

    switch flag,
        case 1,
            q  = [cos(psi)*cos(gam); sin(psi)*cos(gam); -sin(gam)];
            XX = [c(1), c(1)+S*q(1)];
            YY = [c(2), c(2)+S*q(2)];
            ZZ = [c(3), c(3)+S*q(3)];
        case 2,
            N = 100;
            t = [0:.1:6*pi];
            XX = c(1) + rho*cos(lam*t+psi);
            YY = c(2) + rho*sin(lam*t+psi);
            %ZZ = c(3) - t*sin(gam);
            ZZ = c(3) - t*rho*tan(gam);
    end
    
    if isempty(handle),
        handle = plot3(YY,XX,-ZZ,'r', 'EraseMode', mode);
    else
        set(handle,'XData', YY, 'YData', XX, 'ZData', -ZZ);
        drawnow
    end
end 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function handle = drawWaypoints(waypoints, R_min, handle, mode)

    if waypoints(1,4)==-9999, % check to see if Dubins paths
        XX = [waypoints(:,1)];
        YY = [waypoints(:,2)];
        ZZ = [waypoints(:,3)];
    else
        XX = [];
        YY = [];
        for i=2:size(waypoints,1),
            dubinspath = dubinsParameters(waypoints(i-1,:),waypoints(i,:),R_min);
            [tmpX,tmpY] = pointsAlongDubinsPath(dubinspath,0.1);
            XX = [XX; tmpX];
            YY = [YY; tmpY];     
        end
        ZZ = waypoints(i,3)*ones(size(XX));
    end
    
    if isempty(handle),
        handle = plot3(YY,XX,-ZZ,'b', 'EraseMode', mode);
    else
        set(handle,'XData', YY, 'YData', XX, 'ZData', -ZZ);
        drawnow
    end
end 


%%%%%%%%%%%%%%%%%%%%%%%
function XYZ=rotate(XYZ,phi,theta,psi)
  % define rotation matrix
  R_roll = [...
          1, 0, 0;...
          0, cos(phi), -sin(phi);...
          0, sin(phi), cos(phi)];
  R_pitch = [...
          cos(theta), 0, sin(theta);...
          0, 1, 0;...
          -sin(theta), 0, cos(theta)];
  R_yaw = [...
          cos(psi), -sin(psi), 0;...
          sin(psi), cos(psi), 0;...
          0, 0, 1];

  % rotate vertices
  XYZ = R_yaw*R_pitch*R_roll*XYZ;
  
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% translate vertices by pn, pe, pd
function XYZ = translate(XYZ,pn,pe,pd)

  XYZ = XYZ + repmat([pn;pe;pd],1,size(XYZ,2));
  
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% drawMap
%   plot obstacles and path
function drawMap(map)%,path,smoothedPath,tree,R_min)
  
 
  % draw buildings 
  V = [];
  F = [];
  patchcolors = [];
  count = 0;
  for i=1:map.NumBlocks,
      for j=1:map.NumBlocks,
        [Vtemp,Ftemp,patchcolorstemp] = buildingVertFace(map.buildings_n(i),...
            map.buildings_e(j),map.BuildingWidth,map.heights(j,i));
        V = [V; Vtemp];
        Ftemp = Ftemp + count;
        F = [F; Ftemp];
        count = count + 8;
        patchcolors = [patchcolors;patchcolorstemp];
      end
  end
  
  patch('Vertices', V, 'Faces', F,...
                 'FaceVertexCData',patchcolors,...
                 'FaceColor','flat');
 

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% buildingVertFace(x,y,width,height)
%%   define patches for a building located at (x,y)
function [V,F,patchcolors] = buildingVertFace(n,e,width,height)
 
  % vertices of the building
  V = [...
        e+width/2, n+width/2, 0;...
        e+width/2, n-width/2, 0;...
        e-width/2, n-width/2, 0;...
        e-width/2, n+width/2, 0;...
        e+width/2, n+width/2, height;...
        e+width/2, n-width/2, height;...
        e-width/2, n-width/2, height;...
        e-width/2, n+width/2, height;...
        ];    
  % define faces of fuselage
  F = [...
        1, 4, 8, 5;... % North Side
        1, 2, 6, 5;... % East Side
        2, 3, 7, 6;... % South Side
        3, 4, 8, 7;... % West Side
        5, 6, 7, 8;... % Top
        ];   

  myred = [1, 0, 0];
  mygreen = [0, 1, 0];
  myblue = [0, 0, 1];
  myyellow = [1,1,0];
  mymagenta   = [0, 1, 1];

  patchcolors = [...
    mygreen;... % North
    mygreen;... % East
    mygreen;... % South
    mygreen;... % West
    myyellow;...  % Top
    ];

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% pointsAlongDubinsPath
%%   Find points along Dubin's path separted by Del (to be used in
%%   collision detection)
function [X,Y] = pointsAlongDubinsPath(dubinspath,Del)


  % points along start circle
  th1 = mod(atan2(dubinspath.ps(2)-dubinspath.cs(2),dubinspath.ps(1)-dubinspath.cs(1)),2*pi);
  th2 = mod(atan2(dubinspath.w1(2)-dubinspath.cs(2),dubinspath.w1(1)-dubinspath.cs(1)),2*pi);
  if dubinspath.lams>0,
      if th1>=th2,
        th = [th1:Del:2*pi,0:Del:th2];
      else
        th = [th1:Del:th2];
      end
  else
      if th1<=th2,
        th = [th1:-Del:0,2*pi:-Del:th2];
      else
        th = [th1:-Del:th2];
      end
  end
  X = [];
  Y = [];
  for i=1:length(th),
    X = [X; dubinspath.cs(1)+dubinspath.R*cos(th(i))]; 
    Y = [Y; dubinspath.cs(2)+dubinspath.R*sin(th(i))];
  end
  
  % points along straight line 
  sig = 0;
  while sig<=1,
      X = [X; (1-sig)*dubinspath.w1(1) + sig*dubinspath.w2(1)];
      Y = [Y; (1-sig)*dubinspath.w1(2) + sig*dubinspath.w2(2)];
      sig = sig + Del;
  end
    
  % points along end circle
  th2 = mod(atan2(dubinspath.pe(2)-dubinspath.ce(2),dubinspath.pe(1)-dubinspath.ce(1)),2*pi);
  th1 = mod(atan2(dubinspath.w2(2)-dubinspath.ce(2),dubinspath.w2(1)-dubinspath.ce(1)),2*pi);
  if dubinspath.lame>0,
      if th1>=th2,
        th = [th1:Del:2*pi,0:Del:th2];
      else
        th = [th1:Del:th2];
      end
  else
      if th1<=th2,
        th = [th1:-Del:0,2*pi:-Del:th2];
      else
        th = [th1:-Del:th2];
      end
  end
  for i=1:length(th),
    X = [X; dubinspath.ce(1)+dubinspath.R*cos(th(i))]; 
    Y = [Y; dubinspath.ce(2)+dubinspath.R*sin(th(i))];
  end
  
end


  