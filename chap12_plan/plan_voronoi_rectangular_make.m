function plan_voronoi_rectangular_make;

% generate random rectangular obstacle field
VG.num_obstacles = 5; % number of rectangular objects
VG.field_size = 10;
VG.B = generateRectangularField(VG.num_obstacles,VG.field_size);

% plot obstacles
figure(1), clf
  hold on
  xlabel('East (km)')
  ylabel('North (km)')
  eps = .1*VG.field_size;
  axis([-eps,VG.field_size+eps,-eps,VG.field_size+eps])
  for i=1:VG.num_obstacles,
    fill(VG.B(:,1,i),VG.B(:,2,i),0.8*[1,1,1])
  end
 %print -depsc plan-voronoi-rectangular1.eps
 

%generate Voronoi points along edges of obstacles
VG = generatePointsOnObstacles(VG);

% add random points
%VG = addRandomPoints(VG, 10);

% add start and end nodes not in the obstacles
VG = addStartEndNodes(VG);
VG.start = [0, 10];
VG.end   = [10, 0];

% plot obstacles with points
figure(2), clf
  hold on
  xlabel('East (km)')
  ylabel('North (km)')
  eps = .1*VG.field_size;
  axis([-eps,VG.field_size+eps,-eps,VG.field_size+eps])
  for i=1:VG.num_obstacles,
    fill(VG.B(:,1,i),VG.B(:,2,i),0.8*[1,1,1])
  end
  % plot points 
  %plot(VG.P(:,1),VG.P(:,2),'.k')
  for i=1:size(VG.P,1),
    drawcircle(VG.P(i,1),VG.P(i,2),.07,'k');
  end
  % plot start node
  plot(VG.start(1), VG.start(2),'ok','MarkerSize',10)
  % plot end node
  plot(VG.end(1), VG.end(2),'sk','MarkerSize',10)
 %print -depsc plan-voronoi-rectangular2.eps


% create Voronoi graph
VG = createVoronoiGraphFull(VG);

% connect start and end nodes to graph 
VG = addClosestEdges(VG);

% plot voronoi diagram
figure(3), clf
  hold on
  eps = .1*VG.field_size;
  axis([-eps,VG.field_size+eps,-eps,VG.field_size+eps])
  for i=1:VG.num_obstacles,
    fill(VG.B(:,1,i),VG.B(:,2,i),0.8*[1,1,1])
  end
  
  % plot edges
  plot(VG.edges_x,VG.edges_y,'-k')
  %hold on
  % plot all but start and end 
  %plot(VG.P(:,1),VG.P(:,2),'.')
  for i=1:size(VG.P,1),
    drawcircle(VG.P(i,1),VG.P(i,2),.07,'k');
  end
  % plot start node
  plot(VG.start(1), VG.start(2),'ok','MarkerSize',10)
  % plot end node
  plot(VG.end(1), VG.end(2),'sk','MarkerSize',10)
  % set axis properties and labels
 % axis(10*[-1,2,-1,2])
  xlabel('East (km)')
  ylabel('North (km)')
  
   %print -depsc plan-voronoi-rectangular3.eps
  
% create Voronoi graph
VG = createVoronoiGraph(VG);

% connect start and end nodes to graph 
VG = addClosestEdges(VG);


% assign costs to edges
VG = assignCosts(VG);

% find the minimum cost path
VG = findMinimumCostPath(VG);

% plot voronoi diagram
figure(4), clf
  hold on
  eps = .1*VG.field_size;
  axis([-eps,VG.field_size+eps,-eps,VG.field_size+eps])
  for i=1:VG.num_obstacles,
    fill(VG.B(:,1,i),VG.B(:,2,i),0.8*[1,1,1])
  end
  
  % plot edges
  plot(VG.edges_x,VG.edges_y,'--k')
  %hold on
  % plot all but start and end 
  %plot(VG.P(:,1),VG.P(:,2),'.')
  for i=1:size(VG.P,1),
    drawcircle(VG.P(i,1),VG.P(i,2),.07,'k');
  end
  % plot start node
  plot(VG.start(1), VG.start(2),'ok','MarkerSize',10)
  % plot end node
  plot(VG.end(1), VG.end(2),'sk','MarkerSize',10)
  % set axis properties and labels
 % axis(10*[-1,2,-1,2])
  xlabel('East (km)')
  ylabel('North (km)')
  % plot minimum path
  path_x = [];
  path_y = [];
  for i=1:length(VG.min_path)-1,
      path_x = [path_x, [VG.nodes(VG.min_path(i),1); VG.nodes(VG.min_path(i+1),1)]];
      path_y = [path_y, [VG.nodes(VG.min_path(i),2); VG.nodes(VG.min_path(i+1),2)]];
  end
  plot(path_x,path_y,'-k','linewidth',2)
  
  %print -depsc plan-voronoi-rectangular4.eps


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% draw circle
function drawcircle(x,y,size,color)

    phi = 0:.1:2*pi;
    X = x+size*cos(phi);
    Y = y+size*sin(phi);
    fill(X,Y,color)

  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% add random points
function VG = addRandomPoints(VG, num_points)
    
    i = 0;
    while i < num_points,

        tmp = VG.field_size*[rand, rand];
        if inObstacle(tmp,VG)~=1,
            VG.P = [VG.P; tmp];
            i = i+1;
        end
        
        
    end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% add start and end nodes not in the obstacles
function VG = addStartEndNodes(VG)
    flag = 0;
    while flag==0,
        VG.start = VG.field_size*[rand, rand];  
        if inObstacle(VG.start,VG)~=1,
            flag = 1;
        end
    end
    flag = 0;
    while flag==0,
        VG.end = VG.field_size*[rand, rand];  
        if inObstacle(VG.end,VG)~=1,
            flag = 1;
        end
    end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% check to see if point in inside a rectagular obstacle
function flag = inObstacle(p,VG)
  flag = 0;
  for i=1:VG.num_obstacles,
      n1 = VG.B(2,:,i)-VG.B(1,:,i);
      n2 = VG.B(3,:,i)-VG.B(2,:,i);
      n3 = VG.B(4,:,i)-VG.B(3,:,i);
      n4 = VG.B(1,:,i)-VG.B(4,:,i);
      if ((p-VG.B(2,:,i))*n1'<=0)...
              & ((p-VG.B(3,:,i))*n2'<=0)...
                & ((p-VG.B(4,:,i))*n3'<=0)...
                    & ((p-VG.B(1,:,i))*n4'<=0),
          flag = 1;
      end
  end

            
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% generate Voronoi points along edges of obstacles
function VG = generatePointsOnObstacles(VG)
    VG.P = [];
    for i=1:VG.num_obstacles,
        if norm(VG.B(1,:,i)-VG.B(2,:,i))>VG.field_size/20,
            VG.P = [VG.P;...
                VG.B(:,:,i);...
                (VG.B(1,:,i)+VG.B(2,:,i))/2;...
                (VG.B(2,:,i)+VG.B(3,:,i))/2;...
                (VG.B(3,:,i)+VG.B(4,:,i))/2;...
                (VG.B(4,:,i)+VG.B(1,:,i))/2;...
                ];
        else
            VG.P = [VG.P;...
                (VG.B(1,:,i)+VG.B(2,:,i)+VG.B(3,:,i)+VG.B(4,:,i))/4;...
                ];
        end
    end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% generate random rectangular field
function F = generateRectangularField(N,field_size)

    for i=1:N,
        p = field_size*[rand, rand];
        dir = 2*pi*rand;
        building_size = rand*field_size/N;
        F(:,:,i) = [...
            p + [building_size/2, building_size/2]*rot(dir)';...
            p + [building_size/2, -building_size/2]*rot(dir)';...
            p + [-building_size/2, -building_size/2]*rot(dir)';...
            p + [-building_size/2, building_size/2]*rot(dir)';...
            ];
    end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function R = rot(theta)
    R = [...
        cos(theta), sin(theta);...
        -sin(theta), cos(theta);...
        ];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function VG = findMinimumCostPath(VG)
  for i = 1:VG.num_nodes,
     % initialize the farthest node to be itself;
     farthestPreviousHop(i) = i;     % used to compute the RTS/CTS range;
     farthestNextHop(i) = i;
  end;
  [path, totalCost, farthestPreviousHop, farthestNextHop] = dijkstra(VG.num_nodes, VG.cost_matrix, 1, 2, farthestPreviousHop, farthestNextHop);
  VG.min_path = path;
  VG.min_cost = totalCost;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function VG = assignCosts(VG)

  % initialize cost matrix
  num_nodes = size(VG.nodes,1);
  VG.cost_matrix = inf*ones(num_nodes,num_nodes);
  
  % cycle through each node
  for i=1:VG.num_nodes,
      for j=(i+1):VG.num_nodes,
          if VG.adjacency_matrix(i,j)>0,
              n1 = VG.nodes(i,:)';
              n2 = VG.nodes(j,:)';
              % for each point p, find minimum distance from edge to p
              for k=1:size(VG.P,1),
                  p = VG.P(k,:)';
                  dist = norm(p-n1)^2 - (n1-p)'*(n1-n2)/norm(n1-n2)^2;
              end
              % add reciprical of minimum length
              gam = 0.1;
              VG.cost_matrix(i,j) = gam*norm(n1-n2) + (1-gam)*(1/min(dist));
              VG.cost_matrix(j,i) = VG.cost_matrix(i,j);
          end
      end
  end
 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function VG = addClosestEdges(VG)
  
  [closest_nodes_start,idx_start] = findClosestNodes(VG.start, VG.nodes);
  [closest_nodes_end,idx_end] = findClosestNodes(VG.end, VG.nodes);

  % construct new edges
  new_edges_x = [[closest_nodes_start(:,1),[1;1;1]*VG.start(1)]',...
                 [closest_nodes_end(:,1),[1;1;1]*VG.end(1)]'];
  new_edges_y = [[closest_nodes_start(:,2),[1;1;1]*VG.start(2)]',...
                 [closest_nodes_end(:,2),[1;1;1]*VG.end(2)]'];

  VG.edges_x = [VG.edges_x, new_edges_x];
  VG.edges_y = [VG.edges_y, new_edges_y];
  
  % adjust the adjacency matrix
  for j=1:3,
      VG.adjacency_matrix(1,idx_start(j)) = 1;
      VG.adjacency_matrix(idx_start(j),1) = 1;
      VG.adjacency_matrix(2,idx_end(j)) = 1;
      VG.adjacency_matrix(idx_end(j),2) = 1;
  end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function VG = createVoronoiGraph(VG)
  P = [VG.start; VG.end; VG.P];
  [edges_x_temp, edges_y_temp] = voronoi(P(:,1), P(:,2));
  VG.edges_x = [];
  VG.edges_y = [];
  edges_x_temp2 = [];
  edges_y_temp2 = [];
  % remove edges in obstacles
  for i = 1:size(edges_x_temp,2)
    if (inObstacle([edges_x_temp(1,i),edges_y_temp(1,i)],VG)==0)...
            & (inObstacle([edges_x_temp(2,i),edges_y_temp(2,i)],VG)==0),
        edges_x_temp2 = [edges_x_temp2, edges_x_temp(:,i)];
        edges_y_temp2 = [edges_y_temp2, edges_y_temp(:,i)];
    end
  end
%   % remove stranded links
%   N = size(edges_y_temp2,2);
%   for i=1:N,
%      flag1 = 0;
%      flag2 = 0;
%      for j=[1:i-1,i+1:N],
%           if (abs(edges_x_temp2(1,i)-edges_x_temp2(1,j))<.01...
%               & abs(edges_y_temp2(1,i)-edges_y_temp2(1,j))<.01)...
%                 |(abs(edges_x_temp2(1,i)-edges_x_temp2(2,j))<.01...
%                     & abs(edges_y_temp2(1,i)-edges_y_temp2(2,j))<.01),
%             flag1 = 1;
%           end
%           if (abs(edges_x_temp2(2,i)-edges_x_temp2(2,j))<.01...
%                 & abs(edges_y_temp2(2,i)-edges_y_temp2(2,j))<.01)...
%                    | (abs(edges_x_temp2(2,i)-edges_x_temp2(1,j))<.01...
%                         & abs(edges_y_temp2(2,i)-edges_y_temp2(1,j))<.01),
%             flag2 = 1;
%           end       
%      end
%      if flag1==1 & flag2==1,
%         VG.edges_x = [VG.edges_x, edges_x_temp2(:,i)]; 
%         VG.edges_y = [VG.edges_y, edges_y_temp2(:,i)]; 
%      end
%   end
%   
  VG.edges_x = edges_x_temp2;
  VG.edges_y = edges_y_temp2;
  
  VG = generateNodes(VG);
  VG.num_nodes = size(VG.nodes,1);
  VG.num_edges = size(VG.edges_x,2);
  VG.adjacency_matrix = generateAdjacencyMatrix(VG);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function VG = createVoronoiGraphFull(VG)
  P = [VG.start; VG.end; VG.P];
  [edges_x_temp, edges_y_temp] = voronoi(P(:,1), P(:,2));
  VG.edges_x = edges_x_temp;
  VG.edges_y = edges_y_temp;
  
  VG = generateNodes(VG);
  VG.num_nodes = size(VG.nodes,1);
  VG.num_edges = size(VG.edges_x,2);
  VG.adjacency_matrix = generateAdjacencyMatrix(VG);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function adjacency_matrix = generateAdjacencyMatrix(VG)
  adjacency_matrix = zeros(VG.num_nodes, VG.num_nodes);
  for i=1:VG.num_edges,
      n1 = [VG.edges_x(1,i); VG.edges_y(1,i)];
      n2 = [VG.edges_x(2,i); VG.edges_y(2,i)];
      flag_found_n1 = 0;
      flag_found_n2 = 0;
      j = 1;
      while ((flag_found_n1==0) | (flag_found_n2==0))&(j<=VG.num_nodes),
          if norm(n1-VG.nodes(j,:)')<.01,
              idx_n1 = j;
              flag_found_n1 = 1;
          elseif norm(n2-VG.nodes(j,:)')<.01,
              idx_n2 = j;
              flag_found_n2 = 1;
          end
          j=j+1;
      end
      adjacency_matrix(idx_n1,idx_n2) = 1;
      adjacency_matrix(idx_n2,idx_n1) = 1;
  end

  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function VG=generateNodes(VG)

  % create temporary list of nodes
  nodes_ = [VG.edges_x(1,:)', VG.edges_y(1,:)'; VG.edges_x(2,:)', VG.edges_y(2,:)'];

  % remove redundant nodes
  nodes = nodes_(1,:);
  num_nodes = 1;
  for i=2:size(nodes_,1),
      flag = 0;
      for j=1:num_nodes,
          if norm(nodes_(i,:)-nodes(j,:))<0.001,
              flag = 1;
          end
      end
      if flag==0,
        nodes = [nodes; nodes_(i,:)];
        num_nodes = num_nodes+1;
      end
  end
  
  % add start and end nodes
  VG.nodes = [VG.start; VG.end; nodes];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% return the three closest nodes
function [closest_nodes,idx_closest] = findClosestNodes(v_start, nodes)

  % compute distance to nodes
  tmp = ones(size(nodes,1),1)*v_start-nodes;
  dist = sqrt(diag(tmp*tmp'));
  % sort the nodes by distance
  [tmp2, idx] = sort(dist);
  % return the three closest nodes (assumes v_start is one of the nodes and
  % so it deletes the closest node which is itself)
  closest_nodes = [nodes(idx(2),:); nodes(idx(3),:); nodes(idx(4),:)];
  idx_closest = idx(2:4);
  
  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% plot voronoi diagram
function plotVoronoiGraph(VG);
  figure(1), clf
  hold on
  eps = .1*VG.field_size;
  axis([-eps,VG.field_size+eps,-eps,VG.field_size+eps])
  for i=1:VG.num_obstacles,
    fill(VG.B(:,1,i),VG.B(:,2,i),'g')
  end
  
  % plot edges
  plot(VG.edges_x,VG.edges_y,'-b')
  %hold on
  % plot all but start and end 
  plot(VG.P(:,1),VG.P(:,2),'.')
  % plot start node
  plot(VG.start(1), VG.start(2),'or')
  % plot end node
  plot(VG.end(1), VG.end(2),'sr')
  % set axis properties and labels
 % axis(10*[-1,2,-1,2])
  xlabel('East (km)')
  ylabel('North (km)')
  % plot minimum path
  path_x = [];
  path_y = [];
  for i=1:length(VG.min_path)-1,
      path_x = [path_x, [VG.nodes(VG.min_path(i),1); VG.nodes(VG.min_path(i+1),1)]];
      path_y = [path_y, [VG.nodes(VG.min_path(i),2); VG.nodes(VG.min_path(i+1),2)]];
  end
  plot(path_x,path_y,'-k','linewidth',2)
  
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Dijkstra's algorithm.  Obtained from MatlabCentral  
% function [path, totalCost, farthestPreviousHop, farthestNextHop] = dijkstra(n, netCostMatrix, s, d, farthestPreviousHop, farthestNextHop)
% % path: the list of nodes in the path from source to destination;
% % totalCost: the total cost of the path;
% % farthestNode: the farthest node to reach for each node after performing
% % the routing;
% % n: the number of nodes in the network;
% % s: source node index;
% % d: destination node index;
% 
% 
% % clear;
% % noOfNodes  = 50;
% % rand('state', 0);
% % figure(1);
% % clf;
% % hold on;
% % L = 1000;
% % R = 200; % maximum range;
% % netXloc = rand(1,noOfNodes)*L;
% % netYloc = rand(1,noOfNodes)*L;
% % for i = 1:noOfNodes
% %     plot(netXloc(i), netYloc(i), '.');
% %     text(netXloc(i), netYloc(i), num2str(i));
% %     for j = 1:noOfNodes
% %         distance = sqrt((netXloc(i) - netXloc(j))^2 + (netYloc(i) - netYloc(j))^2);
% %         if distance <= R
% %             matrix(i, j) = 1;   % there is a link;
% %             line([netXloc(i) netXloc(j)], [netYloc(i) netYloc(j)], 'LineStyle', ':');
% %         else
% %             matrix(i, j) = inf;
% %         end;
% %     end;
% % end;
% % 
% % 
% % activeNodes = [];
% % for i = 1:noOfNodes,
% %     % initialize the farthest node to be itself;
% %     farthestPreviousHop(i) = i;     % used to compute the RTS/CTS range;
% %     farthestNextHop(i) = i;
% % end;
% % 
% % [path, totalCost, farthestPreviousHop, farthestNextHop] = dijkstra(noOfNodes, matrix, 1, 15, farthestPreviousHop, farthestNextHop);
% % path
% % totalCost
% % if length(path) ~= 0
% %     for i = 1:(length(path)-1)
% %         line([netXloc(path(i)) netXloc(path(i+1))], [netYloc(path(i)) netYloc(path(i+1))], 'Color','r','LineWidth', 0.50, 'LineStyle', '-.');
% %     end;
% % end;
% % hold off;
% % return;
%     
% 
%     
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % all the nodes are un-visited;
% visited(1:n) = 0;
% 
% distance(1:n) = inf;    % it stores the shortest distance between each node and the source node;
% parent(1:n) = 0;
% 
% distance(s) = 0;
% for i = 1:(n-1),
%     temp = [];
%     for h = 1:n,
%          if visited(h) == 0   % in the tree;
%              temp=[temp distance(h)];
%          else
%              temp=[temp inf];
%          end
%      end;
%      [t, u] = min(temp);    % it starts from node with the shortest distance to the source;
%      visited(u) = 1;       % mark it as visited;
%      for v = 1:n,           % for each neighbors of node u;
%          if ( ( netCostMatrix(u, v) + distance(u)) < distance(v) )
%              distance(v) = distance(u) + netCostMatrix(u, v);   % update the shortest distance when a shorter path is found;
%              parent(v) = u;                                     % update its parent;
%          end;             
%      end;
% end;
% 
% path = [];
% if parent(d) ~= 0   % if there is a path!
%     t = d;
%     path = [d];
%     while t ~= s
%         p = parent(t);
%         path = [p path];
%         
%         if netCostMatrix(t, farthestPreviousHop(t)) < netCostMatrix(t, p)
%             farthestPreviousHop(t) = p;
%         end;
%         if netCostMatrix(p, farthestNextHop(p)) < netCostMatrix(p, t)
%             farthestNextHop(p) = t;
%         end;
% 
%         t = p;      
%     end;
% end;
% 
% totalCost = distance(d);
% 
% return;
