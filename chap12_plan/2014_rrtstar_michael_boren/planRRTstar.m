function path_out = planRRTstar(wp_start, wp_end, map, segmentLength, radius, iterations)

    % desired down position is down position of end node
    pd = wp_end(3);
    chi = -9999; % straight line paths
    
    % specify start and end nodes from wpp_start and wpp_end
    start_node = [wp_start(1), wp_start(2), pd, chi, 0, 0, 0];
    end_node = [wp_end(1), wp_end(2), pd, chi 0, 0, 0];
    % format: [N, E, D, chi, cost, parent_idx, flag_connect_to_goal]
    
    % initialize tree
    tree = start_node;
    
    % check to see if start_node connects directly to end_node
    if ((norm(start_node(1:3)-end_node(1:3))<segmentLength)...
            && (collision(start_node,end_node,map)==0)),
        path = [start_node; end_node];
    else
        tic; % start timer
        flag = 0;
        while flag < 1,
%         for count = 1:(iterations-1),
            % add new node to tree
            [tree,flag] = extendTree(tree,end_node,segmentLength,radius,map,pd,chi);
        end
        path = findMinimumPath(tree,end_node);
        path_out = smoothPath(path,map);
        time_path = toc;
    end
    
    plotmap(map,tree,path);
    disp(['time to min path:  ', num2str(time_path), ' sec']);
end


%% PATH PLANNING FUNCTIONS
% add new node to tree
function [new_tree,flag] = extendTree(tree,end_node,segmentLength,radius,map,pd,chi)

    flag1 = 0;
    while flag1 == 0,
        % generate a random node
        randomNode = generateRandomNode(map,pd,chi);
        
        % create new node acording to steering function
        nearest_node = nearest(randomNode,tree);
        new_node = steer(randomNode,nearest_node,segmentLength);
        
        if collision(new_node,nearest_node,map) == 0,
            % find nodes within radius
            distances = ones(size(tree,1),1)*new_node(1:3) - tree(:,1:3);
            distances = sqrt(diag(distances*distances'));
            near_nodes = find(distances<radius);
            %choose parent
            [p_cost,idx] = min(tree(near_nodes,5));
            new_node(6) = near_nodes(idx); % set parent
            new_node(5) = distances(near_nodes(idx)) + p_cost; % set cost
            % DO WE NEED A COLLISION CHECK HERE?
            new_tree = [tree; new_node];
            flag1 = 1;
            % rewire tree
            for i = 1:length(near_nodes),
                index = near_nodes(i);
                ell = distances(index);
                if tree(index,5) > (new_node(5)+ell),
                    new_tree(index,6) = size(new_tree,1);
                end
            end
        end
    end
    
    % check to see if new node connects directly to end_node
    if ((norm(new_node(1:3)-end_node(1:3)) < segmentLength)...
            && (collision(new_node,end_node,map)==0)),
        flag = 1;
        new_tree(end,7)=1; % mark node as connecting to end
    else
        flag = 0;
    end
    
end

% generate random node
function node = generateRandomNode(map,pd,chi)

    % choose random configuration
    pn = map.width*rand;
    pe = map.width*rand;
    pd = pd;
    cost = 0;
    if abs(chi) < 1000, % set random heading for dubins path
        chi = 2*pi*rand;
    end
    node = [pn, pe, pd, chi, cost, 0, 0];
    % format: {N, E, D, chi, cost, parent_idx, flag_connect_to_goal]
    
end

% find nearest node to a given node
function nearest_node = nearest(node,tree)

    distances = tree(:,1:3) - ones(size(tree,1),1)*node(1:3);
    [~,idx] = min(diag(distances*distances'));
    
    nearest_node = tree(idx,:);
end

% steering function
function newnode = steer(randomNode,nearestNode,segmentLength)
    
    % determine distance and direction from nearest node to random node
    diff = randomNode(1:3) - nearestNode(1:3);
    ell = min(sqrt(diff*diff'), segmentLength);
    q = diff/norm(diff);
   
    % define new node
    new_point = nearestNode(1:3) + ell*q;
    cost = 0;
    chi = randomNode(4);
    newnode = [new_point, chi, cost, 0, 0];
    
end

% find minimum path
function path = findMinimumPath(tree,end_node)

    % find nodes that connect to end_node
    connectingNodes = [];
    for i = 1:size(tree,1),
        if tree(i,7) == 1,
            connectingNodes = [connectingNodes; tree(i,:)];
        end
    end
    
    % find parent for end_node
    [cost,idx] = min(connectingNodes(:,5));
    ell = end_node(1:3) - connectingNodes(idx,1:3);
    cost = cost + sqrt(ell*ell');
    disp(['min path cost: ', num2str(cost)]);
    
    % construct path
    path = [connectingNodes(idx,:); end_node];
    parent_node = connectingNodes(idx,6);
    while parent_node > 1,
        parent_node = tree(parent_node,6); % switch these statements?
        path = [tree(parent_node,:); path];
    end

end

% smooth path
function newPath = smoothPath(path,map)

    newPath = path(1,:); % add the start node 
    ptr =2;  % pointer into the path
    while ptr <= size(path,1)-1,
        if collision(newPath(end,:), path(ptr+1,:), map)~=0, % if there is a collision
            newPath = [newPath; path(ptr,:)];  % add previous node
        end
        ptr=ptr+1;
    end
    newPath = [newPath; path(end,:)];
    
    % find minimum cost last node
    cost = zeros(1,size(newPath,1)-1);
    for n=2:size(newPath,1),
        cost(n-1) = norm([newPath(n,1)-newPath(n-1,1), newPath(n,2)-newPath(n-1,2)]);
    end
    cost = sum(cost);
    disp(['min path cost: ', num2str(cost)]);

end

%% COLLISION DETECTION FUNCTIONS
% detect collision
function collision_flag = collision(start_node, end_node, map)

    collision_flag = 0;
    
    [X,Y,Z] = pointsAlongPath(start_node, end_node, 0.1);
    
    for i = 1:length(X),
        if Z(i) >= downAtNE(map, X(i), Y(i)),
            collision_flag = 1;
        end
    end
    
end


% return points along path between two nodes
function [X,Y,Z] = pointsAlongPath(start_node, end_node, Del)

    X = [start_node(1)];
    Y = [start_node(2)];
    Z = [start_node(3)];
    
    q = [end_node(1:3)-start_node(1:3)];
    L = norm(q);
    q = q/L;
    
    w = start_node(1:3);
    for i=2:floor(L/Del),
        w = w + Del*q;
        X = [X, w(1)];
        Y = [Y, w(2)];
        Z = [Z, w(3)];
    end 
    
end

% return down value (-altitude) at given map coordinates
function down = downAtNE(map, n, e)

    % find north and east distances to closest building centers
    [d_n,idx_n] = min(abs(n - map.buildings_n));
    [d_e,idx_e] = min(abs(e - map.buildings_e));
    
    % return building height if within building size of center, else 0
    if (d_n<=map.BuildingWidth) && (d_e<=map.BuildingWidth),
        down = -map.heights(idx_e,idx_n);
    else
        down = 0;
    end
    
end

%% OUTPUT FUNCTIONS
% plot map
function plotmap(map,tree,path)

    % setup plot
    figure(3), clf
    axis([0,map.width,0,map.width,0,2*map.MaxHeight]);
    view([0,90]);
    xlabel('E')
    ylabel('N')
    zlabel('h')
    hold on
  
    % plot buildings 
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
   
    % draw tree
    for i=2:size(tree,1),
        X = [tree(i,1), tree(tree(i,6),1)];
        Y = [tree(i,2), tree(tree(i,6),2)];   
        Z = [tree(i,3), tree(tree(i,6),3)];            
        plot3(Y,X,-Z,'g')
    end
    
    % draw minimum path
    X = path(:,1);
    Y = path(:,2);
    Z = path(:,3);
    plot3(Y,X,-Z,'r','linewidth',2);
%     terminal_nodes = find(tree(:,7)>0);
%     if isempty(terminal_nodes) == 0,
%         for index = 1:length(terminal_nodes),
%             current = tree(terminal_nodes(index),1:3);
%             parent = tree(terminal_nodes(index),6);
%             % plot connection to end_node
%             X = [end_node(1), current(1)];
%             Y = [end_node(2), current(2)];
%             Z = [end_node(3), current(3)];
%             plot3(Y,X,-Z,'r','linewidth',2);
%             % plot intermediate connections
%             while parent > 1,
%                 X = [current(1), tree(parent,1)];
%                 Y = [current(2), tree(parent,2)];   
%                 Z = [current(3), tree(parent,3)];            
%                 plot3(Y,X,-Z,'r','linewidth',2)
%                 current = tree(parent,1:3);
%                 parent = tree(parent,6);
%             end
%             % plot connection to start_node
%             X = [current(1), start_node(1)];
%             Y = [current(2), start_node(2)];   
%             Z = [current(3), start_node(3)];            
%             plot3(Y,X,-Z,'r','linewidth',2)
%         end
%     end
                

end

% path smoothing
% function newPath = smoothPath(path,map)
% 
%     newPath = path(1,:); % add the start node 
%     ptr =2;  % pointer into the path
%     while ptr <= size(path,1)-1,
%         if collision(newPath(end,:), path(ptr+1,:), map)~=0, % if there is a collision
%             newPath = [newPath; path(ptr,:)];  % add previous node
%         end
%         ptr=ptr+1;
%     end
%     newPath = [newPath; path(end,:)];
% 
% end

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