% Load the map from file
load('map_46x42.mat', 'map');
start_node = [1, 1];
goal_node = [size(map, 1), size(map, 2)];

% Initialize the open and closed lists
open_list = start_node;
closed_list = [];

% Initialize the cost and heuristic function values
g_score = Inf(size(map));
f_score = Inf(size(map));

g_score(start_node(1), start_node(2)) = 0;
f_score(start_node(1), start_node(2)) = h(start_node, goal_node);

% Initialize the came_from matrix
came_from = zeros(size(map));
%%
while ~isempty(open_list)
    [~, current_index] = min_Fscore(f_score(open_list(:, 1), open_list(:, 2)));
    current_node = open_list(current_index, :);
    % Check if the goal node has been reached
    if isequal(current_node, goal_node)
        % Reconstruct the path from the came_from matrix
        path = reconstruct_path(came_from, start_node, goal_node);
        break;
    end
    % Remove the current node from the open list and add it to the closed list
    open_list(current_index, :) = [];
    closed_list = [closed_list; current_node];
    neighbor_nodes = get_neighbors(current_node, map);
    % Loop through each neighbor node
    for i = 1:size(neighbor_nodes, 1)
        neighbor_node = neighbor_nodes(i, :);
        if ismember(neighbor_node, closed_list, 'rows')
            continue;
        end
        tentative_g_score = g_score(current_node(1), current_node(2)) + distance(current_node, neighbor_node);
        % Check if the neighbor node is not in the open list or if the tentative g-score is lower than the current g-score
        if ~ismember(neighbor_node, open_list, 'rows') || tentative_g_score < g_score(neighbor_node(1), neighbor_node(2))
            % Update the came_from, g_score, and f_score matrices for the neighbor node
            came_from(neighbor_node(1), neighbor_node(2)) = sub2ind(size(map), current_node(1), current_node(2));
            g_score(neighbor_node(1), neighbor_node(2)) = tentative_g_score;
            f_score(neighbor_node(1), neighbor_node(2)) = g_score(neighbor_node(1), neighbor_node(2)) + heuristic(neighbor_node, goal_node);
            % Add the neighbor node to the open list if it is not already there
            if ~ismember(neighbor_node, open_list, 'rows')
                open_list = [open_list; neighbor_node];
            end
        end
    end
end

%% Ham chi phi
function cost = distance(block, nexBlock)
    dx = abs(block(1) - nexBlock(1));
    dy = abs(block(2) - nexBlock(2));
    if dx + dy == 2
        cost = 1.414; 
    else
        cost = 1;
    end
end
%% Ham tim kiem cac o lan can
function neighbors = get_neighbors(current_Pos, gridmap)
    [row_size, col_size] = size(gridmap);
    neighbors = zeros(8,2);
    count = 0;
    for i = -1:1
    for j = -1:1
    if i == 0 && j == 0
        continue;   % Not count the current node
    end
    if current_Pos(1)+i < 1 || current_Pos(1)+i > row_size || current_Pos(2)+j < 1 || current_Pos(2)+j > col_size
        continue;   % Not count the barriers
    end
    if gridmap(current_Pos(1)+i, current_Pos(2)+j) == 2
        continue;   % Not count the obstacles
    end
    count = count + 1;
    neighbors(count,:) = [current_Pos(1)+i, current_Pos(2)+j];
    end
    end
    neighbors = neighbors(1:count,:);  
end
%% Ham tra lai cac node nho nhat
function node = min_Fscore(open_list, f_score)
    [min_f_score, ~] = min(f_score(open_list == 1));
    [i, j] = ind2sub(size(f_score), find(f_score == min_f_score & open_list == 1));
    node = [i(1), j(1)];
end
%% Tinh Heuristic
function h = heuristic(node, goal)
    h = abs(node(1)-goal(1)) + abs(node(2)-goal(2));
end

