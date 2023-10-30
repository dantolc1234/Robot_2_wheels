clear;
clc;

% map setting
map_size = [46, 42];
% obstacle
obs0 = [1:map_size(1), ... 
        map_size(1):map_size(1):map_size(1) * map_size(2), ...
        1:map_size(1):(map_size(1) - 1) * map_size(2), ...
        map_size(1) * (map_size(2) - 1):map_size(1) * map_size(2)];
% real map have 21 column and 23 rows => 1 square = 2 units
% position = y +(x - 1) * m (with m = the number of rows of the map, not the order)
% Scale map = 2 with mapsize of vrep = 21,23; mapsize of matlab = 42,46

obs1 = [1354:1:1358,1400:1:1404,1446:1:1450,1492:1:1496];

obs2 = [430:1:433,476:1:479,384:1:387];

obs3 = [884:1:886,930:1:932,976:1:978];

obs4 = [904:1:906,950:1:952,996:1:998];

obs5 = [222:1:226,268:1:272,314:1:318,360:1:364,406:1:410,452:1:456,498:1:502];

obstacle = [obs0,obs1,obs2,obs3,obs4,obs5];

% creat grid
clf;
hold on;
grid_map = generate_grid(map_size, obstacle);

% save and plot
save map_46x42 grid_map
plot_grid(grid_map);

start = [3,3];
goal = [38,45];

[path, ~] = A_star(grid_map, start, goal);
grid_with_path = grid_map + path;
plot_grid(grid_with_path);
%%
function grid_map = generate_grid(size, obstacle)
%%
    grid_map = ones(size(1), size(2));
    grid_map(obstacle) = 2;
end
%%
function plot_grid(grid_map)
    cmap = [1 1 1; ...        
            0 0 0];
    colormap(cmap);
    %%
    [rows, cols] = size(grid_map);
    image(1.5, 1.5, grid_map);
    grid on
    set(gca,'xtick', 1:cols, 'ytick', 1:rows);
    axis image;
    
    for row = 1:rows
        line([1, cols + 1], [row, row], 'Color','black');
    end
    for col = 1:cols
        line([col, col], [1, rows + 1], 'Color','black');
    end
end
%%
function [path, cost] = A_star(gridmap, start, goal)

% Set up thuat tuan A*
[row_size, col_size] = size(gridmap); % get size of the map

open_list = zeros(row_size,col_size); % Danh sach dang cho duyet
closed_list = zeros(row_size,col_size); % Danh sach da duyet
back_track = zeros(row_size,col_size,2); % Mang cha

g_score = inf(row_size,col_size); % Gia tri G cua cac o
g_score(start(1), start(2)) = 0;    % Gia tri o bat dau = 0
f_score = inf(row_size,col_size); % Gia tri F cua cac o
f_score(start(1), start(2)) = heuristic(start, goal); 

%% 
open_list(start(1), start(2)) = 1;
while sum(open_list(:)) > 0
    current = min_Fscore(open_list, f_score);
    if isequal(current, goal)
        path = finished_path(back_track, goal);
        cost = g_score(goal(1), goal(2));
        return; 
    end
    open_list(current(1), current(2)) = 0;
    closed_list(current(1), current(2)) = 1;
    neighbors = get_neighbors(current, gridmap);
    for i = 1:length(neighbors)
        neighbor = neighbors(i,:);
        if closed_list(neighbor(1), neighbor(2)) == 1
            continue;
        end
        pre_g_score = g_score(current(1), current(2)) + cost_fee(current, neighbor);
        if open_list(neighbor(1), neighbor(2)) == 0 || pre_g_score < g_score(neighbor(1), neighbor(2)) 
            back_track(neighbor(1), neighbor(2), :) = current;
            g_score(neighbor(1), neighbor(2)) = pre_g_score;  
            f_score(neighbor(1), neighbor(2)) = pre_g_score + heuristic(neighbor, goal);
            open_list(neighbor(1), neighbor(2)) = 1;
        end
    end
end
return;
end
%% Ham chi phi
function cost = cost_fee(block, nexBlock)
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
                continue;   % Ko dem node hien tai
            end
            if current_Pos(1)+i < 1 || current_Pos(1)+i > row_size || current_Pos(2)+j < 1 || current_Pos(2)+j > col_size
                continue;   % Ko di qua rao
            end
            if gridmap(current_Pos(1)+i, current_Pos(2)+j) == 2
                continue;   % Ko dem khi co vat can
            end
            count = count + 1;
            neighbors(count,:) = [current_Pos(1)+i, current_Pos(2)+j];
        end
    end
    neighbors = neighbors(1:count,:);  
end
%% Tinh Heuristic
function h = heuristic(node, goal)
    h = abs(node(1)-goal(1)) + abs(node(2)-goal(2));
end
%% Ham tra lai cac node nho nhat
function node = min_Fscore(open_list, f_score)
    [min_f_score, ~] = min(f_score(open_list == 1));
    [i, j] = ind2sub(size(f_score), find(f_score == min_f_score & open_list == 1));
    node = [i(1), j(1)];
end
%% Hoan thien path
function path = finished_path(back_track, current)
    [row_size, col_size, ~] = size(back_track);
    path = zeros(row_size, col_size);
    while current(1) > 0 || current(2) > 0
        path(current(1), current(2)) = 1;
        current = back_track(current(1), current(2), :);
        current = reshape(current, 1, []);
    end
end