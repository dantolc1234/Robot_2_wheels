clear;
clc;

% map setting
map_size = [46, 42];
% obstacle
obs0 = [1:map_size(1), ... 
        map_size(1):map_size(1):map_size(1) * map_size(2), ...
        1:map_size(1):(map_size(1) - 1) * map_size(2), ...
        map_size(1) * (map_size(2) - 1):map_size(1) * map_size(2)];

obs1 = [903:1:908,...
        949:1:954,...
        995:1:1000,...
        1041:1:1046,...
        1087:1:1092,...
        1133:1:1138];
    
obs2 = [423:1:436,...
        469:1:482,...
        515:1:528,...
        561:1:574,...
        607:1:620,...
        653:1:666,...
        699:1:712,...
        745:1:758];
        
obstacle = [obs0, obs1,obs2];

% creat grid
clf;
hold on;
grid_map = generate_grid(map_size, obstacle);

% save and plot
save gridmap_46x42_scene1 grid_map
plot_grid(grid_map);

function grid_map = generate_grid(size, obstacle)
%%
    grid_map = ones(size(1), size(2));
    grid_map(obstacle) = 2;
end

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
        line([1, cols + 1], [row, row], 'Color','#4DBEEE');
    end
    for col = 1:cols
        line([col, col], [1, rows + 1], 'Color','#4DBEEE');
    end
end