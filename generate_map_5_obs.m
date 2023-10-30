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