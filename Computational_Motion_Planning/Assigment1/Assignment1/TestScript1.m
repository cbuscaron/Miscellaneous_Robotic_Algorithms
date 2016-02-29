%
% TestScript for Assignment 1
%

%% Define a small map
map = false(10*2)

% Add an obstacle
map (1:5*2, 6*2) = true;
map (1:5*2, 6*2-1) = true;
map (1:5*2, 6*2+1) = true;

map (14:20, 6) = true;
map (14:20, 6-1) = true;

map (17:20, 12) = true;
map (17:20, 12-1) = true;

start_coords = [12, 4];
start_coords = [19, 2];
dest_coords  = [8*2, 9*2];

%map

%%
close all;
[route, numExpanded] = DijkstraGrid (map, start_coords, dest_coords)
% Uncomment following line to run Astar
% [route, numExpanded] = AStarGrid (map, start_coords, dest_coords);
