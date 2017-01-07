% RRT Script

% define constants
REAL_TIME_PLOT = 0;
CONST_BUILD_HEIGHT = 1;
NUM_ITERATIONS = 1000;

city_width = 2000;
building_height = 300;
num_blocks = 5;
street_width = 0.7;
altitude = 200;
segmentLength = 200;

wp_start = [0; 0; altitude];
wp_end = [city_width; city_width; -altitude];


% generate terrain
if CONST_BUILD_HEIGHT == 1,
    flag = 1;
else
    flag = 0;
end
map = createWorld(city_width, building_height, num_blocks, street_width, flag);


% calculate path
if REAL_TIME_PLOT == 1,
    % display plot of path tree in real time as it develops
    path = planRRT_RT(wp_start, wp_end, map, segmentLength, NUM_ITERATIONS);
else
    % calculate path and display plot along with calculation time
    path = planRRT(wp_start, wp_end, map, segmentLength, NUM_ITERATIONS);
end