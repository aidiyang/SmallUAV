% RRTstar_script

% define constants
CONST_BUILD_HEIGHT = 1; % 1 forces all buildings to specified height
NUM_ITERATIONS = 1000;

city_width = 2000;
building_height = 300;
num_blocks = 5;
street_width = 0.7;
altitude = 200;
segmentLength = 200;
radius = (segmentLength+1); % ball radius around a new point to look at for rewiring

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
path = planRRTstar(wp_start, wp_end, map, segmentLength, radius, NUM_ITERATIONS);

