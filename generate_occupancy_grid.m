% Function to generate an occupancy grid map of an environment
%
% Input Arguments:
% env               - Structure defining the environment
%
% Output:
% occupancy_grid    - Occupancy grid map for given environment description
function occupancy_grid = generate_occupancy_grid(env)

    mask = zeros(env.size.breadth, env.size.length);

    for itr = 1:length(env.circular_obstacles)
        circular_mask = binaryOccupancyMap(env.size.length, env.size.breadth);
        setOccupancy(circular_mask, env.circular_obstacles{itr}.center.', 1);
        inflate(circular_mask, env.circular_obstacles{itr}.radius);
        mask = mask | occupancyMatrix(circular_mask);
    end
    
    for itr = 1:length(env.polygonal_obstacles)
        poly = env.polygonal_obstacles{itr};
        mask = mask | flip(poly2mask(poly(1,:), poly(2,:),env.size.breadth,env.size.length),1);
    end

    occupancy_grid = binaryOccupancyMap(mask);
end