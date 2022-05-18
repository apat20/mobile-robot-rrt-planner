% This function is used to generate samples in the environment by dividing
% the entire space into a discrete number of regions. We sample a fixed
% number of points from each region. The environment is discretized into a
% fixed number of regions along the X axis i.e the width W

%% INPUT:
% total_poses: Total number of poses to be sampled

% num_regions: Number of regions 

% W: width of the environment along the X Axis

% H: width of the environment along the Y Axis

%% OUTPUT:

% sampled_poses: A multidimensional array with the sampled poses.
% Dimension: 2 x (points per region) x (number of regions)

function[sampled_poses] = generate_samples(total_poses, num_regions, W, H)
    % Step size based on the enumber of regions:
    step_x = W/num_regions;
    x_lims = 1:step_x:W;
    
    % Number of poses per region:
    num_poses_per_region = total_poses/num_regions;
    
    % Generating the coordinates along the X direction i.e the width of the
    % environment:
    for i = 1:size(x_lims, 2)
        if i < size(x_lims, 2)
            x(:, :, i)  = (x_lims(:,i+1) -x_lims(:,i)).*rand(num_poses_per_region,1) + x_lims(:,i);
        elseif i == size(x_lims, 2)
            x(:, :, i) = (W -x_lims(:,i)).*rand(num_poses_per_region,1) + x_lims(:,i);
        else
            fprintf('Incorrect Computation');
        end
    end
    
    % Generating the coordinates along the Y direction i.e the height of the
    % environment:
    for i = 1:num_regions
        y(:,:,i) = rand(1, num_poses_per_region)*H;
    end
    
    y = reshape(y, size(x));
    
    % Combining the x and y coordinates to get the final pose.      
    for i = 1:num_regions
        for j = 1:num_poses_per_region
            sampled_poses(:,j,i) = [round(x(j,:,i), 2), round(y(j,:,i),2)].';
        end
    end

end