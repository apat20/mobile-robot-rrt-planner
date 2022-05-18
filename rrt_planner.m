close all;
clear;

environment = 3;

% Set environment occupancy map filename
switch environment
    case 1
        occupancy_grid_filename = 'environment1.csv';
        start_pose = [15;27.5];
        goal_pose = [30;12.5];
    case 2
        occupancy_grid_filename = 'environment2.csv';
        start_pose = [15;27.5];
        goal_pose = [30;12.5];
    case 3
        occupancy_grid_filename = 'environment3.csv';
        start_pose = [2.5;2.5];
        goal_pose = [47.5;27.5];
end

%% Load occupancy grid of environment

occupancy_matrix = readmatrix(occupancy_grid_filename);

occupancy_grid = binaryOccupancyMap(occupancy_matrix);

env_x_range = occupancy_grid.XWorldLimits(2);
env_y_range = occupancy_grid.YWorldLimits(2);

%% Initialize start and goal pose
% start_pose = [3;18];
% goal_pose = [28;12];

%% Visualize environment

figure
show(occupancy_grid)
hold on;

fig_handle = gcf;

% Visualization properties for trees
trees_visualization_prop{1}.node.shape = 'ob';
trees_visualization_prop{1}.node.fill = 'b';
trees_visualization_prop{1}.edge_line_style = '-';
trees_visualization_prop{1}.edge_color = [0.8500 0.3250 0.0980];
trees_visualization_prop{1}.edge_width = 2;

trees_visualization_prop{2}.node.shape = 'og';
trees_visualization_prop{2}.node.fill = 'g';
trees_visualization_prop{2}.edge_line_style = '-';
trees_visualization_prop{2}.edge_color = [0.4940 0.1840 0.5560];
trees_visualization_prop{2}.edge_width = 2;

plot(start_pose(1), start_pose(2), 'oc', 'MarkerSize', 15);
plot(goal_pose(1), goal_pose(2), 'oc', 'MarkerSize', 15);

%% Initialize RRT planner parameters

no_of_trees = 1;

switch(no_of_trees)
    case 1
        trees{1} = initialize_tree(start_pose);
    case 2
        % Initialize two trees one with start_pose as its root node and another
        % with goal_pose as its root node
        trees{1} = initialize_tree(start_pose);
        trees{2} = initialize_tree(goal_pose);
end

% Number of nearnest nodes to connect to sampled pose
n = 1;

% Number of random samples to generate
no_rand_samples = 1000;

% Number of regions into which samples should be split into
no_of_regions = 1;

%% Initialize planner flags
% tree_connected_flag = [false, false];
tree_connected_flag = zeros(1,no_of_trees);

motion_plan_success = false;

%% Plan motion using RRT

% Sample nodes
sampled_poses = generate_samples(no_rand_samples, no_of_regions, env_x_range, env_y_range);
% load("sampled_poses_2.mat");

stop_sampling = false;

pose_itr = 1;
region_itr = 1;
no_of_poses_sampled = 0;
total_iterations = 0;

no_of_poses_per_region = no_rand_samples/no_of_regions;

can_increment_iterators = false;

while ((~stop_sampling) && (pose_itr <= no_of_poses_per_region))
    while((pose_itr <= no_of_poses_per_region) && (no_of_poses_sampled <= 5))
        sampled_pose = sampled_poses(:, pose_itr, region_itr);
        
        if(mod(total_iterations,100) == 0)
            sampled_pose = start_pose;
            can_increment_iterators = false;
        elseif(mod(total_iterations,50) == 0)
            sampled_pose = goal_pose;
            can_increment_iterators = false;
        else
            can_increment_iterators = true;
        end

        %if(~check_if_colliding(occupancy_grid, sampled_pose, 0.3))
            sample_plot = plot(sampled_pose(1),sampled_pose(2),'or','MarkerFaceColor','r');
        
            for tree_itr = 1:length(trees)
                [nearest_node, exists_in_tree] = find_n_nearest_nodes(trees{tree_itr}, n, sampled_pose);
        
                for j = 1:length(nearest_node)
        
                    [trees{tree_itr}, tree_connected_flag(tree_itr)] = extend_tree(    ...
                        trees{tree_itr}, nearest_node(j), occupancy_grid, sampled_pose, true, 0, ...
                        'FigureHandle', fig_handle, ...
                        'NodeMarkerSpec', trees_visualization_prop{tree_itr}.node, ...
                        'EdgeColor', trees_visualization_prop{tree_itr}.edge_color, ...
                        'EdgeWidth', trees_visualization_prop{tree_itr}.edge_width);
        
                    if(tree_connected_flag(tree_itr))
                        break;
                    end
                end
            end

            delete(sample_plot);
        %end
    
        % Stopping criteria : Stop if both trees have connected to a common
        % node
        if(all(tree_connected_flag))
            if(no_of_trees == 2)
                motion_plan_success = true;
                stop_sampling = true;
                break;
            end
            
            if(no_of_trees == 1)
                if(sampled_pose == goal_pose)
                    motion_plan_success = true;
                    stop_sampling = true;
                    break;
                end
            end
        end

        total_iterations = total_iterations + 1;

        if(can_increment_iterators)
            pose_itr = pose_itr + 1;
            no_of_poses_sampled = no_of_poses_sampled + 1;
        end
        
    end

    region_itr = region_itr + 1;
    if(region_itr > no_of_regions)
        region_itr = 1;
    end
    no_of_poses_sampled = 0;

    
end

%% Connect root node of both trees

if(motion_plan_success)
    motion_plan = {};

    for tree_itr = 1:length(trees)
        motion_plan{tree_itr} = find_path_to_root(trees{tree_itr}, length(trees{tree_itr}.nodes));
    end

    if(length(trees) == 2)
        % Reverse the motion plan for tree starting from goal pose
        motion_plan{2} = flip(motion_plan{2},2);
        
        % Concatenate the motion plan from tree starting at start pose and the
        % motion plan from the tree starting at goal pose
        connected_motion_plan = [motion_plan{1}, motion_plan{2}(:,2:end)];
    else
        connected_motion_plan = motion_plan{1};
    end
end

%% Visualize Computed Motion Plan

if(motion_plan_success)
    figure
    show(occupancy_grid)
    hold on;
    
    plot(connected_motion_plan(1,1), connected_motion_plan(2,1), 'or', 'MarkerSize', 15, 'MarkerFaceColor', 'r');
    plot(connected_motion_plan(1,end), connected_motion_plan(2,end), 'og', 'MarkerSize', 15, 'MarkerFaceColor', 'g');
    for i = 2:size(connected_motion_plan,2)
        if(i < size(connected_motion_plan,2))
            plot(connected_motion_plan(1,i), connected_motion_plan(2,i), 'ob', 'MarkerFaceColor', 'b', 'MarkerSize', 10);
        end
        current_edge = connected_motion_plan(:,i-1:i);
        plot(current_edge(1,:), current_edge(2,:), '-k', 'LineWidth', 2);
    end
end

%% Print Performance Results
fprintf('\n#######################################################\n');
for i = 1:no_of_trees
    fprintf('Number of nodes in tree %d   : %d\n', i, length(trees{i}.nodes));
end
fprintf('Number of samples generated : %d\n', total_iterations);
fprintf('Number of regions           : %d\n', no_of_regions);
fprintf('Number of nearest nodes (n) : %d\n', n);

%% Save figures
exp = '26';

fig_1_name = strcat('exp_',exp,'_tree.fig');
saveas(figure(1), fig_1_name);

fig_2_name = strcat('exp_',exp,'_motion_plan.fig');
saveas(figure(2), fig_2_name);