close all;
clear;

% Parse environment description
env = parse_environment_description('environment_description.xml');

% visualize_environment(env);

occupancy_grid = generate_occupancy_grid(env);

figure
show(occupancy_grid)
hold on;

fig_handle = gcf;

sampled_poses = generate_samples(1000, 1, env.size.length, env.size.breadth);
% load("sampled_poses.mat");

start_pose = [3;18];
goal_pose = [28;12];

trees{1} = initialize_tree(start_pose);
trees{2} = initialize_tree(goal_pose);

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

% Number of nearnest nodes to connect to sampled pose
n = 2;

plot(start_pose(1), start_pose(2), 'oc', 'MarkerSize', 15);
plot(goal_pose(1), goal_pose(2), 'oc', 'MarkerSize', 15);

tree_connected_flag = [false, false];

motion_plan_success = false;

for i = 1:1000

    sample_plot = plot(sampled_poses(1,i),sampled_poses(2,i),'or','MarkerFaceColor','r');

    fprintf('\nSample %d\n', i);

    for tree_itr = 1:length(trees)
        nearest_node = find_n_nearest_nodes(trees{tree_itr}, n, sampled_poses(:,i));

        for j = 1:length(nearest_node)
            fprintf('Growing from tree %d node %d\n', tree_itr, nearest_node(j));

            [trees{tree_itr}, tree_connected_flag(tree_itr)] = extend_tree(    ...
                trees{tree_itr}, nearest_node(j), occupancy_grid, sampled_poses(:,i), true, 0, ...
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

    if(all(tree_connected_flag))
        motion_plan_success = true;
        break;
    end
end

%% Connect root of both trees

if(motion_plan_success)
    motion_plan = {};

    for tree_itr = 1:length(trees)
        motion_plan{tree_itr} = find_path_to_root(trees{tree_itr}, length(trees{tree_itr}.nodes));
    end
end

motion_plan{2} = flip(motion_plan{2},2);
connected_motion_plan = [motion_plan{1}, motion_plan{2}(:,2:end)];

%% Visualize Computed Motion Plan

figure
show(occupancy_grid)
hold on;

for i = 2:size(connected_motion_plan,2)
    plot(connected_motion_plan(1,i), connected_motion_plan(2,i), 'ob', 'MarkerFaceColor', 'b', 'MarkerSize', 10);
    current_edge = connected_motion_plan(:,i-1:i);
    plot(current_edge(1,:), current_edge(2,:), '-k', 'LineWidth', 2);
end
plot(connected_motion_plan(1,1), connected_motion_plan(2,1), 'or', 'MarkerSize', 15, 'MarkerFaceColor', 'r');
plot(connected_motion_plan(1,end), connected_motion_plan(2,end), 'og', 'MarkerSize', 15, 'MarkerFaceColor', 'g');