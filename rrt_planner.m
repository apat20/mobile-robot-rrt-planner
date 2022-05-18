close all;
clear;

% Parse environment description
env = parse_environment_description('environment_description.xml');

visualize_environment(env);

occupancy_grid = generate_occupancy_grid(env);

figure
show(occupancy_grid)
hold on;

fig_handle = gcf;

sampled_poses = generate_samples(1000, 1, env.size.length, env.size.breadth);

start_pose = [3;18];
goal_pose = [28;12];

tree.nodes{1} = start_pose;
tree.edges = 0;

for i = 1:1000
    nearest_node = find_n_nearest_nodes(tree, 1, sampled_poses(:,i));
    
    for j = 1:length(nearest_node)
        [tree, position_added] = extend_tree(tree, nearest_node(j), occupancy_grid, sampled_poses(:,i), true, 0, 'FigureHandle', fig_handle);
    end
end