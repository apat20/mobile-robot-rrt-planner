close all;
clear;

%% Parse environment from environment descriptioin file
env = parse_environment_description('environment_description.xml');

%% Construct unidirectional tree containing a collision-free path to the goal 
fig_handle = visualize_environment(env);

start_pose = [3;18;pi/4];
goal_pose = [28;12;pi];

plot_config(start_pose, fig_handle, 'MarkerStyle', '.r');
plot_config(goal_pose, fig_handle, 'MarkerStyle', '.g');

[plan_result, connected_tree] = construct_unidirectional_search_tree(start_pose, goal_pose, env, 'FigureHandle', fig_handle);

plot_config(start_pose, fig_handle, 'MarkerStyle', '.r');
plot_config(goal_pose, fig_handle, 'MarkerStyle', '.g');

%% Determine path from constructed unidirectional tree

if(plan_result)
    fig_handle = visualize_environment(env);
    plot_config(start_pose, fig_handle, 'MarkerStyle', '.r');
    plot_config(goal_pose, fig_handle, 'MarkerStyle', '.g');

    motion_plan = find_path_to_root(connected_tree, length(connected_tree.nodes));

    for i = 2:(length(motion_plan))
        if(i < length(motion_plan))
            plot_config(connected_tree.nodes{motion_plan(i)}, fig_handle, 'MarkerStyle', '.b');
        end
        edge = [connected_tree.nodes{motion_plan(i-1)}(1:2), connected_tree.nodes{motion_plan(i)}(1:2)];
        plot(edge(1,:), edge(2,:), '-k');
    end
end