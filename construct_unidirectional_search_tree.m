% Function to construct a unidirectional search tree given an initial and
% final pose of the robot and an environment description
%
% Input Arguments:
% initial_pose         - Initial pose of the robot
% final_pose           - Final pose of the robot
% env                  - A struct defining the environment
%
% Optional arguments:
% varargin
%   'FigureHandle'     - Handle of figure to plot
%
% Output:
% success              - Flag to indicate success/failure of tree construction
% tree                 - A struct containing the nodes and edges in the tree

function [success, tree] = construct_unidirectional_search_tree(initial_pose, final_pose, env, varargin)
    % Set result as fail initially
    success = false;

    prop_list = {'FigureHandle'};
    prop_list_map = 1:length(prop_list);
    temp_list = prop_list;

    visualize_search = false;

    fig_handle = {};

    arg_count = length(varargin);
    if mod(arg_count ,2) ~= 0
        error(message('construct_unidirectional_search_tree():InvalidNumInputs'));
    end

    for i = 1:2:arg_count
        for j = 1:length(temp_list)
            if(strcmp(varargin{i}, temp_list{j}))
                temp_list = setdiff(temp_list, {varargin{i}}, 'stable');
                
                switch prop_list_map(j)
                    case 1
                        fig_handle = varargin{i+1};
                        visualize_search = true;
                end

                prop_list_map(j) = [];

                break;
            end
        end
    end

    check_with_final_pose_cnt = 25;

    % Number of random configurations that need to be sampled
    sample_count = 1000;

    % Array to store the generated random configurations
    random_samples = rand(3, 1000);
    random_samples(1,:) = random_samples(1,:) * env.size.length;
    random_samples(2,:) = random_samples(2,:) * env.size.breadth;
    random_samples(3,:) = wrapToPi(random_samples(3,:));

    sample_itr = 1;

    % Initialize the tree struct
    tree.nodes = {initial_pose};
    tree.edges = [];

    % Parameters for interpolating between randomly sample configuration
    % and nearest configuration on tree
    t_start = 0.05;
    t_end = 0.95;
    t_increment = 0.05;

    % Check if initial_pose and final_pose are collision free
    if(check_for_collision(initial_pose, env))
        error(message('construct_unidirectional_search_tree():Initial pose is not collision free'));
    elseif(check_for_collision(final_pose, env))
        error(message('construct_unidirectional_search_tree():Final pose is not collision free'));
    end

    % Loop until final_pose is added to the tree or if we run out of
    % samples
    while sample_itr <= 1000
        if(rem(sample_itr, check_with_final_pose_cnt) == 0)
            rand_config = final_pose;
        else
            %[rand_x, rand_y, rand_theta] = get_random_config(env.size.length, env.size.breadth);
            %rand_config = [rand_x, rand_y, rand_theta].';
            rand_config = random_samples(:,sample_itr);
        end

        sample_itr = sample_itr + 1;

        % Check if sampled random configuration is collision free
        if(check_for_collision(rand_config, env))
            continue;
        end

        if(visualize_search)
            rand_config_plot_handle = plot_config(rand_config, fig_handle, 'MarkerStyle', '.k');
            drawnow();
        end

        % Find node on tree that is closest to the sampled random
        % configuration
        closest_dist = norm(rand_config(1:2) - initial_pose(1:2));
        closest_node = 1;
        for node_itr = 2:length(tree.nodes)
            dist = norm(rand_config(1:2) - tree.nodes{node_itr}(1:2));

            if(dist < closest_dist)
                closest_dist = dist;
                closest_node = node_itr;
            end
        end

        % Check if interpolated path between nearest node and sampled
        % random node is collision free
        interpolated_path_collision_free = true;
        for t = t_start:t_increment:t_end
            interpolated_config = ((1-t) * tree.nodes{closest_node}) + (t * rand_config);

            if(visualize_search)
                interpolated_config_plot_handle = plot_config(interpolated_config, fig_handle, 'MarkerStyle', 'ok', 'MarkerSize', 7);
            end

            if(check_for_collision(interpolated_config, env))
                interpolated_path_collision_free = false;
                
                if(t > t_start)
                    t_collision_free = t - t_increment;
                    tree.nodes{end+1} = ((1-t_collision_free) * tree.nodes{closest_node}) + (t_collision_free * rand_config);
                    new_node = length(tree.nodes);
                    tree.edges(closest_node, new_node) = 1;
                    tree.edges(new_node, closest_node) = 0;

                    if(visualize_search)
                        new_config_plot_handle = plot_config(tree.nodes{end}, fig_handle, 'MarkerStyle', '.b');
                        
                        edge = [tree.nodes{end}(1:2), tree.nodes{closest_node}(1:2)];
                        plot(edge(1,:), edge(2,:), '-b');

                        pause(0.05);
                    end
                end

                if(visualize_search)
                    delete(rand_config_plot_handle.pos_handle);
                    delete(rand_config_plot_handle.dir_handle);

                    delete(interpolated_config_plot_handle.pos_handle);
                    delete(interpolated_config_plot_handle.dir_handle);
                end

                break;
            else
                if(visualize_search)
                    delete(interpolated_config_plot_handle.pos_handle);
                    delete(interpolated_config_plot_handle.dir_handle);
                end
            end
        end

        % Add sampled random node to tree if the path is collision free
        if(interpolated_path_collision_free)
            tree.nodes{end+1} = rand_config;
            new_node = length(tree.nodes);
            tree.edges(closest_node, new_node) = 1;
            tree.edges(new_node, closest_node) = 0;

            if(visualize_search)
                delete(rand_config_plot_handle.pos_handle);
                delete(rand_config_plot_handle.dir_handle);

                new_config_plot_handle = plot_config(tree.nodes{end}, fig_handle, 'MarkerStyle', '.b');
                
                edge = [tree.nodes{end}(1:2), tree.nodes{closest_node}(1:2)];
                plot(edge(1,:), edge(2,:), '-b');
            end
        end

        % Stop if final pose is added to the tree
        if(tree.nodes{new_node} == final_pose)
            success = true;
            return;
        end

    end
    
end