% Function to extend the tree
%
% Input Arguments:
% tree              - A struct defining the connected tree
% node              - ID of the Node from which tree should extend
% occupancy_grid    - Occupancy grid of the environment
% position          - Position to which we must extend to
% connect           - Flag to indicate if the tree should try to extend 
%                     upto the given position
% d_max             - Maximum length by which tree should extend
%                     Ignored if the 'connect' flag is set as true
%
% Output:
% extended_tree     - A struct defining the extended tree
% position_connect  - Flag to mention if the position has been added to the
%                     tree's node list

% Tree data structure
% tree.nodes        - Nodes present in the tree
% tree.edges        - Square matrix of size length(tree.nodes)

function [extended_tree, position_added] = extend_tree(tree, node, occupancy_grid, position, connect, d_max, varargin)
    
    extended_tree = tree;
    position_added = false;

    robot_radius = 0.30;

    prop_list = {'FigureHandle','NodeMarkerSpec','InterpolatedNodeMarkerSpec','MarkerSize','EdgeLineStyle', 'EdgeColor', 'EdgeWidth'};
    prop_list_map = 1:length(prop_list);
    temp_list = prop_list;

    visualize_search = false;

    fig_handle = {};
    node_marker_spec.shape = 'ob';
    node_marker_spec.fill = 'b';
    interp_node_marker_spec.shape = 'om';
    interp_node_marker_spec.fill = 'm';
    marker_size = 10;
    edge_line_style = '-';
    edge_color = 'k';
    edge_width = 1;

    arg_count = length(varargin);
    if mod(arg_count ,2) ~= 0
        error(message('extend_tree():InvalidNumInputs'));
    end

    for i = 1:2:arg_count
        for j = 1:length(temp_list)
            if(strcmp(varargin{i}, temp_list{j}))
                temp_list = setdiff(temp_list, {varargin{i}}, 'stable');
                
                switch prop_list_map(j)
                    case 1
                        fig_handle = varargin{i+1};
                        visualize_search = true;
                    case 2
                        node_marker_spec = varargin{i+1};
                    case 3
                        interp_node_marker_spec = varargin{i+1};
                    case 4
                        marker_size = varargin{i+1};
                    case 5
                        edge_line_style = varargin{i+1};
                    case 6
                        edge_color = varargin{i+1};
                    case 7
                        edge_width = varargin{i+1};
                end

                prop_list_map(j) = [];

                break;
            end
        end
    end

    if(visualize_search)
        figure(fig_handle)
        new_pos_plot_handle = plot(position(1), position(2), node_marker_spec.shape, 'MarkerSize', marker_size);
        drawnow();
    end

    if(~connect)
        dist = norm(tree.nodes{node}(1:2) - position(1:2));
        
        if(dist <= d_max)
            connect = 1;
        else
            dir = (position(1:2) - tree.nodes{node}(1:2)) / dist;
            position = tree.nodes{node}(1:2) + (d_max * dir);
        end
    end

    delta = 0.025;

    start_pose = tree.nodes{node};

    for i = delta:delta:1
        interpolated_pos = (1-i) * start_pose(1:2) + i * position(1:2);
        
        if(visualize_search)
            interpolated_pos_plot_handle = plot(interpolated_pos(1), interpolated_pos(2), interp_node_marker_spec.shape, 'MarkerSize', marker_size, 'MarkerFaceColor', interp_node_marker_spec.fill);
            drawnow();
        end

        if(check_if_colliding(occupancy_grid, interpolated_pos, robot_radius))
            if(i == delta)
                delete(new_pos_plot_handle);
                delete(interpolated_pos_plot_handle);
                return
            else
                interpolated_pos = (1-(i-delta)) * start_pose(1:2) + (i-delta) * position(1:2);
                break
            end
        end

        if(visualize_search)
            %pause(0.01);
            delete(interpolated_pos_plot_handle);
        end
    end

    extended_tree.nodes{end+1} = interpolated_pos;
                
    new_node = length(extended_tree.nodes);
    extended_tree.edges(node, new_node) = 1;
    extended_tree.edges(new_node, node) = 0;

    if(connect && i == 1)
        position_added = true;
    end

    if(visualize_search)
        delete(interpolated_pos_plot_handle);
        delete(new_pos_plot_handle);
        prev_pos = extended_tree.nodes{node};
        extend_pos = extended_tree.nodes{end};
        new_edge = [prev_pos, extend_pos];
        plot(extend_pos(1), extend_pos(2), node_marker_spec.shape, 'MarkerSize', marker_size, 'MarkerFaceColor', node_marker_spec.fill);
        plot(new_edge(1,:), new_edge(2,:), edge_line_style, 'Color', edge_color, 'LineWidth', edge_width);
        drawnow();
        pause(0.01);
    end
end