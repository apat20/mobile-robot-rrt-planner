% Function to plot a given robot configuration
%
% Input Arguments:
% config                    - Pose of the robot
% fig_handle                - Handle of the plot
%
% Optional arguments:
% varargin
%   'MarkerStyle'           - Style of marker which represents the robot
%   'MarkerSize'            - Size of marker which represents the robot
%   'DirectionScale'        - Scale of the direction vector of robot
%   'DirectionLineWidth'    - Width of the direction vector of robot
%
% Output:
% config_plot_handle        - Plot handle of robot configuration

function config_plot_handle = plot_config(config, fig_handle, varargin)
    figure(fig_handle);
    hold on;
    
    arg_count = length(varargin);

    prop_list = {'MarkerStyle', 'MarkerSize', 'DirectionScale', 'DirectionLineWidth', 'PlotOrientation'};
    prop_list_map = 1:length(prop_list);
    temp_list = prop_list;

    marker_style = 'b.';
    marker_size = 25;
    direction_scale = 2;
    direction_line_width = 2;
    plot_orientation = true;

    if mod(arg_count ,2) ~= 0
        error(message('plot_config():InvalidNumInputs'));
    end

    for i = 1:2:arg_count
        for j = 1:length(temp_list)
            if(strcmp(varargin{i}, temp_list{j}))
                temp_list = setdiff(temp_list, {varargin{i}}, 'stable');
                
                switch prop_list_map(j)
                    case 1
                        marker_style = varargin{i+1};
                    case 2
                        marker_size = varargin{i+1};
                    case 3
                        direction_scale = varargin{i+1};
                    case 4
                        direction_line_width = varargin{i+1};
                    case 5
                        plot_orientation = varargin{i+1};
                end

                prop_list_map(j) = [];

                break;
            end
        end
    end

    x = config(1);
    y = config(2);
    theta = config(3);

    config_plot_handle.pos_handle = plot(x, y, marker_style, 'MarkerSize', marker_size);
    if(plot_orientation)
        config_plot_handle.dir_handle = quiver(x, y, cos(theta), sin(theta), direction_scale, '-r', 'LineWidth', direction_line_width);
    end
end