% Function to visualize an environment
%
% Input Arguments:
% env               - A struct defining the environment
%
% Optional arguments:
% varargin
%   varargin{1}     - Handle of figure to plot
%
% Output:
% fig_handle        - Handle to figure in which the environment was plotted
function fig_handle = visualize_environment(env, varargin)

    if(length(varargin) == 1)
        fig_handle = varargin{1};
        figure(fig_handle);
    else
        fig_handle = figure;
    end

    axis equal;
    hold on;
    grid on;
    xlim([0 env.size.length])
    ylim([0 env.size.breadth])
    xlabel('X (m)')
    ylabel('Y (m)')
    
    % Plot polygonal obstacles
    for i = 1:length(env.polygonal_obstacles)
        pgon = polyshape(env.polygonal_obstacles{i}.');
        plot(pgon, 'FaceColor', 'black');
    end

    % Plot circular obstacles
    for i = 1:length(env.circular_obstacles)
        obstacle_corners = [env.circular_obstacles{i}.radius * cos(0:0.1:2*pi); env.circular_obstacles{i}.radius * sin(0:0.1:2*pi)];
        obstacle_corners = obstacle_corners + env.circular_obstacles{i}.center;
%         plot(obstacle_corners(1,:), obstacle_corners(2,:), '-k');
        fill(obstacle_corners(1,:), obstacle_corners(2,:), 'black', 'FaceAlpha', 0.35);
    end
end