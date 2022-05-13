% Function to get the closest distance between a point and a line segment
%
% Input Arguments:
% line_segment      - 2x2 matrix of the format [x1, x2; y1, y2]
%                     End points (x1,y1) and (x2,y2) of the line segment
% robot_position    - 2x1 matrix of the format [x; y]
%                     Position of the robot (x,y)
% 
% Output:
% closest_pts       - 2x2 matrix of the format [x_r,x_p; y_r,y_p]
%                     Closest point on the line segment (x_p,y_p) to the
%                     robot (x_r,y_r)
% distance          - Distance between closest point (x_p,y_p) and the
%                     robot position (x,y)
function [closest_pts, distance] = distance_to_line_segment(line_segment, robot_position)

    robot_radius = 0.5;
    closest_pts = zeros(2,2);

    x1 = line_segment(1,1); y1 = line_segment(2,1);
    x2 = line_segment(1,2); y2 = line_segment(2,2);

    slope_of_line = (y2 - y1) / (x2 - x1);

    if(slope_of_line == 0)
        foot_x = robot_position(1);
        foot_y = y1;
    elseif(isinf(slope_of_line))
        foot_x = x1;
        foot_y = robot_position(2);
    else
        % Determine foot of perpendicular from (x,y) to the line
        foot_x = ((slope_of_line * x1) + (robot_position(1)/slope_of_line) + robot_position(2) - y1) / ((slope_of_line^2 + 1) / slope_of_line);
        foot_y = (-(1/slope_of_line) * (foot_x - robot_position(1))) + robot_position(2);
    end
        
    f_d_1 = norm([foot_x; foot_y] - line_segment(:,1));
    f_d_2 = norm([foot_x; foot_y] - line_segment(:,2));
    len = norm(line_segment(:,1) - line_segment(:,2));

    % Check if foot of perpendicular lies inbetween the given points
    if((f_d_1 + f_d_2) == len)
        closest_pts(:,2) = [foot_x; foot_y];
        distance = norm(closest_pts(:,2) - robot_position);
    else
        d_1 = norm(line_segment(:,1) - robot_position);
        d_2 = norm(line_segment(:,2) - robot_position);

        if(d_1 <= d_2)
            distance = d_1;
            closest_pts(:,2) = line_segment(:,1);
        else
            distance = d_2;
            closest_pts(:,2) = line_segment(:,2);
        end
    end

    dir = closest_pts(:,2) - robot_position;
    dir = dir / norm(dir);

    closest_pts(:,1) = robot_position + (robot_radius * dir);

end