% Function to get the closest distance between a point and a circle
%
% Input Arguments:
% circle_center     - 2x1 matrix containing the center of the circle
% circle_radius     - Scalar defining the radius of the circle
% robot_position    - 2x1 matrix of the format [x; y]
%                     Position of the robot (x,y)
% 
% Output:
% closest_pts       - 2x2 matrix of the format [x_r,x_p; y_r,y_p]
%                     Closest point on the circle (x_p,y_p) to the
%                     robot (x_r,y_r)
% distance          - Distance between closest point on the circle and the
%                     robot position (x,y)
function [closest_pts, distance] = distance_to_circle(circle_center, circle_radius, robot_position)
    robot_radius = 0.5;

    distance = norm(circle_center - robot_position);

    if (distance >= (robot_radius + circle_radius))
        dir = (circle_center - robot_position) / distance;
        closest_pts = zeros(2,2);
        closest_pts(:,1) = robot_position + (dir * robot_radius);
        closest_pts(:,2) = circle_center + (dir * -circle_radius);
    else
        distance = -1;
        closest_pts = [robot_position, robot_position];
    end

end