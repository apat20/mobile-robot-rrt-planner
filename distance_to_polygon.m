% Function to get the closest distance between a point and a polygon
%
% Input Arguments:
% polygon_vertices  - nx2 matrix containing the n polygon vertices
% robot_position    - 2x1 matrix of the format [x; y]
%                     Position of the robot (x,y)
% 
% Output:
% closest_pts       - 2x2 matrix of the format [x_r,x_p; y_r,y_p]
%                     Closest point on the polygon (x_p,y_p) to the
%                     robot (x_r,y_r)
% distance          - Distance between closest point (x_p,y_p) and the
%                     robot position (x,y)
function [closest_pts, distance] = distance_to_polygon(polygon_vertices, robot_position)

    polygon_edges = polygon_vertices(:,1:2);
    for i = 2:(size(polygon_vertices, 2)-1)
        polygon_edges(:,:,end+1) = polygon_vertices(:,i:i+1);
    end
    polygon_edges(:,:,end+1) = [polygon_vertices(:,end), polygon_vertices(:,1)];

    % Count the number of edges in the polygon to which the point lies to
    % the left of
    left_counter = 0;
    for i = 1:size(polygon_edges, 3)
        if(~check_if_to_the_right(robot_position, polygon_edges(:,:,i)))
            left_counter = left_counter + 1;
        end
    end

    if(left_counter == size(polygon_edges, 3))
        % If point lies to the left of all the edges, the point lies inside the
        % polygon
        distance = -1;
        closest_pts = [robot_position, robot_position];
    else
        % Else find the polygon edge to which the point lies closest
        [closest_pts,distance] = distance_to_line_segment(polygon_edges(:,:,1), robot_position);
        for i = 2:size(polygon_edges, 3)
            [c_pts,d] = distance_to_line_segment(polygon_edges(:,:,i), robot_position);
            if(d < distance)
                closest_pts = c_pts;
                distance = d;
            end
        end
    end
end

% Function to check if a given point lies to the right of a line segment
function res = check_if_to_the_right(point, line_segment)
    vec_1 = line_segment(:,2) - line_segment(:,1);
    vec_2 = point - line_segment(:,1);

    % Compute cross product of vec_1 and vec_2
    cross_product = [0; 0; (vec_1(1) * vec_2(2)) - (vec_2(1) * vec_1(2))];

    if(cross_product(3) <= 0)
        res = true;
    else
        res = false;
    end
end