% Function to check for collision between the robot and any obstacle in the
% environment that is defined using an occupancy map
%
% Input Arguments:
% occupancy_map     - Occupancy map defining the enviornment obstacles
% robot_pose        - Pose of robot
% robot_radius      - Radius of the circular robot base
%
% Output:
% is_colliding      - Returns true if colliding and false if collision free

function is_colliding = check_if_colliding(occupancy_map, robot_pose, robot_radius)
    points_on_robot = robot_pose;
    if(robot_radius)
        points_on_robot = robot_pose + (robot_radius * [cos(0:0.2:2*pi); sin(0:0.2:2*pi)]);
    end

    occupancy_val = checkOccupancy(occupancy_map, points_on_robot.');

    if(any(occupancy_val, 'all'))
        is_colliding = 1;
    else
        is_colliding = 0;
    end
end