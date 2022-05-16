close all;
clear;

% Parse environment description
env = parse_environment_description('environment_description.xml');

visualize_environment(env);

occupancy_grid = generate_occupancy_grid(env);

figure
show(occupancy_grid)
hold on;

robot_radius = 0.25;
pose = [robot_radius;10];
points_on_robot = pose + (robot_radius * [cos(0:0.2:2*pi); sin(0:0.2:2*pi)]);
robot_chassis = plot(points_on_robot(1,:), points_on_robot(2,:), '-b', 'LineWidth', 2);

for x = robot_radius:0.1:(30-robot_radius)
    pose = [x;10];

    is_colliding = check_if_colliding(occupancy_grid, pose, robot_radius);

    points_on_robot = pose + (robot_radius * [cos(0:0.2:2*pi); sin(0:0.2:2*pi)]);
    
    delete(robot_chassis);

    if(is_colliding)
        robot_chassis = plot(points_on_robot(1,:), points_on_robot(2,:), '-r');
    else
        robot_chassis = plot(points_on_robot(1,:), points_on_robot(2,:), '-g');
    end
    pause(0.05);
end