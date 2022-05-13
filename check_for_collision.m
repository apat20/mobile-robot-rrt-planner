% Function to check if a given robot configuration is colliding with
% obstacles present in the environment
%
% Input Arguments:
% config            - Configuration of the robot
% env               - A struct defining the environment
%
% Output:
% result            - Returns true if give configuration is colliding

function result = check_for_collision(config, env)
    x = config(1); y = config(2);

    if(x < 0 || y < 0)
        error(message('check_for_collision():Config out of bounds'));
    elseif(x > env.size.length || y > env.size.breadth)
        error(message('check_for_collision():Config out of bounds'))
    end

    for i = 1:length(env.polygonal_obstacles)
        [~, distance] = distance_to_polygon(env.polygonal_obstacles{i}, [x;y]);

        if(distance < 0)
            result = true;
            return;
        end
    end

    for i = 1:length(env.circular_obstacles)
        [~, distance] = distance_to_circle(env.circular_obstacles{i}.center, env.circular_obstacles{i}.radius, [x;y]);

        if(distance < 0)
            result = true;
            return;
        end
    end

    result = false;
end