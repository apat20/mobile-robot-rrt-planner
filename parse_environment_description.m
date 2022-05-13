% Function to parse an xml file containing the environment description and
% return a structure containing the environment description
%
% Input Arguments:
% filename          - File name of the environment description xml file
%
% Output:
% env               - Structure define the environment
function env = parse_environment_description(filename)
    env_struct = readstruct(filename);
    
    % Parse length and breadth of environment
    env.size.length = env_struct.size.lengthAttribute;
    env.size.breadth = env_struct.size.breadthAttribute;

    env.polygonal_obstacles = {};
    env.circular_obstacles = {};

    % Parse environment description file for obstacles
    obstacle_types = fieldnames(env_struct.obstacles);

    % Parse all polygonal obstacles
    if(any(strcmp(obstacle_types, 'polygon')))
        polygonal_obstacles = env_struct.obstacles.polygon;

        for i = 1:size(polygonal_obstacles, 2)
            polygonal_obs = polygonal_obstacles(i);

            poly_vertices = [0;0];

            vertices = polygonal_obs.vertex;
            for j = 1:size(vertices, 2)
                vertex_x = vertices(j).xAttribute;
                vertex_y = vertices(j).yAttribute;

                poly_vertices(:,end+1) = [vertex_x; vertex_y];
            end
            poly_vertices = poly_vertices(:,2:end);
            env.polygonal_obstacles{end+1} = poly_vertices;
        end
    end

    % Parse all circular obstacles
    if(any(strcmp(obstacle_types, 'circle')))
        circular_obstacles = env_struct.obstacles.circle;
        
        for i = 1:size(circular_obstacles, 2)
            circular_obs = circular_obstacles(i);

            c_obs.center = [circular_obs.center_xAttribute; circular_obs.center_yAttribute];
            c_obs.radius = circular_obs.radiusAttribute;
        end
        env.circular_obstacles{end+1} = c_obs;
    end
end