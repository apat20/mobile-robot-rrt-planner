% Function to get a random configuration given range of the environment
%
% Input Arguments:
% x_range   - Range of x co-ordinates of the environment
% y_range   - Range of y co-ordinates of the environment
%
% Output:
% x         - x co-ordinate of generated configuration
% y         - y co-ordinate of generated configuration
% theta     - Orientation of generated configuration
function [x, y, theta] = get_random_config(x_range, y_range)
    rand_vals = rand(3,1);
    x = rand_vals(1) * x_range;
    y = rand_vals(2) * y_range;
    theta = wrapToPi(rand_vals(3) * 2 * pi);
end