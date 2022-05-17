% Function to find the first n nearest nodes to tree
%
% Input Arguments:
% tree              - A struct defining the connected tree
% n                 - Number of nearest nodes to return
% position          - Position to which we must find the n nearest nodes
%
% Output:
% nodes             - Vector of node ids

% tree.nodes        - Nodes present in the tree
% tree.edges        - Square matrix of size length(tree.nodes)

function nodes = find_n_nearest_nodes(tree, n, position)
    
    if(n < length(tree.nodes))
        n = length(tree.nodes);
    end
    
    nodes = 1;

    min_d = norm(tree.nodes{1}(1:2) - position);

    for i = 2:length(tree.nodes)
        d = norm(tree.nodes{i}(1:2) - position);

        if(d < min_d)
            min_d = d;
            nodes = [i, nodes];
        end
    end

    nodes = nodes(1:n);
end