% Function to find the first n nearest nodes to tree
%
% Input Arguments:
% tree              - A struct defining the connected tree
% n                 - Number of nearest nodes to return
% position          - Position to which we must find the n nearest nodes
%
% Output:
% nodes             - Vector of node ids

% Tree data structure
% tree.nodes        - Nodes present in the tree
% tree.edges        - Square matrix of size length(tree.nodes)

function nodes = find_n_nearest_nodes(tree, n, position)
    
    if(n > length(tree.nodes))
        n = length(tree.nodes);
    end
    
    nodes = zeros(1,n);
    
    dist_array = zeros(1,length(tree.nodes));

    for i = 1:length(tree.nodes)
        dist_array(i) = norm(tree.nodes{i}(1:2) - position);
    end

    dist_array_srt = sort(dist_array);

    i = 1;
    while i <= n
        indices = find(dist_array == dist_array_srt(i));
        
        for itr = 1:length(indices)
            nodes(i) = indices(itr);
            i = i + 1;
        end
    end
end