% Function to traverse to the root of a tree from a given node on the tree
%
% Input Arguments:
% tree              - A struct defining the connected tree
% node_id           - The id of the node(index of the node) from which to
%                     traverse to the root of the tree
%
% Output:
% path              - The sequence of nodes through which we can traverse
%                     to the root of the tree from the given node
function path = find_path_to_root(tree, node_id)
    current_node = node_id;

    node_path = current_node;
    path = tree.nodes{current_node};

    while(current_node > 1)
        parent_node = find(tree.edges(:,current_node));

        if(length(parent_node) ~= 1)
            error("find_path_to_root() : Node %d has more than one parent", current_node);
        end

        node_path = [parent_node, node_path];
        path = [tree.nodes{parent_node}, path];
        current_node = parent_node;
    end

end