% Function to initiazlize the tree data structure
%
% Input Arguments:
% root_node         - Root node of the tree
%
% Output:
% tree              - A struct defining the tree with root_node as its root

function tree = initialize_tree(root_node)
    tree.nodes{1} = root_node;
    tree.edges = 0;
end