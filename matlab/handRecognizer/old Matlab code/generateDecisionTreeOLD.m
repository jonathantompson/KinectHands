function [ DT ] = generateDecisionTree( DB, WLCoeffs, WLFunction, tree_height, brute_force )

global im_width; im_width = int32(im_width);
global im_height; im_height = int32(im_height);
global WL_samples_per_node;
global max_offset; max_offset = int32(max_offset);

% DB:              database of training data, N 1D images
% WLCoeffs:        Cell of possible coefficients.  Each cell item describes
%                  the subset of possible WL coefficients, for the 1st,
%                  2nd, 3rd coeff, etc.
% WLFunction:      The low level weak lerner function pointer
% tree_height:     Desired tree height

% Assume the tree is fully bushy and preallocate the max possible size
tree_size = (2^tree_height - 1);
num_labels = 2;
DT = [];
DT.weak_learners = zeros(tree_size, length(WLCoeffs), class(WLCoeffs{1}));
DT.leaf_histograms = ones(tree_size, num_labels) * -1;  % -1 indicates we're not a leaf

% We're going to desend the tree DFS.  To avoid recursion overhead, we're 
% going to keep an explicit stack of nodes to be processed.  Since this is 
% a binary tree, we'll never have more than 2 * tree_height nodes.
stack_size = tree_height*2;
stack = zeros(stack_size, 1, 'uint32');
stack_heights = zeros(stack_size, 1, 'uint32');
stack_ptr = uint32(1);
stack(stack_ptr) = uint32(1);  % Seed the stack with the root node
stack_heights(stack_ptr) = uint32(1);

num_images = length(DB.image_data);

im_width_border = im_width + 2*max_offset;
im_height_border = im_height + 2*max_offset;

% Since we're desending the tree DFS we need to store the occupancy
% list only for each level of the stack.
% The occupancy list takes up the most memory of all the data structures,
% It's the size of the (input data * 2 * height)
% So we're going to pack it tightly into 32 bit ints.
occupancy_length = int32(ceil((double(im_width_border) * double(im_height_border)) / 32));
occupancy = zeros(stack_size, num_images, occupancy_length, 'uint32');
% Only fill in the occupancy for the internal pixels, and which do not have
% zero values.
occupancy_image = zeros(occupancy_length*32, 1, 'uint8');
for cur_image_ind = 1:num_images
    occupancy_image(find(DB.image_data{cur_image_ind} ~= 0)) = uint8(1);
    occupancy(1,cur_image_ind,:) = pack32BitOccupancy(occupancy_image);
end


% Permute the weak learners so we can record which learners we've already
% used.  The used list is also done per stack point.  Same with entropy.
WLSet = permuteWLCoeffs(WLCoeffs);
num_WLPermutations = uint32(length(WLSet(:,1)));
if (WL_samples_per_node > num_WLPermutations)
    WL_samples_per_node = num_WLPermutations;
end
WLUsed = zeros(stack_size, num_WLPermutations, 'uint8');
entropy = zeros(stack_size, 1);

% Calculate the entropy of the root:
hist_root = zeros(2, 1);
for cur_image_ind = 1:num_images
    hist_root(1) = hist_root(1) + length(find(DB.label_data{cur_image_ind} == 0));
    hist_root(2) = hist_root(2) + length(find(DB.label_data{cur_image_ind} == 1));
end
P_root = hist_root ./ sum(hist_root);
entropy(1) = sum(-P_root .* log2(P_root + 0.0000001));
DT.leaf_histograms(1,:) = P_root;

% Main loop:
num_nodes_finished = 0;
while (stack_ptr > 0)
    disp(['Processing tree node ', num2str(num_nodes_finished + 1), ' of ', num2str(tree_size)]);
    
    % Get the next node off the stack
    cur_node = stack(stack_ptr);
    cur_height = stack_heights(stack_ptr);
    
    % Now, from the list of still avaliable WL Permunations:
    % Choose a maximal WL from a random subset of 10
    max_gain = 0;
    index_best_WL = 0;
    num_attemps = 0;
    for cur_n = 1:num_WLPermutations
        if (~brute_force) 
            if (num_attemps >= WL_samples_per_node)
               break; 
            end
            n = floor(rand(1) * (num_WLPermutations - 1)) + 1;
        else
            n = cur_n;
        end
        if (WLUsed(stack_ptr, n) == 0)
            if (~brute_force)
                disp(['  --> Trying weak learner ', num2str(num_attemps + 1), ' of ', num2str(WL_samples_per_node)]);
            else
                disp(['  --> Trying weak learner ', num2str(num_attemps + 1), ' of ', num2str(num_WLPermutations)]);
            end
            % test the nth WL --> then Calculate entropy
            num_attemps = num_attemps + 1;
            cur_WL = WLSet(n, :);
            
            % For all of the data points still alive, bin all of the data points 
            % using the weak lerner
            hist_left = zeros(2, 1);
            hist_right = zeros(2, 1);
            length_parent_occupancy = 0;
            occupancy_left = zeros(num_images, occupancy_length, 'uint32');
            occupancy_right = zeros(num_images, occupancy_length, 'uint32'); 
            
            occupancy_left_image = zeros(occupancy_length*32, 1, 'uint8');
            occupancy_right_image = zeros(occupancy_length*32, 1, 'uint8');  
                
            for cur_image_ind = 1:num_images
                % Unpack the occupancy list for this image and this stack
                % position into a flat array
                occupancy_image = unpack32BitOccupancy(occupancy(stack_ptr, cur_image_ind, :));
                
                % DEBUG: Plot the occupancy image
                % im_dat = double(reshape(occupancy_image(1:im_width_border*im_height_border), im_width_border, im_height_border)');
                % Argb = cat(3, im_dat, im_dat, im_dat);
                % imshow(Argb);
                % END DEBUG
                
                indices = int32(find(occupancy_image == 1));
                length_parent_occupancy = length_parent_occupancy + length(indices);
                
                results = ones(im_width_border * im_height_border, 1, 'uint8') * 2;  %% All results outside of the occupancy list will be = 2
                results(indices) = WLFunction(DB.image_data{cur_image_ind}, indices, cur_WL);
                
                indices_left = find(results == 0);
                occupancy_left_image(indices_left) = uint8(1);
                indices_right = find(results == 1);
                occupancy_right_image(indices_right) = uint8(1);
                
                for cur_label = 0:1
                    if (length(indices_left) > 0)
                        hist_left(cur_label+1) = hist_left(cur_label+1) + length(find(DB.label_data{cur_image_ind}(indices_left) == cur_label));
                    end
                    if (length(indices_right) > 0)
                        hist_right(cur_label+1) = hist_right(cur_label+1) + length(find(DB.label_data{cur_image_ind}(indices_right) == cur_label));
                    end
                end
                
                % Now re-pack the occupancy list tightly
                occupancy_left(cur_image_ind, :) = pack32BitOccupancy(occupancy_left_image);
                occupancy_right(cur_image_ind, :) = pack32BitOccupancy(occupancy_right_image);

            end  % for cur_image_ind = 1:length(DB.image_data)
            
            % normalize the histograms
            if (sum(hist_left) > 0)
                P_left = hist_left ./ sum(hist_left);
            else
                P_left = hist_left;
            end
            if (sum(hist_right) > 0)
                P_right = hist_right ./ sum(hist_right);
            else
                P_right = hist_right;
            end
            
            % From the left and right histograms calculate the entropy
            entr_parent = entropy(stack_ptr);
            entr_left = sum(-P_left .* log2(P_left + 0.0000001));  % avoid nan when p = 0
            entr_right = sum(-P_right .* log2(P_right + 0.0000001));
            
            % Calculate the normalized information gain
            % From Murphy's matlab code (thanks Papa Murphy!):
            % gain = h0 - (h1 * s1/s0 + h2 * s2/s0);
            gain = entr_parent - (entr_left * sum(hist_left)/length_parent_occupancy + entr_right * sum(hist_right)/length_parent_occupancy);
            
            if (gain > max_gain)
                index_best_WL = n;
                max_gain = gain;
                DT.weak_learners(cur_node, :) = cur_WL;
                occupancy_left_best_WL = occupancy_left;
                occupancy_right_best_WL = occupancy_right;
                P_left_best_WL = P_left;
                P_right_best_WL = P_right;
                entr_left_best_WL = entr_left;
                entr_right_best_WL = entr_right;
            end
        end  % if (WLUsed(stack_ptr, n) == 0)
    end  % for n = 1:num_WLPermutations
    
    if (index_best_WL == 0)
        fprintf('Couldnt find a weak lerner if info gain > 0!\n');
        stack_ptr = stack_ptr - 1;
    else
        DT.leaf_histograms(getLChild(cur_node),:) = P_left_best_WL;
        DT.leaf_histograms(getRChild(cur_node),:) = P_right_best_WL;
           
        WLUsed_children = WLUsed(stack_ptr, :);
        WLUsed_children(index_best_WL) = 1;

%         fprintf('    --> Using WL = <%d, %d, %d>, for info gain of %.4e\n', ...
%             DT.weak_learners(cur_node, 1), DT.weak_learners(cur_node, 2), DT.weak_learners(cur_node, 3), ...
%             max_gain);

        % Are we a leaf?  If not, then put our children on the stack
        stack_ptr = stack_ptr - 1;  % Pop this node off the stack
        if (cur_height < (tree_height-1))
            % Add left child to the stack for processing
            stack_ptr = stack_ptr + 1;
            stack(stack_ptr) = getLChild(cur_node);
            stack_heights(stack_ptr) = cur_height + 1;
            occupancy(stack_ptr, :, :) = occupancy_left_best_WL;
            WLUsed(stack_ptr, :) = WLUsed_children;
            entropy(stack_ptr) = entr_left_best_WL;

            % Add right child to the stack for processing
            stack_ptr = stack_ptr + 1;
            stack(stack_ptr) = getRChild(cur_node);
            stack_heights(stack_ptr) = cur_height + 1;
            occupancy(stack_ptr, :, :) = occupancy_right_best_WL;
            WLUsed(stack_ptr, :) = WLUsed_children;
            entropy(stack_ptr) = entr_right_best_WL;
        end

        num_nodes_finished = num_nodes_finished + 1;
    end
end


end

