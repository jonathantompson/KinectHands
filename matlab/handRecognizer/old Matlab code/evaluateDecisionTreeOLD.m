function [ label_data ] = evaluateDecisionTree( DT, image_data, WLFunction )

global im_width;
global im_height;
global max_offset;
im_width_border = im_width + 2*max_offset;
im_height_border = im_height + 2*max_offset;

% Border pixels have label = 2
label_data = ones(im_width_border * im_height_border, 1, 'uint16') * 2;
DT_height = log2(length(DT.weak_learners(:,1)) + 1);

% Just evaluate the internal pixels (not the added border)
for v = (max_offset + 1):(im_height + max_offset)
    for u = (max_offset + 1):(im_width + max_offset)
        index = (v-1) * im_width_border + u;
        
        cur_node = 1;
        while (true)
            cur_height = floor(log2(cur_node)) + 1;
            if (cur_height == DT_height)
                [max_prob, ind_max_prob] = max(DT.leaf_histograms(cur_node,:));
                label_data((v-1)*im_width_border + u) = ind_max_prob - 1;
                break;
            end
            if (DT.leaf_histograms(getLChild(cur_node), 1) == -1 || ...
                DT.leaf_histograms(getRChild(cur_node), 1) == -1)
                % Choose the label based on the maximum probability
                [max_prob, ind_max_prob] = max(DT.leaf_histograms(cur_node,:));
                label_data((v-1)*im_width_border + u) = ind_max_prob - 1;
                break;
            end
            result = WLFunction(image_data, index, DT.weak_learners(cur_node,:));
            if (result == 0)
                cur_node = getLChild(cur_node);
            else
                cur_node = getRChild(cur_node);
            end
        end
    end
end

end

