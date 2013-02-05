function [ ] = printDTHistograms( DT_histograms )

global im_width;
global im_height;

DT_height = log2(length(DT_histograms(1,:)));
num_leaves = 2^(DT_height - 1);

% Search BFS
queue = zeros(length(DT_histograms(1,:)), 1);
queue_heights = zeros(length(DT_histograms(1,:)), 1);
head = 1;
tail = 2;
queue(1) = 1;
queue_heights(1) = 1;
last_height = 1; cur_height = 1;
while (head ~= tail)
    cur_node = queue(head);
    last_height = cur_height;
    cur_height = queue_heights(head);
    head = head + 1;
    
    if (cur_height ~= last_height)
        fprintf('\n\n');
    end
    
    % Print the current node
    half_length = 8;
    if (cur_height ~= DT_height)
        num_half_lengths = 2^(DT_height - cur_height) - 1;
        for n = 1:num_half_lengths
            for m = 1:half_length
                fprintf(' ');
            end
        end
    end
    if (DT_histograms(1,cur_node) ~= -1)
        fprintf('[%+.3f  %+.3f]', DT_histograms(1,cur_node), DT_histograms(2,cur_node));
    else
        for n = 1:2
            for m = 1:half_length
                fprintf(' ');
            end
        end
    end
    if (cur_height ~= DT_height)
        num_half_lengths = 2^(DT_height - cur_height) - 1;
        for n = 1:num_half_lengths
            for m = 1:half_length
                fprintf(' ');
            end
        end
    end
    
    % If we haven't finished, then put our children on the back of the
    % queue
    if (cur_height ~= DT_height)
        queue(tail) = getLChild(cur_node);
        queue_heights(tail) = cur_height + 1;
        tail = tail + 1;
        queue(tail) = getRChild(cur_node);
        queue_heights(tail) = cur_height + 1;
        tail = tail + 1;
    end
    
end
fprintf('\n');


end

