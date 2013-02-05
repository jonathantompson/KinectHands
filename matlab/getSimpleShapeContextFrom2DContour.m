function [ shape_context, samples ] = getSimpleShapeContextFrom2DContour( x_vals, y_vals, n_pts, n_bins )

if length(x_vals) ~= length(y_vals)
    error('getShapeContextFrom2DContour() - ERROR: x and y vals are not the same length!');
end

% First re-sample the contour at even length spacings along the contour:
% A) Calculate the total contour length:
dist = 0;
vec_length = length(x_vals);
contour_vectors = zeros(vec_length, 2);
contour_vectors_lengths = zeros(1,vec_length);
for i = 1:vec_length
    next_i = i + 1;
    if (next_i > vec_length)
        next_i = next_i - vec_length;
    end
    contour_vectors(i,1) = x_vals(next_i) - x_vals(i);
    contour_vectors(i,2) = y_vals(next_i) - y_vals(i);
    contour_vectors_lengths(i) = sqrt(contour_vectors(i,1)*contour_vectors(i,1) + ...
        contour_vectors(i,2)*contour_vectors(i,2));
    dist = dist + contour_vectors_lengths(i);
end

% B) Calculate the highest point in the image:
[y_max, y_max_ind] = max(y_vals);

% C) Now resample
samples = zeros(n_pts,2);
cur_sample_point = y_max_ind;
sample_spacing = dist / n_pts;
cur_sample_dist = 0;
% The first sample is just the maximum y-value (this orients our shape
% descriptor --> and is why it is so simple)
samples(1,1) = x_vals(cur_sample_point);
samples(1,2) = y_vals(cur_sample_point);
% Now find the rest
for m = 1:(n_pts-1)
    while (cur_sample_dist < sample_spacing * m)
        cur_sample_dist = cur_sample_dist + contour_vectors_lengths(cur_sample_point);
        prev_sample_point = cur_sample_point;
        cur_sample_point = cur_sample_point + 1;  % step to the next sample point
        if cur_sample_point > vec_length
            cur_sample_point = cur_sample_point - vec_length;
        end
    end
    % Now we went one sample too far, so subtract off the amount necessary
    remainder = cur_sample_dist - (sample_spacing * m);
    cur_sample_dist = sample_spacing * m;
    
    neg_vector = -contour_vectors(prev_sample_point,:);
    % normalize the negative vector
    cur_length = sqrt(neg_vector(1)*neg_vector(1) + neg_vector(2)*neg_vector(2));
    neg_vector = neg_vector ./ cur_length;
    % multiply by the remainder
    neg_vector = neg_vector .* remainder;
    % calculate the intermediate midpoint --> This is the next sample point
    samples(m+1,1) = x_vals(cur_sample_point) + neg_vector(1);
    samples(m+1,2) = y_vals(cur_sample_point) + neg_vector(2);
    % Now for the next iteration, update the x_val and y_val sample point
    % (as if it was the new value) and update the contour_vectors and their
    % lengths).
    %     x_vals(prev_sample_point) = samples(m+1,1);
    %     y_vals(prev_sample_point) = samples(m+1,2);
    contour_vectors(prev_sample_point,1) = x_vals(cur_sample_point) - samples(m+1,1);
    contour_vectors(prev_sample_point,2) = y_vals(cur_sample_point) - samples(m+1,2);
    contour_vectors_lengths(prev_sample_point) = remainder;
    cur_sample_point = prev_sample_point;
end

% Calculate the COM of the new sample points and subtract it off
% Also calculate the radius and the theta angle as we go
COM(1:2) = [0 0];
for i = 1:n_pts
    COM = COM + samples(i,:);
end
COM = COM ./ n_pts;
samples_shifted = samples;
radius = zeros(1,n_pts);
max_radius = -inf;
theta = zeros(1,n_pts);
for i = 1:n_pts
    samples_shifted(i,:) = samples(i,:) - COM;
    radius(i) = sqrt(samples_shifted(i,1)*samples_shifted(i,1) + samples_shifted(i,2)*samples_shifted(i,2));
    if (max_radius < radius(i))
        max_radius = radius(i);
    end
    theta(i) = (atan2(samples_shifted(i,2), samples_shifted(i,1)) + pi)/(2*pi);
end
% scale radius by the max radius
for i = 1:n_pts
    radius(i) = radius(i) / max_radius;
end
% Now radius is 0->1 and theta is 0->1

% Now put the R and theta values into the shape context array
shape_context = zeros(n_bins, n_bins);
for i = 1:n_pts
    cur_r = 1 + floor(radius(i) * (n_bins - 1));
    cur_theta = 1 + floor(theta(i) * (n_bins - 1));
    shape_context(cur_r, cur_theta) = shape_context(cur_r, cur_theta) + 1;
end

end