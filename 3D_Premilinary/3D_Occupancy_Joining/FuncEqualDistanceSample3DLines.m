function [sampled_points,val] = FuncEqualDistanceSample3DLines(points,Param)
SampleDistance = Param.SampleDistance;
ValOddHit = Param.ValOddHit;
ValOddFree = Param.ValOddFree;
    % Check the input dimensions
    [num_points, dim] = size(points);
    if dim ~= 3
        error('Input points must be an N x 3 matrix.');
    end
    cell_sampled_points = cell(1,num_points);
    cell_val = cell(1,num_points);

    % Define the origin

    % Loop through each point to create a line segment from the origin
    parfor i = 1:num_points
        % Get the end point of the current line segment
        p = points(i, :);
        distance = norm(p);   
        if distance > 40
            direction = p / distance;    
            origin = direction * 40;
        else
            origin = [0, 0, 0];
        end    
        num_samples = fix(sqrt(p(1)^2+p(2)^2+p(3)^2) / SampleDistance)+1;
        % Generate linearly spaced interpolation parameters
        t = linspace(1/num_samples, 1, num_samples)';        
        % Compute the sampled points on the line segment
        segment_points = (1-t) * origin + t * p;
        val_i = repmat(ValOddFree,num_samples,1);
        val_i(end) = ValOddHit;
        % Append the sampled points to the list
        cell_sampled_points{i} = segment_points;
        cell_val{i} = val_i;
    end
    sampled_points = vertcat(cell_sampled_points{:});
    val = vertcat(cell_val{:});
end

