function [sampled_points,val] = FuncEqualDistanceSample3DLines(points,Param)
SampleDistance = Param.LocalScale;
ValOddHit = Param.ValOddHit;
ValOddFree = Param.ValOddFree;

    % Check the input dimensions
    [num_points, dim] = size(points);
    if dim ~= 3
        error('Input points must be an N x 3 matrix.');
    end
    
    % Initialize the array to hold sampled points
    % sampled_points = [];
    % val = [];

    cell_sampled_points = cell(1,num_points);
    cell_val = cell(1,num_points);

    % Define the origin
    origin = [0, 0, 0];

    % Loop through each point to create a line segment from the origin
    parfor i = 1:num_points
        % Get the end point of the current line segment
        p = points(i, :);
        
        num_samples = fix(sqrt(p(1)^2+p(2)^2+p(3)^2) / SampleDistance)+1;
        % Generate linearly spaced interpolation parameters
        t = linspace(1/num_samples, 1, num_samples)';

        diff = t(end) - t(end-1);
        t = [t;1+diff;1+2*diff;1+3*diff;1+4*diff;1+5*diff];

        % t(end) = 1;
        
        % Compute the sampled points on the line segment
        segment_points = (1-t) * origin + t * p;
        val_i = repmat(ValOddFree,num_samples+5,1);

        val_i(end-5) = ValOddHit;
        val_i(end-4) = ValOddHit/2;
        val_i(end-3) = ValOddHit/4;
        val_i(end-2) = ValOddHit/8;
        val_i(end-1) = ValOddHit/16;
        val_i(end) = ValOddHit/32;
        
        if length(val_i)>7
            val_i(end-6) = (ValOddHit+ValOddFree)/2;
        end
        if length(val_i)>8
            val_i(end-7) = ((ValOddHit+ValOddFree)/2 + ValOddFree)/2;
        end
        if length(val_i)>9
            val_i(end-8) = (((ValOddHit+ValOddFree)/2 + ValOddFree)/2 + ValOddFree)/2;
        end
        if length(val_i)>10
            val_i(end-9) = ((((ValOddHit+ValOddFree)/2 + ValOddFree)/2 + ValOddFree)/2+ValOddFree)/2;
        end
        
        % Append the sampled points to the list
        % sampled_points = [sampled_points; segment_points];
        % val = [val;val_i];
        cell_sampled_points{i} = segment_points;
        cell_val{i} = val_i;
    end
    sampled_points = vertcat(cell_sampled_points{:});
    val = vertcat(cell_val{:});
end

