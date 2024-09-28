function [LowGrid,LowN] = FuncCreateLowFromHigh(Map)
Size_i = Map.Size_i;
Size_j = Map.Size_j;
Size_h = Map.Size_h;

Grid = Map.Grid;
N = Map.N;

LowRate = 2;

LowGrid = zeros(Size_i/2,Size_j/2,Size_h/2);
LowN = zeros(Size_i/2,Size_j/2,Size_h/2);


for i = 1:Size_i/2
    for j = 1:Size_j/2
        for k = 1:Size_h/2
            i_range = (i-1)*LowRate + 1:i*LowRate;
            j_range = (j-1)*LowRate + 1:j*LowRate;
            k_range = (k-1)*LowRate + 1:k*LowRate;
            
            LowGrid(i, j, k) = mean(Grid(i_range, j_range, k_range), 'all');
            LowN(i, j, k) = mean(N(i_range, j_range, k_range), 'all');
        end
    end
end


end