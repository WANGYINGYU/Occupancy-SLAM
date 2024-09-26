function Map = FuncSmoothOGMbyNDT(Map,NDTMap,Scale,Origin,ZoomScale)

Size_i = Map.Size_i;
Size_j = Map.Size_j;
Size_h = Map.Size_h;

% ZoomScale = 5;

NDTSize_i = Size_i/5;
NDTSize_j = Size_j/5;
NDTSize_h = Size_h/5;

for i=1:size(NDTMap.Grid,2)

    if isempty(NDTMap.Grid{i}) ==0
        Ind = i;
        % Calculate Corresponding Grid in OGM
        [sub1,sub2,sub3] = ind2sub([NDTSize_i,NDTSize_j,NDTSize_h],Ind);
        sub = [sub1,sub2,sub3];
        RowRange = (sub(1)-1)*ZoomScale+1:sub(1)*ZoomScale;
        ColRange = (sub(2)-1)*ZoomScale+1:sub(2)*ZoomScale;
        HeightRange = (sub(3)-1)*ZoomScale+1:sub(3)*ZoomScale;
        
        [X, Y, Z] = meshgrid(RowRange, ColRange, HeightRange);
        OGMSub = [X(:), Y(:), Z(:)];
        OGMInd = (Size_i*Size_j*(Z(:)-1)) + (X(:)-1)*Size_i + Y(:);

        % convert OGMSub to NDTSub
        NDTSub = (OGMSub-1+0.5)./ZoomScale+1; % center point
        
        % convert NDTSub to world coordinate
        WorldCoord = (NDTSub-1)*(Scale*ZoomScale)+Origin';
        % mean and covariance of NDT
        mean = NDTMap.Grid{i}.mean;
        covariance = NDTMap.Grid{i}.covariance;

        if ~isequal(covariance, covariance')
            covariance = (covariance + covariance') / 2;
        end

        % if any(eig(covariance) <= 0)
        %     min_eig = min(eig(covariance));
        %     offset = abs(min_eig) + eps; 
        %     covariance = covariance + offset * eye(size(covariance));
        % end

        min_eig = min(eig(covariance));
        if min_eig <= 1e-6
            % fprintf("cnmd")
            eigen = eig(covariance);
            neg_eigen = find(eigen<=1e-6);
            large_neg = max(abs(neg_eigen));
            % 计算对角偏移
            offset = abs(large_neg) + 1e-6; % 添加一个小的正数偏移，例如 1e-6
            % 对角线加上偏移，确保所有特征值都是正的
            covariance_post = covariance + offset * eye(size(covariance));
            covariance = covariance_post;
        end

        %calculate pdf
        pdf = mvnpdf(WorldCoord,mean,covariance);
        normal_pdf = pdf./max(pdf);

        % Get occupancy value
        Occupancy = Map.Grid(OGMInd);
        N = Map.N(OGMInd);
        WeightOcc = sum(Occupancy.*normal_pdf)*normal_pdf;
        WeightN = sum(N.*normal_pdf)*normal_pdf;
        Map.Grid(OGMInd) = WeightOcc;
        Map.N(OGMInd) = WeightN;

    end
end    


end