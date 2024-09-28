function SparseId = FuncCheckGridSparse(Id,SizeMap,SizeKernel,MapN)
    [I,J,H] = ind2sub(SizeMap,Id);       
    Offset = -SizeKernel:SizeKernel;
    [OffsetI, OffsetJ, OffsetH] = ndgrid(Offset, Offset, Offset);
    OffsetI = OffsetI(:);
    OffsetJ = OffsetJ(:);
    OffsetH = OffsetH(:);

 
    AllI = reshape((I + OffsetI')', [], 1);
    AllJ = reshape((J + OffsetJ')', [], 1);
    AllH = reshape((H + OffsetH')', [], 1);


    allCombinations = [AllI, AllJ, AllH];

    % ReshapeAll = reshape(allCombinations,16741,[]);

    % IdOutI = find(AllI>SizeMap(1));
    % IdOutJ = find(AllJ>SizeMap(2));
    % IdOutH = find(AllH>SizeMap(3));
    % 
    % IdOutII = find(AllI<=0);
    % IdOutJJ = find(AllJ<=0);
    % IdOutHH = find(AllH<=0);
    % 
    % IdOut = [IdOutI;IdOutJ;IdOutH;IdOutII;IdOutJJ;IdOutHH];

    % AllI(IdOut) = [];
    % AllJ(IdOut) = [];
    % AllH(IdOut) = [];

    AllId = (AllH-1)*SizeMap(2)*SizeMap(1) + (AllJ-1)* SizeMap(1) + AllI;

    ReshapeAllId = reshape(AllId',[],size(I,1))';
    
    SparseId = [];
    for i=1:size(I,1)
        AllIdi = ReshapeAllId(i,:);
        IdOut1 = find(AllIdi<=0);
        IdOut2 = find(AllIdi>SizeMap(1)*SizeMap(2)*SizeMap(3));
        IdOut = [IdOut1;IdOut2];
        AllIdi(IdOut) = [];
        AllIdi = unique(AllIdi);
        CheckN = MapN(AllIdi);
        SumCheck = sum(CheckN);
        if SumCheck<4
            SparseId = [SparseId;i];
        end
    end


 
    

end