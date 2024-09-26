function HH = FuncMapConst3D(Map)

Size_i = Map.Size_i;
Size_j = Map.Size_j;
Size_h = Map.Size_h;

ID1 = [];
ID2 = [];
Val = [];

nCnt = 0;


for i = 1:Size_i
    for j = 1:Size_j
        for h = 1:Size_h

            ij0 = Size_i*Size_j*(h-1) + Size_j*(i-1)+j;
            ij1 = Size_i*Size_j*(h-1) + Size_j*(i-1)+j+1;
            ij2 = Size_i*Size_j*(h-1) + Size_j*i+j;
            ij3 = Size_i*Size_j*h + Size_j*(i-1)+j;

            if j+1<=Size_j
                nCnt = nCnt+1;
                ID1 = [ID1;nCnt;nCnt];
                ID2 = [ID2;ij0;ij1];
                Val = [Val;1;-1];
            end
        
            if i+1<=Size_i  
                nCnt = nCnt+1;
                ID1 = [ID1;nCnt;nCnt];
                ID2 = [ID2;ij0;ij2];
                Val = [Val;1;-1];
            end
           
            if h+1<=Size_h  
                nCnt = nCnt+1;
                ID1 = [ID1;nCnt;nCnt];
                ID2 = [ID2;ij0;ij3];
                Val = [Val;1;-1];
            end


        end
    end    
end

% ID1 = single(ID1);
J = sparse(ID1,ID2,Val);
HH = J'*J;

end