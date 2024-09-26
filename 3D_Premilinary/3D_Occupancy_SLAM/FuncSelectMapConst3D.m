function [HH,Param] = FuncSelectMapConst3D(SelectMap,Param)

SelectCoords = Param.SelectCoords;

Size_i = SelectMap.Size_i;
Size_j = SelectMap.Size_j;
% Size_h = SelectMap.Size_h;

Row = SelectCoords(:,1);
Col = SelectCoords(:,2);
Height = SelectCoords(:,3);

% Sort the selected coordinates to variable order
VarId = Size_i*Size_j*(Height-1) + Size_j*(Row-1)+Col;
SortId = sort(VarId);

Param.SortId = SortId; % variable order, from smaller to larger

% Change back to coordinates
MatHeight = ceil(SortId./(Size_j*Size_i));
MatRow = ceil((SortId - (MatHeight-1).* (Size_j*Size_i))/Size_j);
MatCol = SortId - (MatHeight-1).* (Size_j*Size_i) - (MatRow-1)*Size_j;

SelectCoords = [MatRow,MatCol,MatHeight]; % in variable order

Param.SelectCoords = SelectCoords;

% define the adjecent cells of x,y and z directions in the map(matrix)
RightId = MatCol+1; % y
BelowId = MatRow+1; % x
TopId = MatHeight+1; %z

CoordsRight = [MatRow,RightId,MatHeight];
CoordsBelow = [BelowId,MatCol,MatHeight];
CoordsTop = [MatRow,MatCol,TopId];

% if the x/y/z hand has variables
FindRight = ismember(CoordsRight,SelectCoords,'rows');
FindBelow = ismember(CoordsBelow,SelectCoords,'rows');
FindTop = ismember(CoordsTop,SelectCoords,'rows');

% CheckEdge = find((FindRight.*FindBelow.*FindTop)==1);

Num = sum(FindRight+FindBelow+FindTop);

% Num = length(CheckEdge);

ID1 = zeros(2*Num,1);
ID2 = zeros(2*Num,1);
Val = zeros(2*Num,1);


% ID1 = [];
% ID2 = [];
% Val = [];

AllSelVarId = 1:length(FindRight); % start from 1 to the number of select variables

% Deal with cells which right hand 
RightIncudeId = find(FindRight==1); % in original order
ij0 = AllSelVarId(RightIncudeId); % change to select order
ij1 = ij0+1; % the index of right hand is always plus 1

ID1(1:2*length(ij0),1) = [(1:length(ij0))';(1:length(ij0))'];
ID2(1:2*length(ij0),1) = [ij0';ij1'];
Val(1:2*length(ij0),1) = [ones(length(ij0),1);-ones(length(ij0),1)];

nCnt = 2*length(ij0);
nCntR = length(ij0);

% Deal with cells whch below hand 
BelowIncludedId = find(FindBelow==1); % in original order ij0

BelowCoordpPrev = CoordsBelow(BelowIncludedId,:); % the coordinates of these cells in original order

ij0 = AllSelVarId(BelowIncludedId); % in select order ij0

[check,ij2] = ismember(BelowCoordpPrev,SelectCoords,"rows"); % find the positions of below cells in the select variables

ID1(nCnt+1:nCnt+2*length(ij0),1) = [(nCntR+1:nCntR+length(ij0))';(nCntR+1:nCntR+length(ij0))'];
ID2(nCnt+1:nCnt+2*length(ij0),1) = [ij0';ij2];
Val(nCnt+1:nCnt+2*length(ij0),1) = [ones(length(ij0),1);-ones(length(ij0),1)];

nCnt = nCnt+2*length(ij0);
nCntR = nCntR+length(ij0);

% Deal with cells whch top (z-axis) hand included in
TopIncludedId = find(FindTop==1);
TopCoordpPrev = CoordsTop(TopIncludedId,:);
ij0 = AllSelVarId(TopIncludedId);
[check,ij3] = ismember(TopCoordpPrev,SelectCoords,"rows");
ID1(nCnt+1:nCnt+2*length(ij0),1) = [(nCntR+1:nCntR+length(ij0))';(nCntR+1:nCntR+length(ij0))'];
ID2(nCnt+1:nCnt+2*length(ij0),1) = [ij0';ij3];
Val(nCnt+1:nCnt+2*length(ij0),1) = [ones(length(ij0),1);-ones(length(ij0),1)];




% nCnt = 0;



% for i = 1:Num
% 
%     Coori = SelectCoords(i,:);
% 
%     ij0 = i;
% 
%     if FindRight(i)==1
%         ij1 = i+1;
%         nCnt = nCnt+1;
%         ID1(2*nCnt-1:2*nCnt) = [nCnt;nCnt];
%         ID2(2*nCnt-1:2*nCnt) = [ij0;ij1];
%         Val(2*nCnt-1:2*nCnt) = [1;-1];
%     end
% 
%     if FindBelow(i)==1
%         CoorBelowi = [Coori(1)+1,Coori(2),Coori(3)];
%         Checkp = CoorBelowi==SelectCoords;
%         Checkpp = Checkp(:,1).*Checkp(:,2).*Checkp(:,3);
%         ij2 = find(Checkpp==1);
%         nCnt = nCnt+1;
%         ID1(2*nCnt-1:2*nCnt) = [nCnt;nCnt];
%         ID2(2*nCnt-1:2*nCnt) = [ij0;ij2];
%         Val(2*nCnt-1:2*nCnt) = [1;-1];
%     end
% 
%     if FindTop(i)==1
%         CoorTopi = [Coori(1),Coori(2),Coori(3)+1];
%         Checkp = CoorTopi==SelectCoords;
%         Checkpp = Checkp(:,1).*Checkp(:,2).*Checkp(:,3);
%         ij3 = find(Checkpp==1);
%         nCnt = nCnt+1;
%         ID1(2*nCnt-1:2*nCnt) = [nCnt;nCnt];
%         ID2(2*nCnt-1:2*nCnt) = [ij0;ij3];
%         Val(2*nCnt-1:2*nCnt) = [1;-1];
%     end 
% end

J = sparse(ID1,ID2,Val);
HH = J'*J;

end