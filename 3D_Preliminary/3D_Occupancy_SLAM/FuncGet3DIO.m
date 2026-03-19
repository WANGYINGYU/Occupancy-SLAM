function IO = FuncGet3DIO(ErrorO,Sigma_X,Sigma_Y,Sigma_Z,Sigma_r,Sigma_p,Sigma_y)


Sigma = 1./[Sigma_X,Sigma_Y,Sigma_Z,Sigma_r,Sigma_p,Sigma_y];

nO = length(ErrorO);

NumPose = nO/6;

Sigma_Rep = repmat(Sigma,NumPose,1);
AllSigma = reshape(Sigma_Rep',[],1);

IO = sparse(1:nO,1:nO,AllSigma.^2);

end