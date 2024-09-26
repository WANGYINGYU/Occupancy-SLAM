% ===============================================================================================
% Linear SLAM: A Linear Solution to the Pose Feature and Pose Graph SLAM based on Submap Joining 
% Version: 1.0
% ===============================================================================================
% 
% Copyright (C) 2013 Liang Zhao, Shoudong Huang and Gamini Dissanayake
% University of Technology, Sydney, Australia
% 
% Authors:  Liang Zhao         -- Liang.Zhao-1@uts.edu.au 
%           Shoudong Huang     -- Shoudong.Huang@uts.edu.au
%           Gamini Dissanayake -- Gamini.Dissanayake@uts.edu.au
% 
%           Centre for Autonomous Systems
%           Faculty of Engineering and Information Technology
%           University of Technology, Sydney
%           NSW 2007, Australia
% 
% License
% 
% Linear SLAM by Liang Zhao, Shoudong Huang, Gamini Dissanayake is licensed under a 
% Creative Commons Attribution-NonCommercial-ShareAlike 3.0 Unported License.
% 
% Please contact Liang Zhao {Liang.Zhao-1@uts.edu.au} if you have any questions/comments about the code.
% 
%%
function RMatrix = RMatrixYPR22(Alpha,Beta,Gamma)

Rx = [1 0 0;0 cos(Gamma) sin(Gamma);0 -sin(Gamma) cos(Gamma)];
Ry = [cos(Beta) 0 -sin(Beta);0 1 0;sin(Beta) 0 cos(Beta)];
Rz = [cos(Alpha) sin(Alpha) 0;-sin(Alpha) cos(Alpha) 0;0 0 1];
RMatrix = Rx*Ry*Rz;

end
