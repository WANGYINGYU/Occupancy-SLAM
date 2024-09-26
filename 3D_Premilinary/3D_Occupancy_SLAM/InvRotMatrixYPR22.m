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
function [Alpha,Beta,Gamma] = InvRotMatrixYPR22(K)

 Beta=atan2(-(K(1,3)),sqrt(K(1,1)^2+K(1,2)^2));
 if (cos(Beta)<1e-6)
     Alpha=0;
     Beta=pi/2;
     Gamma=atan2(K(1,2),K(2,2));
 else
     Alpha=atan2(K(1,2)/cos(Beta),K(1,1)/cos(Beta));
     Gamma=atan2(K(2,3)/cos(Beta),K(3,3)/cos(Beta));
 end


end
