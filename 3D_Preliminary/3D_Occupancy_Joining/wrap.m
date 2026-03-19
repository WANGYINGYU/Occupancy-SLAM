%
% function nu = wrap(alpha);
%
% 
% limits angles to -pi +pi
%
%

function nu = wrap(alpha)

clear nu;
nu = alpha;

	while (nu > pi)
		nu = nu - 2 * pi;
	end;

	while (nu < -pi)
		nu = nu + 2 * pi;
	end;
