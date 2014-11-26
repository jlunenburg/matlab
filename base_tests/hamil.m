function out = hamil(a, b);

% Based on en.wikipedia.org/wiki/Quaternion
% But convention is xyzw
% Assumes quaternions are given [x,y,z,w];

out = zeros(4,1);
out(1) = a(4)*b(1) + a(1)*b(4) + a(2)*b(3) - a(3)*b(2);
%out(1) = a1b2+b1a2+c1d2-d1c2

out(2) = a(4)*b(2) - a(1)*b(3) + a(2)*b(4) + a(3)*b(1);
%out(2) = a1c2-b1d2+c1a2+d1b2

out(3) = a(4)*b(3) + a(1)*b(2) - a(2)*b(1) + a(3)*b(4);
%out(3) = a1d2+b1c2-c1b2+d1a2

out(4) = a(4)*b(4) - a(1)*b(1) - a(2)*b(2) - a(3)*b(3);
%out(4) = a1a2 -b1b2 -c1c2 -d1d2
