function [roll,pitch,yaw] = getRPY(quat);

% Returns roll, pitch, yaw angles

roll  = -pi-atan2( 2*(quat(1)*quat(4) + quat(2)*quat(3)), 1-2*(quat(3)*quat(3) + quat(4)*quat(4)) );
pitch = -asin( 2 * (quat(1)*quat(3) -quat(4)*quat(2)) );
yaw   = -pi+atan2( 2*(quat(1)*quat(2) + quat(3)*quat(4)), 1-2*(quat(2)*quat(2) + quat(3)*quat(3)) );