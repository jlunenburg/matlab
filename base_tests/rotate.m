function out = rotate(in, q_rot);

q_in = [in(1);in(2);in(3);0.0]; % Input vector with additional zero
q_con = [-q_rot(1);-q_rot(2);-q_rot(3);q_rot(4)];

q_out = hamil(q_rot, q_in);
q_out = hamil(q_out, q_con);

if abs(q_out(4)) > 0.0000001
    fprintf('q_4 = %f, I guess there is something wrong\n',q_out(4))
end

out = q_out(1:3);