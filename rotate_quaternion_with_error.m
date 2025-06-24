function q_new = rotate_quaternion_with_error(dtheta, q)
    % Apply small-angle error vector to quaternion (scalar-last)
    % dtheta : 3x1 vector (Gibbs/small-angle rotation vector)
    % q      : 4x1 quaternion (scalar-last)
    % q_new  : rotated quaternion (scalar-last)

    % Create small-angle quaternion (scalar-last)
    dq = [0.5 * dtheta; 1];
    dq = dq / norm(dq);  % Optional normalization

    % Multiply: q_new = dq ⊗ q
    q_new = quatmultiply_scalar_last(dq, q);
end

function q = quatmultiply_scalar_last(q1, q2)
    % Hamilton product for scalar-last quaternions
    % q1, q2 : 4x1 quaternions [x; y; z; w], with w = scalar

    v1 = q1(1:3); s1 = q1(4);
    v2 = q2(1:3); s2 = q2(4);

    v = s1 * v2 + s2 * v1 + cross(v1, v2);
    s = s1 * s2 - dot(v1, v2);

    q = [v; s];
    q = q / norm(q);  % Normalize to keep unit quaternion
end
