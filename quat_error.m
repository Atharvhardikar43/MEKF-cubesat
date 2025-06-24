function delta_q = quat_error(q_prev, q_star)
    % Ensure quaternions are normalized (optional but recommended)
    q_prev = q_prev / norm(q_prev);
    q_star = q_star / norm(q_star);

    % Compute the inverse of q_prev (unit quaternion => conjugate)
    q_prev_inv = [-q_prev(1:3); q_prev(4)];

    % Quaternion multiplication (delta_q = q_star ⊗ q_prev_inv)
    v1 = q_star(1:3);
    s1 = q_star(4);
    v2 = q_prev_inv(1:3);
    s2 = q_prev_inv(4);

    delta_v = s1 * v2 + s2 * v1 + cross(v1, v2);
    delta_s = s1 * s2 - dot(v1, v2);

    delta_q = [delta_v; delta_s];
    delta_q = delta_q / norm(delta_q);  % Normalize
end
