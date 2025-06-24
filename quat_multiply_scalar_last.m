function q_out = quat_multiply_scalar_last(q1, q2)
    % Quaternion multiplication for scalar-last quaternions (4x1 column vectors)
    % Input:  q1 = [x1; y1; z1; w1] (4x1), q2 = [x2; y2; z2; w2] (4x1)
    % Output: q_out = q1 ⊗ q2 (4x1), scalar-last (Hamilton product)
    % Note: w1 and w2 are the scalar (real) parts
    
    % Ensure inputs are column vectors
    q1 = q1(:);
    q2 = q2(:);
    
    % Extract vector and scalar parts
    v1 = q1(1:3);  w1 = q1(4);
    v2 = q2(1:3);  w2 = q2(4);
    
    % Compute Hamilton product
    v_out = w1*v2 + w2*v1 + cross(v1, v2);
    w_out = w1*w2 - dot(v1, v2);
    
    % Combine and normalize (unit quaternion)
    q_out = [v_out; w_out];
    q_out = q_out / norm(q_out);  % Normalization
end