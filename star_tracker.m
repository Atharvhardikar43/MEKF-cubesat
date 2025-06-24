function q_new = star_tracker(q, max_deg)
    % Inputs:
    %   q        - Original quaternion (4x1, scalar-last format)
    %   max_deg  - Maximum angle in degrees (e.g., 1 for 1°)
    % Output:
    %   q_new    - Perturbed quaternion within max_deg of q

    % Random unit axis
    axis = randn(3,1);
    axis = axis / norm(axis);

    % Small random angle between 0 and max_deg
    theta = deg2rad(rand * max_deg);

    % Small rotation quaternion (scalar-last)
    delta_q = [axis * sin(theta/2); cos(theta/2)];

    % Apply perturbation multiplicatively: q_new = delta_q ⊗ q
    q_new = quat_multiply_scalar_last(q', delta_q')';  % quatmultiply expects row vectors
    q_new = q_new' / norm(q_new);          % Normalize to unit norm
end
