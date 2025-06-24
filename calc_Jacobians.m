function [F, H] = calc_Jacobians(wpos, dt)
% CALC_JACOBIANS Computes Jacobians F (dynamics) and H (measurement)
%
% Inputs:
%   q_est - Estimated quaternion (4x1)
%   w_est - Estimated angular velocity (3x1)
%   J     - Inertia vector [Jx, Jy, Jz]
%   b     - Magnetic field vector in inertial frame (3x1)
%   s     - Sun vector in inertial frame (3x1)
%
% Outputs:
%   F     - State transition Jacobian (7x7)
%   H     - Measurement Jacobian (6x7)
    s = sin(norm(wpos)*dt);
    c = cos(norm(wpos)*dt);
    sks = skewsym(wpos);
    n = norm(wpos);
    p11 = eye(3) - sks*s/n + sks^2*(1-c)/(n^2);
    p12 = sks * (1-c)/(n^2) - eye(3)*dt - sks^2*(n*dt - s)/(n^3);
    p21 = zeros(3,3);
    p22 = eye(3);
    F = [p11 p12; p21 p22];

    H = [eye(3) zeros(3,3)];
end
