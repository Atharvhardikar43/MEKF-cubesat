function [q_plus, b_plus, P_plus] = mekf_update(q_prev, b_pos, P_prev, q_star, gyro_meas, R, Q, G, dt)
    % MEKF update step for star tracker (quaternion output) and gyro
    %
    % Inputs:
    %   q_prev    - Previous quaternion estimate (4x1)
    %   b_prev    - Previous gyro bias estimate (3x1)
    %   P_prev    - Previous covariance matrix (6x6)
    %   q_star    - Star tracker quaternion measurement (4x1)
    %   gyro_meas - Gyro measurement (3x1)
    %   R_star    - Star tracker attitude noise covariance (3x3)
    %   Q         - Process noise covariance (6x6)
    %   dt        - Time step
    %
    % Outputs:
    %   q_plus    - Updated quaternion estimate (4x1)
    %   b_plus    - Updated gyro bias estimate (3x1)
    %   P_plus    - Updated covariance matrix (6x6)
    %   w_est     - Estimated angular velocity (3x1)

    % Gyro bias correction
    w_pos = gyro_meas - b_pos*3.14/180;
    
    % % Attitude update
    % % Quaternion propagation (using gyro)
    % Omega = [0, -w_est(1), -w_est(2), -w_est(3);
    %          w_est(1), 0, w_est(3), -w_est(2);
    %          w_est(2), -w_est(3), 0, w_est(1);
    %          w_est(3), w_est(2), -w_est(1), 0];
    % q_pred = q_prev + 0.5 * Omega * q_prev * dt;
    % q_pred = q_pred/norm(q_pred);  % Normalize

    % State transition matrix (F) and noise input matrix (G)
    [F, H] = calc_Jacobians(w_pos, dt);
    % Predict covariance 
    P_prev = F * P_prev * F' + G * Q * G;

    % Bias 
    b_prev = b_pos;

    % Exp meas
    q_e = quat_error(q_prev, q_star);
    y = q_e(1:3);
    
    % Gain
    K = P_prev * H' / (H * P_prev * H' + R);
    
    % Update
    update = K * y;

    da = update(1:3);
    db = update(4:6); 
    %q_plus = quat_multiply(-da, q_prev);
    q_plus = quat_multiply(-da, q_star);
    q_plus = q_plus / norm(q_plus);
    b_plus = b_prev + db;
    
    P_plus = (eye(6) - K * H) * P_prev;
end