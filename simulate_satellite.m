function simulate_satellite()
    % SIMULATE_SATELLITE Simulates satellite attitude dynamics and EKF-based estimation

    %% Simulation setup
    dt = 1;                 % Time step (s)
    t_end = 60;            % Total simulation time (s)
    steps = t_end / dt;     % Number of simulation steps

    % Define reference epoch (J2000) and current time in UTC
    epoch_ref = datetime(2000, 1, 1, 0, 0, 0, 'TimeZone', 'UTC');
    now_utc = datetime('now', 'TimeZone', 'UTC');
    utc0 = seconds(now_utc - epoch_ref);  % Time since epoch in seconds

    %% Initial true states
    r = [6771e3; 0; 0];                      % Initial position in ECEF (m)
    v = [0; 7.67e3; 0];                      % Initial velocity in ECEF (m/s)
    q = [0; 0; 0; 1];                        % Initial true attitude (quaternion, ECI to body)
    w = deg2rad([1; 2; 1]);            % Initial angular velocity (rad/s)
    J = 0.0001 * [10, 11, 12];               % Moment of inertia (kg·m²)
    w_est = w;                               % Initial angular velocity estimate
    mu = 3.986e14;                           % Gravitational parameter (m³/s²)
    STA_accuracy = 0.0;                        % Star Tracker accuracy (deg)
    gyro_noise = 1;                          % Gyro noise (deg)
    gyro_drift = 0.01;                        % Gyro drift (deg/sec)

    %% EKF initialization
    q_est = [0.0; 0.1; 0; 0.9];                % Initial estimated quaternion
    q_est = q_est / norm(q_est);            % Normalize quaternion
    P = 0.01 * eye(6);                       % Initial state covariance
    Q = 1e-6 * eye(6);                       % Process noise covariance
    R = 1e-2 * eye(3);                       % Measurement noise covariance
    G = 1e-2 * eye(6);                       % Gyro
    %% Logging arrays
    q_true_log = zeros(4, steps);           % Log of true quaternions
    q_est_log = zeros(4, steps);            % Log of estimated quaternions
    t_log = now_utc;                        % Preallocate time log
    ang_error = zeros(steps, 1);            % Log of angle errors
  
    %% Main simulation loop
    for k = 1:steps
        % Current time stamp
        t_utc = now_utc + seconds((k - 1) * dt);

        % Log current quaternions
        q_true_log(:, k) = q;
        q_est_log(:, k) = q_est;

        % Compute attitude error angle between true and estimated quaternions
        dot_product = dot(q, q_est);
        angle_rad = 2 * acos(dot_product);
        ang_error(k) = rad2deg(angle_rad);  % Convert to degrees

        % Step 1: Propagate true dynamics
        [r, v] = propagate_orbit(r, v, dt, mu, w);       % Propagate orbit
        [q, w] = propagate_attitude(q, w, dt);           % Propagate attitude

        % Step 2: Compute noisy sensor vectors in body frame
        gyro = gyro_measurement(w, gyro_noise, gyro_drift);
        STA = star_tracker(q, STA_accuracy);

        % Step 3: MEKF attitude estimation update
        [q_est, gyro_drift, P] = mekf_update(q_est, gyro_drift, P, STA, gyro, R, Q, G, dt);
        % Step 4: Log current time
        t_log(k) = t_utc;
    end

    %% Step 6: Export results to workspace and plot
    assignin('base', 'q_true', q_true_log);
    assignin('base', 'q_est', q_est_log);
    plot_results(t_log, q_true_log, q_est_log, ang_error);
end
