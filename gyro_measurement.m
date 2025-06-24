function gyro = gyro_measurement(w, noise_level, gyro_drift)
    gyro = w + norm(w)*noise_level * randn(size(w))/100 + gyro_drift*3.14/180;
end