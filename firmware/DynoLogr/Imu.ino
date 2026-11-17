// ─── IMU CALC ─────────────────────────────────────────────────
void updateIMU() {
    sensors_event_t accel, gyro, temp;
    mpu.getEvent(&accel, &gyro, &temp);

    unsigned long now = millis();
    float dt = (now - lastIMUTime) / 1000.0;
    lastIMUTime = now;

    float ax = accel.acceleration.x - ax_off;
    float ay = accel.acceleration.y - ay_off;
    float az = accel.acceleration.z - az_off;
    gx_d = gyro.gyro.x - gx_off;
    gy_d = gyro.gyro.y - gy_off;
    gz_d = gyro.gyro.z - gz_off;

    // ── Raw (accel-only) angles ──────────────────────
    raw_ax = ax; raw_ay = ay; raw_az = az;
    raw_gx = gx_d; raw_gy = gy_d; raw_gz = gz_d;
    raw_pitch = atan2(ay, az);
    raw_roll  = atan2(-ax, sqrt(ay*ay + az*az));
    raw_yaw   = yaw;  // yaw can't be derived from accel alone; share integrated value
    raw_magnitude = sqrt(ax*ax + ay*ay + az*az) / 9.81;

    // ── Filtered (complementary filter) angles ───────
    float pitch_accel = atan2(ay, az);
    float roll_accel  = atan2(-ax, sqrt(ay*ay + az*az));

    pitch = ALPHA*(pitch + gx_d*dt) + (1.0-ALPHA)*pitch_accel;
    roll  = ALPHA*(roll  + gy_d*dt) + (1.0-ALPHA)*roll_accel;
    yaw  += gz_d * dt;

    float grav_x = -sin(roll);
    float grav_y =  sin(pitch)*cos(roll);
    float grav_z =  cos(pitch)*cos(roll);

    lx_g = (ax - grav_x*9.81) / 9.81;
    ly_g = (ay - grav_y*9.81) / 9.81;
    lz_g = (az - grav_z*9.81) / 9.81;

    float db = 0.05;
    if (abs(lx_g) < db) lx_g = 0;
    if (abs(ly_g) < db) ly_g = 0;
    if (abs(lz_g) < db) lz_g = 0;

    magnitude = sqrt(lx_g*lx_g + ly_g*ly_g + lz_g*lz_g);

    // ── Quaternion from filtered Euler (ZYX order) ───
    float cy = cos(yaw   * 0.5), sy = sin(yaw   * 0.5);
    float cp = cos(pitch * 0.5), sp = sin(pitch * 0.5);
    float cr = cos(roll  * 0.5), sr = sin(roll  * 0.5);
    qw = cr*cp*cy + sr*sp*sy;
    qx = sr*cp*cy - cr*sp*sy;
    qy = cr*sp*cy + sr*cp*sy;
    qz = cr*cp*sy - sr*sp*cy;
}
