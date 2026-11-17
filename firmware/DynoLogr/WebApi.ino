// ─── STEERING ANGLE (degrees from center) ─────────────────────
// Returns -90..+90 where 0 = straight
float steeringAngleDeg() {
    return (float)(servoAngle - SERVO_CENTER);
}

// ─── HTTP: /data endpoint ─────────────────────────────────────
void handleData() {
    // Choose filtered or raw angles based on toggle
    float p = mpuRawMode ? raw_pitch * RAD_TO_DEG : pitch * RAD_TO_DEG;
    float r = mpuRawMode ? raw_roll  * RAD_TO_DEG : roll  * RAD_TO_DEG;
    float y = mpuRawMode ? raw_yaw   * RAD_TO_DEG : yaw   * RAD_TO_DEG;
    float mag = mpuRawMode ? raw_magnitude : magnitude;

    // For raw mode: report raw accel as g-forces instead of gravity-subtracted
    float lx = mpuRawMode ? raw_ax / 9.81 : lx_g;
    float ly = mpuRawMode ? raw_ay / 9.81 : ly_g;
    float lz = mpuRawMode ? raw_az / 9.81 : lz_g;

    StaticJsonDocument<512> doc;
    // IMU
    doc["pitch_deg"]     = p;
    doc["roll_deg"]      = r;
    doc["yaw_deg"]       = y;
    doc["lx_g"]          = lx;
    doc["ly_g"]          = ly;
    doc["lz_g"]          = lz;
    doc["gx_dps"]        = gx_d * RAD_TO_DEG;
    doc["gy_dps"]        = gy_d * RAD_TO_DEG;
    doc["gz_dps"]        = gz_d * RAD_TO_DEG;
    doc["magnitude"]     = mag;
    doc["mpu_filtered"]  = !mpuRawMode;   // true = filtered, false = raw

    // Quaternion (always from filtered for 3D render stability)
    doc["qw"] = qw;
    doc["qx"] = qx;
    doc["qy"] = qy;
    doc["qz"] = qz;

    // Steering
    doc["servo_angle"]    = servoAngle;          // raw 0..180
    doc["steering_deg"]   = steeringAngleDeg();  // -90..+90, 0=straight

    // Encoder / kinetics
    float vel_ms   = speedCMS / 100.0;
    float dist_m   = distanceCM / 100.0;
    float accel_ms2 = (rpm / 60.0 * WHEEL_CIRC_CM / 100.0);  // approx
    doc["rpm"]          = rpm;
    doc["velocity_ms"]  = vel_ms;
    doc["accel_ms2"]    = accel_ms2;
    doc["distance_m"]   = dist_m;
    doc["displacement_m"] = dist_m;   // same until tracked separately
    doc["ticks"]        = (long)encoderCount;
    doc["direction"]    = encoderDir >= 0 ? "FWD" : "REV";

    server.sendHeader("Access-Control-Allow-Origin", "*");
    String out;
    serializeJson(doc, out);
    server.send(200, "application/json", out);
}

// ─── HTTP: /reset endpoint ────────────────────────────────────
void handleReset() {
    encoderCount = 0; lastCount = 0;
    rpm = 0; speedCMS = 0; distanceCM = 0;
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "text/plain", "ok");
}

// ─── HTTP: /mpu_mode endpoint ─────────────────────────────────
// POST body: {"filtered":true}  or  {"filtered":false}
void handleMpuMode() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    if (server.method() == HTTP_OPTIONS) {
        server.sendHeader("Access-Control-Allow-Methods", "POST,GET,OPTIONS");
        server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
        server.send(204);
        return;
    }
    if (server.hasArg("plain")) {
        StaticJsonDocument<64> doc;
        if (deserializeJson(doc, server.arg("plain")) == DeserializationError::Ok) {
            mpuRawMode = !(doc["filtered"] | true);
        }
    }
    server.send(200, "text/plain", mpuRawMode ? "raw" : "filtered");
}
