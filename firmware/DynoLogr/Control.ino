// ─── ENCODER ISR ──────────────────────────────────────────────
void IRAM_ATTR onEncoder() {
    bool a = digitalRead(EC1);
    bool b = digitalRead(EC2);
    if (a == b) { encoderCount++; encoderDir =  1; }
    else        { encoderCount--; encoderDir = -1; }
}

// ─── MOTOR ────────────────────────────────────────────────────
void stopMotor() {
    analogWrite(OT1, 0);
    analogWrite(OT2, 0);
    digitalWrite(EEP, LOW);
    digitalWrite(LED, LOW);
}

void driveMotor(int input, bool forward) {
    if (input <= DEADZONE) { stopMotor(); return; }
    int pwm = map(constrain(input, DEADZONE, ANALOG_MAX), DEADZONE, ANALOG_MAX, 0, 255);
    analogWrite(OT1, forward ? pwm : 0);
    analogWrite(OT2, forward ? 0 : pwm);
    digitalWrite(EEP, HIGH);
    digitalWrite(LED, HIGH);
}

void updateMotor(int throttle, int brake) {
    if      (throttle > DEADZONE) driveMotor(throttle, true);
    else if (brake    > DEADZONE) driveMotor(brake,    false);
    else                          stopMotor();
}

// ─── STEERING ─────────────────────────────────────────────────
void updateSteering(int axisX) {
    servoAngle = map(axisX, -508, 512, SERVO_CENTER+SERVO_MAX, SERVO_CENTER-SERVO_MAX);
    servoAngle = constrain(servoAngle, 0, 180);
    steeringServo.write(servoAngle);
}

// ─── ENCODER CALC ─────────────────────────────────────────────
void updateEncoder() {
    unsigned long now = millis();
    unsigned long dt  = now - lastEncTime;
    if (dt < 100) return;

    long  delta = encoderCount - lastCount;
    float revs  = (float)delta / PPR;
    rpm         = revs * (60000.0 / dt);
    speedCMS    = rpm * WHEEL_CIRC_CM / 60.0;
    distanceCM += revs * WHEEL_CIRC_CM;

    spokeAngle += revs * PI * 2.0;

    lastCount   = encoderCount;
    lastEncTime = now;
}
