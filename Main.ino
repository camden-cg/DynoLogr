#include <Bluepad32.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <math.h>

// ─── WIFI CREDENTIALS ─────────────────────────────────────────
// Change these to match your network
#define WIFI_SSID "***"
#define WIFI_PASS "***"

// ─── PINS ─────────────────────────────────────────────────────
#define OT1 14
#define OT2 27
#define EEP 12
#define LED 2
#define EC1 34
#define EC2 35
#define SIG 13
#define SDA 32
#define SCL 33

// ─── DISPLAY ──────────────────────────────────────────────────
#define SCREEN_WIDTH   128
#define SCREEN_HEIGHT  64
#define OLED_RESET     -1
#define SCREEN_ADDRESS 0x3C
#define DIVIDER_MPU    46
#define DIVIDER_ENC    52
#define CCX            ((DIVIDER_MPU + SCREEN_WIDTH) / 2)
#define CCY            34
#define ARROW_SCALE    16.0
#define ALPHA          0.98

// ─── SERVO ────────────────────────────────────────────────────
const int SERVO_CENTER = 90;
const int SERVO_MAX    = 90;
Servo steeringServo;
int   servoAngle = 90;

// ─── MOTOR / ENCODER ──────────────────────────────────────────
const int   DEADZONE      = 50;
const int   ANALOG_MAX    = 1023;
const int   PPR           = 140;
const float WHEEL_DIAM_CM = 50.0;
const float WHEEL_CIRC_CM = WHEEL_DIAM_CM * PI;

// ─── GLOBALS — CONTROLLER ─────────────────────────────────────
ControllerPtr myController = nullptr;

// ─── GLOBALS — ENCODER ────────────────────────────────────────
volatile long encoderCount = 0;
volatile int  encoderDir   = 0;
long          lastCount    = 0;
float         rpm          = 0;
float         speedCMS     = 0;
float         distanceCM   = 0;
unsigned long lastEncTime  = 0;
float         spokeAngle   = 0;

// ─── GLOBALS — IMU ────────────────────────────────────────────
float ax_off=0, ay_off=0, az_off=0;
float gx_off=0, gy_off=0, gz_off=0;

// Filtered (complementary filter) values
float pitch=0, roll=0, yaw=0;
float lx_g=0, ly_g=0, lz_g=0;
float gx_d=0, gy_d=0, gz_d=0;
float magnitude=0;

// Raw (unfiltered) values
float raw_pitch=0, raw_roll=0, raw_yaw=0;
float raw_ax=0, raw_ay=0, raw_az=0;
float raw_gx=0, raw_gy=0, raw_gz=0;
float raw_magnitude=0;

// Quaternion (from filtered angles)
float qw=1, qx=0, qy=0, qz=0;

unsigned long lastIMUTime = 0;

// ─── MPU FILTER TOGGLE ────────────────────────────────────────
// false = filtered (complementary), true = raw accel-only
bool mpuRawMode = false;

// ─── GLOBALS — UI ─────────────────────────────────────────────
enum Page { HOME, MPU_PAGE, ENC_PAGE };
Page currentPage   = HOME;
int  menuSelection = 0;
bool lastX=false, lastUp=false, lastDown=false, lastCircle=false;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_MPU6050 mpu;
WebServer server(80);

// ─── CIRCLE BITMAP (8x8) ──────────────────────────────────────
static const uint8_t PROGMEM circleBmp[] = {
    0b00111100,
    0b01000010,
    0b10000001,
    0b10000001,
    0b10000001,
    0b10000001,
    0b01000010,
    0b00111100
};

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

// ─── MPU PAGE ─────────────────────────────────────────────────
float cubeVertices[8][3] = {
    {-8,-8,-8},{8,-8,-8},{8,8,-8},{-8,8,-8},
    {-8,-8,8},{8,-8,8},{8,8,8},{-8,8,8}
};
int edges[12][2] = {
    {0,1},{1,2},{2,3},{3,0},
    {4,5},{5,6},{6,7},{7,4},
    {0,4},{1,5},{2,6},{3,7}
};

void rotatePoint(float x, float y, float z, float rx, float ry, float rz,
                 float &ox, float &oy, float &oz) {
    float y1 = y*cos(rx) - z*sin(rx);
    float z1 = y*sin(rx) + z*cos(rx);
    float x2 = x*cos(ry) + z1*sin(ry);
    float z2 = -x*sin(ry) + z1*cos(ry);
    ox = x2*cos(rz) - y1*sin(rz);
    oy = x2*sin(rz) + y1*cos(rz);
    oz = z2;
}

void drawCube(float rx, float ry, float rz) {
    float projected[8][2];
    for (int i = 0; i < 8; i++) {
        float ox, oy, oz;
        rotatePoint(cubeVertices[i][0], cubeVertices[i][1], cubeVertices[i][2],
                    rx, ry, rz, ox, oy, oz);
        float fov   = 28.0;
        float scale = fov / (fov + oz);
        projected[i][0] = ox*scale + CCX;
        projected[i][1] = oy*scale + CCY;
    }
    for (int i = 0; i < 12; i++) {
        int a = edges[i][0], b = edges[i][1];
        display.drawLine((int)projected[a][0], (int)projected[a][1],
                         (int)projected[b][0], (int)projected[b][1], SSD1306_WHITE);
    }
}

void drawForceArrow(float gx, float gy) {
    if (abs(gx) < 0.05 && abs(gy) < 0.05) return;
    int tx = constrain(CCX+(int)(gx*ARROW_SCALE), DIVIDER_MPU+2, SCREEN_WIDTH-2);
    int ty = constrain(CCY-(int)(gy*ARROW_SCALE), 10, SCREEN_HEIGHT-2);
    display.drawLine(CCX, CCY, tx, ty, SSD1306_WHITE);
    float angle = atan2(ty-CCY, tx-CCX);
    float wl=5.0, wa=0.45;
    display.drawLine(tx, ty, tx-(int)(wl*cos(angle-wa)), ty-(int)(wl*sin(angle-wa)), SSD1306_WHITE);
    display.drawLine(tx, ty, tx-(int)(wl*cos(angle+wa)), ty-(int)(wl*sin(angle+wa)), SSD1306_WHITE);
}

void printSigned(float val, int decimals) {
    if (val >= 0) display.print("+");
    display.print(val, decimals);
}

void drawMPUPage() {
    // Decide which values to show based on filter mode
    float dp = mpuRawMode ? raw_pitch * RAD_TO_DEG : pitch * RAD_TO_DEG;
    float dr = mpuRawMode ? raw_roll  * RAD_TO_DEG : roll  * RAD_TO_DEG;
    float dy = mpuRawMode ? raw_yaw   * RAD_TO_DEG : yaw   * RAD_TO_DEG;
    float dlx = mpuRawMode ? raw_ax / 9.81 : lx_g;
    float dly = mpuRawMode ? raw_ay / 9.81 : ly_g;
    float dlz = mpuRawMode ? raw_az / 9.81 : lz_g;
    float dmag = mpuRawMode ? raw_magnitude : magnitude;

    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);

    display.setCursor(0, 0);  display.print("Ax:"); printSigned(dlx, 1);
    display.setCursor(0, 9);  display.print("Ay:"); printSigned(dly, 1);
    display.setCursor(0, 18); display.print("Az:"); printSigned(dlz, 1);
    display.setCursor(0, 27); display.print("Gx:"); printSigned(gx_d, 1);
    display.setCursor(0, 36); display.print("Gy:"); printSigned(gy_d, 1);
    display.setCursor(0, 45); display.print("Gz:"); printSigned(gz_d, 1);

    // Filter mode indicator
    display.setCursor(0, 56);
    display.print(mpuRawMode ? "RAW" : "FLT");

    display.drawFastVLine(DIVIDER_MPU, 0, 64, SSD1306_WHITE);

    display.setCursor(DIVIDER_MPU+3, 0);
    display.print(dmag, 2); display.print("G");
    display.drawFastHLine(DIVIDER_MPU+1, 8, SCREEN_WIDTH-DIVIDER_MPU-1, SSD1306_WHITE);

    drawCube(dp * DEG_TO_RAD, dr * DEG_TO_RAD, dy * DEG_TO_RAD);
    drawForceArrow(dlx, dly);
    display.fillCircle(CCX, CCY, 2, SSD1306_WHITE);
}

// ─── ENCODER PAGE ─────────────────────────────────────────────
void drawWheelWithSpoke(int cx, int cy, int rx, int ry, float angle) {
    for (int i = 0; i < 16; i++) {
        float a1 = (i    / 16.0) * 2 * PI;
        float a2 = ((i+1) / 16.0) * 2 * PI;
        display.drawLine(
            cx + (int)(rx * cos(a1)), cy + (int)(ry * sin(a1)),
            cx + (int)(rx * cos(a2)), cy + (int)(ry * sin(a2)),
            SSD1306_WHITE);
    }
    int sx1 = (int)(rx * 0.7 * cos(angle));
    int sy1 = (int)(ry * 0.7 * sin(angle));
    int sx2 = (int)(rx * 0.7 * cos(angle + PI/2));
    int sy2 = (int)(ry * 0.7 * sin(angle + PI/2));
    display.drawLine(cx-sx1, cy-sy1, cx+sx1, cy+sy1, SSD1306_WHITE);
    display.drawLine(cx-sx2, cy-sy2, cx+sx2, cy+sy2, SSD1306_WHITE);
}

void drawCarSide(int cx, int cy, float tiltAngle, int steerAngleDeg) {
    float s = sin(tiltAngle);
    float c = cos(tiltAngle);

    int bodyW = 38;
    int bodyH = 10;
    int wheelR = 6;
    int wOffX = bodyW/2 - 6;
    int wOffY = bodyH/2;

    int fwx = cx + (int)( wOffX*c - wOffY*s);
    int fwy = cy + (int)( wOffX*s + wOffY*c);
    int rwx = cx + (int)(-wOffX*c - wOffY*s);
    int rwy = cy + (int)(-wOffX*s + wOffY*c);

    float steerNorm = abs(steerAngleDeg - 90) / 90.0;
    int frontRX = max(1, (int)(wheelR * (1.0 - 0.75*steerNorm)));

    int archW = wheelR + 1;

    int tx0 = cx + (int)((-bodyW/2)*c - (-bodyH/2)*s);
    int ty0 = cy + (int)((-bodyW/2)*s + (-bodyH/2)*c);
    int tx1 = cx + (int)(( bodyW/2)*c - (-bodyH/2)*s);
    int ty1 = cy + (int)(( bodyW/2)*s + (-bodyH/2)*c);
    display.drawLine(tx0, ty0, tx1, ty1, SSD1306_WHITE);

    int lx0 = cx + (int)((-bodyW/2)*c - (-bodyH/2)*s);
    int ly0 = cy + (int)((-bodyW/2)*s + (-bodyH/2)*c);
    int lx1 = cx + (int)((-bodyW/2)*c - ( bodyH/2)*s);
    int ly1 = cy + (int)((-bodyW/2)*s + ( bodyH/2)*c);
    display.drawLine(lx0, ly0, lx1, ly1, SSD1306_WHITE);

    int rx0 = cx + (int)((bodyW/2)*c - (-bodyH/2)*s);
    int ry0 = cy + (int)((bodyW/2)*s + (-bodyH/2)*c);
    int rx1 = cx + (int)((bodyW/2)*c - ( bodyH/2)*s);
    int ry1 = cy + (int)((bodyW/2)*s + ( bodyH/2)*c);
    display.drawLine(rx0, ry0, rx1, ry1, SSD1306_WHITE);

    int b0x = cx + (int)((-bodyW/2        )*c - (bodyH/2)*s);
    int b0y = cy + (int)((-bodyW/2        )*s + (bodyH/2)*c);
    int b1x = cx + (int)((-wOffX - archW  )*c - (bodyH/2)*s);
    int b1y = cy + (int)((-wOffX - archW  )*s + (bodyH/2)*c);
    display.drawLine(b0x, b0y, b1x, b1y, SSD1306_WHITE);

    int b2x = cx + (int)((-wOffX + archW  )*c - (bodyH/2)*s);
    int b2y = cy + (int)((-wOffX + archW  )*s + (bodyH/2)*c);
    int b3x = cx + (int)(( wOffX - archW  )*c - (bodyH/2)*s);
    int b3y = cy + (int)(( wOffX - archW  )*s + (bodyH/2)*c);
    display.drawLine(b2x, b2y, b3x, b3y, SSD1306_WHITE);

    int b4x = cx + (int)(( wOffX + archW  )*c - (bodyH/2)*s);
    int b4y = cy + (int)(( wOffX + archW  )*s + (bodyH/2)*c);
    int b5x = cx + (int)(( bodyW/2        )*c - (bodyH/2)*s);
    int b5y = cy + (int)(( bodyW/2        )*s + (bodyH/2)*c);
    display.drawLine(b4x, b4y, b5x, b5y, SSD1306_WHITE);

    int cabCorners[4][2] = {
        {-7, -bodyH/2-7}, { 7, -bodyH/2-7},
        { bodyW/2-6, -bodyH/2}, {-bodyW/2+6, -bodyH/2}
    };
    int rCab[4][2];
    for (int i = 0; i < 4; i++) {
        rCab[i][0] = cx + (int)(cabCorners[i][0]*c - cabCorners[i][1]*s);
        rCab[i][1] = cy + (int)(cabCorners[i][0]*s + cabCorners[i][1]*c);
    }
    for (int i = 0; i < 4; i++) {
        int j = (i+1)%4;
        display.drawLine(rCab[i][0], rCab[i][1], rCab[j][0], rCab[j][1], SSD1306_WHITE);
    }

    drawWheelWithSpoke(rwx, rwy, wheelR, wheelR, spokeAngle);
    drawWheelWithSpoke(fwx, fwy, frontRX, wheelR, 0);
}

void drawDisplacementArrow(int cx, int cy, float distCM) {
    int dir    = (distCM >= 0) ? 1 : -1;
    int len    = constrain((int)(abs(distCM) / 5.0), 2, 20);
    int startX = cx - dir*len/2;
    int endX   = cx + dir*len/2;

    display.drawLine(startX, cy, endX, cy, SSD1306_WHITE);
    display.drawLine(endX, cy, endX-dir*3, cy-3, SSD1306_WHITE);
    display.drawLine(endX, cy, endX-dir*3, cy+3, SSD1306_WHITE);

    display.setCursor(cx-12, cy+4);
    display.print(abs(distanceCM), 0); display.print("cm");
}

void printCompact(float val) {
    float absVal = abs(val);
    if (absVal >= 1000.0) {
        display.print(val / 1000.0, 1);
        display.print("k");
    } else {
        display.print((int)val);
    }
}

void drawEncoderPage() {
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);

    // left column
    display.setCursor(0, 0);  display.print("Spd:"); printCompact(abs(speedCMS));
    display.setCursor(0, 10); display.print("RPM:"); printCompact(abs(rpm));
    display.setCursor(0, 20); display.print("Dst:"); printCompact(abs(distanceCM));
    display.setCursor(0, 30); display.print("Dir:"); display.print(encoderDir >= 0 ? "FWD" : "REV");
    display.setCursor(0, 40); display.print("St:"); display.print(steeringAngleDeg(), 0); display.print("d");

    // circle + esc on same line
    display.drawBitmap(0, 56, circleBmp, 8, 8, SSD1306_WHITE);
    display.setCursor(10, 57); display.print("esc");

    // divider
    display.drawFastVLine(DIVIDER_ENC, 0, 64, SSD1306_WHITE);

    // right panel header
    display.setCursor(DIVIDER_ENC+3, 0);
    display.print(abs(speedCMS), 1); display.print("cm/s");
    display.drawFastHLine(DIVIDER_ENC+1, 8, SCREEN_WIDTH-DIVIDER_ENC-1, SSD1306_WHITE);

    // car centered in right panel
    int cx = (DIVIDER_ENC + SCREEN_WIDTH) / 2;
    int cy = 32;
    drawCarSide(cx, cy, pitch, servoAngle);

    // displacement arrow below
    drawDisplacementArrow(cx, cy+20, distanceCM);
}

// ─── HOME PAGE ────────────────────────────────────────────────
void drawHome() {
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);

    const char* title = "[ SELECT PAGE ]";
    int titleW = strlen(title) * 6;
    display.setCursor((SCREEN_WIDTH - titleW) / 2, 4);
    display.print(title);

    int boxX = 18, boxW = 92, boxH = 13;
    if (menuSelection == 0) display.fillRoundRect(boxX, 18, boxW, boxH, 3, SSD1306_WHITE);
    else                    display.drawRoundRect(boxX, 18, boxW, boxH, 3, SSD1306_WHITE);
    display.setTextColor(menuSelection == 0 ? SSD1306_BLACK : SSD1306_WHITE);
    const char* opt0 = "IMU / Cube";
    display.setCursor(boxX + (boxW - strlen(opt0)*6)/2, 22);
    display.print(opt0);

    display.setTextColor(SSD1306_WHITE);
    if (menuSelection == 1) display.fillRoundRect(boxX, 35, boxW, boxH, 3, SSD1306_WHITE);
    else                    display.drawRoundRect(boxX, 35, boxW, boxH, 3, SSD1306_WHITE);
    display.setTextColor(menuSelection == 1 ? SSD1306_BLACK : SSD1306_WHITE);
    const char* opt1 = "Motor/Encoder";
    display.setCursor(boxX + (boxW - strlen(opt1)*6)/2, 39);
    display.print(opt1);

    display.setTextColor(SSD1306_WHITE);
    const char* footer = "up/dn=nav  X=select";
    display.setCursor((SCREEN_WIDTH - strlen(footer)*6)/2, 54);
    display.print(footer);
}

// ─── BLUEPAD32 CALLBACKS ──────────────────────────────────────
void onConnectedController(ControllerPtr ctl) {
    myController = ctl;
    Serial.println("Controller connected");
}

void onDisconnectedController(ControllerPtr ctl) {
    myController = nullptr;
    Serial.println("Controller disconnected");
    stopMotor();
    steeringServo.write(SERVO_CENTER);
}

// ─── SETUP ────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);

    for (int pin : {OT1, OT2, EEP, LED}) {
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW);
    }
    pinMode(EC1, INPUT);
    pinMode(EC2, INPUT);
    attachInterrupt(digitalPinToInterrupt(EC1), onEncoder, CHANGE);

    ESP32PWM::allocateTimer(0);
    steeringServo.setPeriodHertz(50);
    steeringServo.attach(SIG, 1000, 2000);
    steeringServo.write(SERVO_CENTER);

    Wire.begin(SDA, SCL);
    Wire.setClock(100000);

    if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println("OLED failed"); for(;;);
    }
    display.clearDisplay(); display.display();

    if (!mpu.begin()) {
        display.setTextSize(1); display.setTextColor(SSD1306_WHITE);
        display.setCursor(0,0); display.println("MPU6050 NOT FOUND");
        display.display(); for(;;);
    }
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    display.clearDisplay(); display.setTextSize(1); display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,0); display.println("Calibrating...");
    display.println("Hold still."); display.display();

    int samples = 200;
    for (int i = 0; i < samples; i++) {
        sensors_event_t a, g, t;
        mpu.getEvent(&a, &g, &t);
        ax_off += a.acceleration.x; ay_off += a.acceleration.y; az_off += a.acceleration.z;
        gx_off += g.gyro.x;        gy_off += g.gyro.y;        gz_off += g.gyro.z;
        delay(5);
    }
    ax_off/=samples; ay_off/=samples; az_off/=samples;
    gx_off/=samples; gy_off/=samples; gz_off/=samples;
    az_off -= 9.81;

    sensors_event_t accel, gyro, temp;
    mpu.getEvent(&accel, &gyro, &temp);
    float ax0 = accel.acceleration.x - ax_off;
    float ay0 = accel.acceleration.y - ay_off;
    float az0 = accel.acceleration.z - az_off;
    pitch = atan2(ay0, az0);
    roll  = atan2(-ax0, sqrt(ay0*ay0 + az0*az0));

    lastIMUTime = millis();
    lastEncTime = millis();

    // ── WiFi ──────────────────────────────────────────────────
    display.clearDisplay(); display.setCursor(0,0);
    display.println("Connecting WiFi...");
    display.display();

    WiFi.begin(WIFI_SSID, WIFI_PASS);
    int tries = 0;
    while (WiFi.status() != WL_CONNECTED && tries < 40) {
        delay(250); tries++;
    }
    display.clearDisplay(); display.setCursor(0,0);
    if (WiFi.status() == WL_CONNECTED) {
        display.println("WiFi OK");
        display.println(WiFi.localIP().toString());
        Serial.print("IP: "); Serial.println(WiFi.localIP());
    } else {
        display.println("WiFi FAIL");
        display.println("Running offline");
        Serial.println("WiFi failed — running offline");
    }
    display.display(); delay(1500);

    // ── HTTP server ────────────────────────────────────────────
    server.on("/data",     HTTP_GET,  handleData);
    server.on("/reset",    HTTP_POST, handleReset);
    server.on("/mpu_mode", handleMpuMode);  // GET or POST
    server.begin();
    Serial.println("HTTP server started");

    BP32.setup(&onConnectedController, &onDisconnectedController);
}

// ─── LOOP ─────────────────────────────────────────────────────
void loop() {
    server.handleClient();   // ← service HTTP requests

    BP32.update();
    updateIMU();
    updateEncoder();

    if (myController && myController->isConnected()) {
        int  buttons     = myController->buttons();
        bool pressX      = buttons & BUTTON_A;
        bool pressCircle = buttons & BUTTON_B;
        bool pressUp     = myController->dpad() & DPAD_UP;
        bool pressDown   = myController->dpad() & DPAD_DOWN;

        updateMotor(myController->throttle(), myController->brake());
        updateSteering(myController->axisX());

        if (currentPage == HOME) {
            if (pressUp   && !lastUp)     menuSelection = 0;
            if (pressDown && !lastDown)   menuSelection = 1;
            if (pressX    && !lastX)      currentPage = (menuSelection == 0) ? MPU_PAGE : ENC_PAGE;
        } else {
            if (pressCircle && !lastCircle) currentPage = HOME;
        }

        lastX      = pressX;
        lastUp     = pressUp;
        lastDown   = pressDown;
        lastCircle = pressCircle;
    } else {
        stopMotor();
        steeringServo.write(SERVO_CENTER);
    }

    display.clearDisplay();
    switch (currentPage) {
        case HOME:     drawHome();        break;
        case MPU_PAGE: drawMPUPage();     break;
        case ENC_PAGE: drawEncoderPage(); break;
    }
    display.display();

    delay(20);
}
