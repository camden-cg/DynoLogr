#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>

#define SDA 32
#define SCL 33
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

#define DIVIDER 46
#define CCX ((DIVIDER + SCREEN_WIDTH) / 2)  // always centered in right panel
#define CCY 36
#define ARROW_SCALE 16.0
#define ALPHA 0.98

float ax_off = 0, ay_off = 0, az_off = 0;
float gx_off = 0, gy_off = 0, gz_off = 0;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_MPU6050 mpu;

float cubeVertices[8][3] = {
  {-8, -8, -8}, { 8, -8, -8}, { 8,  8, -8}, {-8,  8, -8},
  {-8, -8,  8}, { 8, -8,  8}, { 8,  8,  8}, {-8,  8,  8}
};

int edges[12][2] = {
  {0,1},{1,2},{2,3},{3,0},
  {4,5},{5,6},{6,7},{7,4},
  {0,4},{1,5},{2,6},{3,7}
};

float pitch = 0, roll = 0, yaw = 0;
unsigned long lastTime = 0;

void rotatePoint(float x, float y, float z,
                 float rx, float ry, float rz,
                 float &ox, float &oy, float &oz) {
  float y1 = y * cos(rx) - z * sin(rx);
  float z1 = y * sin(rx) + z * cos(rx);
  float x2 = x * cos(ry) + z1 * sin(ry);
  float z2 = -x * sin(ry) + z1 * cos(ry);
  float x3 = x2 * cos(rz) - y1 * sin(rz);
  float y3 = x2 * sin(rz) + y1 * cos(rz);
  ox = x3; oy = y3; oz = z2;
}

void drawCube(float rx, float ry, float rz) {
  float projected[8][2];
  for (int i = 0; i < 8; i++) {
    float ox, oy, oz;
    rotatePoint(cubeVertices[i][0], cubeVertices[i][1], cubeVertices[i][2],
                rx, ry, rz, ox, oy, oz);
    float fov = 28.0;
    float scale = fov / (fov + oz);
    projected[i][0] = ox * scale + CCX;
    projected[i][1] = oy * scale + CCY;
  }
  for (int i = 0; i < 12; i++) {
    int a = edges[i][0], b = edges[i][1];
    display.drawLine(
      (int)projected[a][0], (int)projected[a][1],
      (int)projected[b][0], (int)projected[b][1],
      SSD1306_WHITE
    );
  }
}

void drawForceArrow(float gx, float gy) {
  if (abs(gx) < 0.05 && abs(gy) < 0.05) return;

  int tx = constrain(CCX + (int)(gx * ARROW_SCALE), DIVIDER + 2, SCREEN_WIDTH - 2);
  int ty = constrain(CCY - (int)(gy * ARROW_SCALE), 10, SCREEN_HEIGHT - 2);

  display.drawLine(CCX, CCY, tx, ty, SSD1306_WHITE);

  float angle = atan2(ty - CCY, tx - CCX);
  float wingLen = 5.0, wingAngle = 0.45;
  display.drawLine(tx, ty,
    tx - (int)(wingLen * cos(angle - wingAngle)),
    ty - (int)(wingLen * sin(angle - wingAngle)), SSD1306_WHITE);
  display.drawLine(tx, ty,
    tx - (int)(wingLen * cos(angle + wingAngle)),
    ty - (int)(wingLen * sin(angle + wingAngle)), SSD1306_WHITE);
}

void printSigned(float val, int decimals) {
  if (val >= 0) display.print("+");
  display.print(val, decimals);
}

void setup() {
  delay(1000);
  Serial.begin(115200);
  Wire.begin(SDA, SCL);
  Wire.setClock(100000);

  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println("OLED failed"); for (;;);
  }
  display.clearDisplay();
  display.display();

  if (!mpu.begin()) {
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("MPU6050");
    display.println("NOT FOUND");
    display.display();
    for (;;);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Calibrating...");
  display.println("Hold still.");
  display.display();

  int samples = 200;
  for (int i = 0; i < samples; i++) {
    sensors_event_t a, g, t;
    mpu.getEvent(&a, &g, &t);
    ax_off += a.acceleration.x;
    ay_off += a.acceleration.y;
    az_off += a.acceleration.z;
    gx_off += g.gyro.x;
    gy_off += g.gyro.y;
    gz_off += g.gyro.z;
    delay(5);
  }
  ax_off /= samples;
  ay_off /= samples;
  az_off /= samples;
  gx_off /= samples;
  gy_off /= samples;
  gz_off /= samples;

  // az_off needs to account for gravity — sensor reads ~9.81 at rest flat
  az_off -= 9.81;

  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);
  float ax0 = accel.acceleration.x - ax_off;
  float ay0 = accel.acceleration.y - ay_off;
  float az0 = accel.acceleration.z - az_off;
  pitch = atan2(ay0, az0);
  roll  = atan2(-ax0, sqrt(ay0 * ay0 + az0 * az0));

  lastTime = millis();
}

void loop() {
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);

  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  // apply offsets to everything before any math
  float ax_raw = accel.acceleration.x - ax_off;
  float ay_raw = accel.acceleration.y - ay_off;
  float az_raw = accel.acceleration.z - az_off;
  float gx = gyro.gyro.x - gx_off;
  float gy = gyro.gyro.y - gy_off;
  float gz = gyro.gyro.z - gz_off;

  // complementary filter using corrected values
  float pitch_accel = atan2(ay_raw, az_raw);
  float roll_accel  = atan2(-ax_raw, sqrt(ay_raw * ay_raw + az_raw * az_raw));

  pitch = ALPHA * (pitch + gx * dt) + (1.0 - ALPHA) * pitch_accel;
  roll  = ALPHA * (roll  + gy * dt) + (1.0 - ALPHA) * roll_accel;
  yaw  += gz * dt;

  // gravity removal
  float grav_x = -sin(roll);
  float grav_y =  sin(pitch) * cos(roll);
  float grav_z =  cos(pitch) * cos(roll);

  float lin_x = ax_raw - grav_x * 9.81;
  float lin_y = ay_raw - grav_y * 9.81;
  float lin_z = az_raw - grav_z * 9.81;

  float lx = lin_x / 9.81;
  float ly = lin_y / 9.81;
  float lz = lin_z / 9.81;

  float deadband = 0.05;
  if (abs(lx) < deadband) lx = 0;
  if (abs(ly) < deadband) ly = 0;
  if (abs(lz) < deadband) lz = 0;

  float magnitude = sqrt(lx*lx + ly*ly + lz*lz);

  display.clearDisplay();

  // left column
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);  display.print("Accl #s");
  display.setCursor(0, 8);  display.print("X:"); printSigned(lx, 2);
  display.setCursor(0, 16); display.print("Y:"); printSigned(ly, 2);
  display.setCursor(0, 24); display.print("Z:"); printSigned(lz, 2);

  display.setCursor(0, 33); display.print("Gyro #s");
  display.setCursor(0, 41); display.print("X:"); printSigned(gx, 2);
  display.setCursor(0, 49); display.print("Y:"); printSigned(gy, 2);
  display.setCursor(0, 57); display.print("Z:"); printSigned(gz, 2);

  // vertical divider
  display.drawFastVLine(DIVIDER, 0, 64, SSD1306_WHITE);

  // right panel header
  display.setCursor(DIVIDER + 4, 0);
  display.print("Net:");
  display.print(magnitude, 2);
  display.print("G");
  display.drawFastHLine(DIVIDER + 1, 8, SCREEN_WIDTH - DIVIDER - 1, SSD1306_WHITE);

  // cube + arrow
  drawCube(pitch, roll, yaw);
  drawForceArrow(lx, ly);
  display.fillCircle(CCX, CCY, 2, SSD1306_WHITE);

  display.display();
  delay(20);
}