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
#include "Credentials.h"

void onConnectedController(ControllerPtr ctl);
void onDisconnectedController(ControllerPtr ctl);
void IRAM_ATTR onEncoder();

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
