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
