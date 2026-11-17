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
