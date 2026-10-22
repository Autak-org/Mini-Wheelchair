#include <Arduino.h>
// Stair Climbing Wheelchair - ESP32-S3 BLE controller
// Control from a phone via Bluetooth Low Energy (Nordic UART Service)
//
// climbB motor = seat tilt. The BNO055 is mounted on the CHASSIS (not the seat).
// In climb mode the seat motor is moved to the position that cancels the chassis tilt:
//   target steps = chassis angle x SEAT_STEPS_PER_DEG
// so the seat stays level while the chassis pitches on the stairs.

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Wire.h>
#include <Preferences.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

// ===== BLE (Nordic UART Service) =====
#define SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define RX_UUID      "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"  // phone -> ESP32
#define TX_UUID      "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"  // ESP32 -> phone
const char *BLE_NAME = "Stair Climbing Wheelchair";

// ===== IMU: BNO055 over I2C =====
const int IMU_SDA_PIN = 11;
const int IMU_SCL_PIN = 12;
const uint8_t IMU_ADDRESS = 0x29;
Adafruit_BNO055 bno = Adafruit_BNO055(55, IMU_ADDRESS, &Wire);
bool imuReady = false;
unsigned long lastImuRead = 0;
unsigned long lastImuOk = 0;
unsigned long lastImuNotify = 0;

// ===== Motor pins =====
const int leftPins[4]   = {6, 7, 15, 16};
const int rightPins[4]  = {17, 18, 8, 3};
const int climbLPins[4] = {1, 2, 42, 41};
const int climbRPins[4] = {39, 38, 37, 36};
const int climbBPins[4] = {45, 48, 47, 21}; //Use for Seat Leveling

// ===== Settings =====
const bool LEFT_REVERSE    = true;
const bool RIGHT_REVERSE   = false;
const bool CLIMB_L_REVERSE = true;    // wrong direction while climbing -> flip this
const bool CLIMB_R_REVERSE = false;
const bool CLIMB_B_REVERSE = false;   // seat turns the wrong way (tilts MORE when the chassis tilts) -> flip this
const float TURN_STRENGTH = 0.8;      // how much the inner wheel slows when turning

// Step interval in microseconds (larger = slower).
const int MIN_INTERVAL = 2000;
const int MAX_INTERVAL = 7000;        // interval at the lowest speed
const float MIN_SPEED = 0.15;         // lowest speed fraction when moving
const float CLIMB_SPEED = 0.4;        // climbing speed scale (0..1)
const unsigned long FAILSAFE_MS = 500; // stop if no joystick packet for this long (web mode)

// ===== Seat leveling (climbB) =====
const bool  SEAT_AXIS_IS_ROLL   = false; // false = pitch (euler.z), true = roll (euler.y). Depends on how the BNO055 is mounted
const float SEAT_SIGN           = 1.0;   // -1.0 if "tilted back" shows up as "tilted forward" on the web page
const float SEAT_STEPS_PER_DEG  = 6.0;  // motor steps needed to tilt the seat 1 degree (MEASURE THIS)
const float SEAT_DEADBAND_DEG   = 1.0;   // ignore errors smaller than this (stops jitter)
const float SEAT_FILTER         = 0.3;   // IMU smoothing, 0..1 (smaller = smoother but slower)
const float SEAT_SPEED          = 0.5;   // seat motor speed (0..1)
const float SEAT_MAX_COMP_DEG   = 30.0;  // largest compensation the seat will make
const float SEAT_MAX_VALID_DEG  = 60.0;  // chassis angle beyond this is treated as an invalid reading
const unsigned long IMU_STALE_MS = 300;  // no fresh IMU data for this long -> seat motor holds still
// Hard travel limit for the seat motor, in steps from the level position (0 = disabled).
// Set this to what the mechanism can actually do.
const long SEAT_LIMIT_STEPS = 0;
// =========================

const uint8_t seq[4][4] = {
  {1,0,0,1}, {1,1,0,0}, {0,1,1,0}, {0,0,1,1}
};

struct Motor {
  const int *pins;
  bool reverse;
  int idx;
  float speed;             // -1.0 to 1.0
  unsigned long lastStep;
  bool hold;               // true = keep coils energized when stopped
  long pos;                // net steps taken (+ = positive speed direction)
};

Motor leftM   = {leftPins,   LEFT_REVERSE,    0, 0, 0, false, 0};
Motor rightM  = {rightPins,  RIGHT_REVERSE,   0, 0, 0, false, 0};
Motor climbL  = {climbLPins, CLIMB_L_REVERSE, 0, 0, 0, false, 0};
Motor climbR  = {climbRPins, CLIMB_R_REVERSE, 0, 0, 0, false, 0};
Motor climbB  = {climbBPins, CLIMB_B_REVERSE, 0, 0, 0, false, 0};

BLEServer *bleServer;
BLECharacteristic *txChar;
volatile bool deviceConnected = false;
volatile bool needRestartAdv = false;
volatile bool modeChanged = false;

volatile float cmdX = 0, cmdY = 0;   // x: + = right, y: + = forward / climb up
volatile bool streaming = false;     // true = receiving "J,x,y" continuously (failsafe active)
unsigned long lastCmd = 0;
bool climbMode = false;

// Chassis angle (degrees). + = chassis tilted back, - = tilted forward, 0 = level ground
Preferences prefs;
float carRaw = 0;                    // raw IMU angle
volatile float carOffset = 0;        // raw angle when the chassis is on level ground (saved in flash)
float carAngle = 0;                  // filtered chassis angle
bool carAngleInit = false;
volatile bool setLevelFlag = false;
bool seatMoving = false;

// Button states from the Bluefruit Connect Control Pad
bool bUp = false, bDown = false, bLeft = false, bRight = false;

void updateFromButtons() {
  cmdY = bUp ? 1.0 : (bDown ? -1.0 : 0.0);
  cmdX = bRight ? 1.0 : (bLeft ? -1.0 : 0.0);
}

void toggleMode() {
  climbMode = !climbMode;
  cmdX = cmdY = 0;
  bUp = bDown = bLeft = bRight = false;
  modeChanged = true;
}

// Text protocol: "J,0.50,-0.30\n" = joystick, "M\n" = toggle mode,
// "Z\n" = set level (chassis on level ground AND seat level)
void handleLine(const String &line) {
  if (line.startsWith("J,")) {
    int c = line.indexOf(',', 2);
    if (c < 0) return;
    cmdX = constrain(line.substring(2, c).toFloat(), -1.0, 1.0);
    cmdY = constrain(line.substring(c + 1).toFloat(), -1.0, 1.0);
    streaming = true;
    lastCmd = millis();
  } else if (line.startsWith("M")) {
    toggleMode();
  } else if (line.startsWith("Z")) {
    setLevelFlag = true;             // handled in loop()
  }
}

class ServerCB : public BLEServerCallbacks {
  void onConnect(BLEServer *s) { deviceConnected = true; }
  void onDisconnect(BLEServer *s) {
    deviceConnected = false;
    cmdX = cmdY = 0;
    bUp = bDown = bLeft = bRight = false;
    needRestartAdv = true;
  }
};

class RxCB : public BLECharacteristicCallbacks {
  String textBuf;
  void onWrite(BLECharacteristic *c) {
    auto v = c->getValue();
    size_t len = v.length();
    if (len == 0) return;
    const uint8_t *d = (const uint8_t *)v.c_str();

    // Bluefruit Connect button packet: '!' 'B' <button> <1=pressed/0=released> <checksum>
    if (len >= 4 && d[0] == '!' && d[1] == 'B') {
      streaming = false;
      char btn = d[2];
      bool pressed = (d[3] == '1');
      switch (btn) {
        case '5': bUp = pressed;    break;   // arrow up
        case '6': bDown = pressed;  break;   // arrow down
        case '7': bLeft = pressed;  break;   // arrow left
        case '8': bRight = pressed; break;   // arrow right
        case '1': if (pressed) toggleMode(); break;  // button 1 = toggle mode
      }
      updateFromButtons();
      return;
    }

    // Text lines (may arrive in several chunks, so buffer until '\n')
    for (size_t i = 0; i < len; i++) {
      char ch = (char)d[i];
      if (ch == '\n') { handleLine(textBuf); textBuf = ""; }
      else if (ch != '\r' && textBuf.length() < 40) textBuf += ch;
    }
  }
};

void initMotor(Motor &m) {
  for (int i = 0; i < 4; i++) {
    pinMode(m.pins[i], OUTPUT);
    digitalWrite(m.pins[i], LOW);
  }
}

void runMotor(Motor &m) {
  float a = fabs(m.speed);
  if (a < 0.05) {
    if (!m.hold) for (int i = 0; i < 4; i++) digitalWrite(m.pins[i], LOW);
    return;
  }
  float s = MIN_SPEED + (1.0 - MIN_SPEED) * a;
  unsigned long interval = MAX_INTERVAL - (MAX_INTERVAL - MIN_INTERVAL) * s;

  if (micros() - m.lastStep >= interval) {
    m.lastStep = micros();
    int dir = ((m.speed > 0) != m.reverse) ? 1 : -1;
    m.idx = (m.idx + dir + 4) % 4;
    m.pos += (m.speed > 0) ? 1 : -1;
    for (int i = 0; i < 4; i++) digitalWrite(m.pins[i], seq[m.idx][i]);
  }
}

void setup() {
  Serial.begin(115200);

  prefs.begin("seat", false);
  carOffset = prefs.getFloat("off", 0);

  // Initialize I2C on the requested ESP32-S3 pins.
  Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN);
  Wire.setClock(100000); // conservative I2C speed for sensor wiring
  imuReady = bno.begin();
  if (imuReady) {
    bno.setExtCrystalUse(true);
    Serial.println("BNO055 found at I2C address 0x29");
  } else {
    Serial.println("BNO055 not found at 0x29; check wiring/address/module type");
  }

  initMotor(leftM);  initMotor(rightM);
  initMotor(climbL); initMotor(climbR); initMotor(climbB);

  BLEDevice::init(BLE_NAME);
  bleServer = BLEDevice::createServer();
  bleServer->setCallbacks(new ServerCB());

  BLEService *svc = bleServer->createService(SERVICE_UUID);

  txChar = svc->createCharacteristic(TX_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  txChar->addDescriptor(new BLE2902());

  BLECharacteristic *rxChar = svc->createCharacteristic(
      RX_UUID, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  rxChar->setCallbacks(new RxCB());

  svc->start();
  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(SERVICE_UUID);
  adv->setScanResponse(true);
  BLEDevice::startAdvertising();
}

void loop() {
  if (needRestartAdv) {
    needRestartAdv = false;
    delay(200);
    BLEDevice::startAdvertising();
  }

  // "Set level": chassis is on level ground and the seat is level right now
  if (setLevelFlag) {
    setLevelFlag = false;
    carOffset = carRaw;
    carAngle = 0;
    climbB.pos = 0;                  // seat motor position 0 = seat level
    seatMoving = false;
    prefs.putFloat("off", carOffset);
  }

  if (modeChanged) {
    modeChanged = false;
    if (deviceConnected) {
      txChar->setValue(climbMode ? "CLIMB" : "DRIVE");
      txChar->notify();
    }
  }

  // Read the IMU (20 Hz): chassis angle, smoothed
  if (imuReady && millis() - lastImuRead >= 50) {
    lastImuRead = millis();
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    float raw = SEAT_AXIS_IS_ROLL ? euler.y() : euler.z();
    if (isfinite(raw)) {
      carRaw = raw;
      float a = raw - carOffset;
      if (a > 180) a -= 360;
      if (a < -180) a += 360;
      a *= SEAT_SIGN;
      if (!carAngleInit) { carAngle = a; carAngleInit = true; }
      else carAngle += SEAT_FILTER * (a - carAngle);
      lastImuOk = millis();
    }
  }

  // Send the estimated SEAT angle to the browser (10 Hz). Format: SEAT,angle
  // seat angle = chassis angle - how much the seat motor has compensated
  if (deviceConnected && imuReady && millis() - lastImuNotify >= 100) {
    lastImuNotify = millis();
    float seatEst = carAngle - (float)climbB.pos / SEAT_STEPS_PER_DEG;
    String packet = "SEAT," + String(seatEst, 1);
    txChar->setValue(packet.c_str());
    txChar->notify();
  }

  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate >= 20) {
    lastUpdate = millis();

    // Failsafe: in joystick (web) mode, stop if packets stop arriving
    if (streaming && millis() - lastCmd > FAILSAFE_MS) { cmdX = 0; cmdY = 0; }
    // Always stop if the Bluetooth connection is lost
    if (!deviceConnected) { cmdX = 0; cmdY = 0; }

    float x = cmdX, y = cmdY;

    if (!climbMode) {
      // Drive mode: turn by slowing only the inner wheel
      float slow = 1.0 - TURN_STRENGTH * fabs(x);
      leftM.speed  = (x < 0) ? y * slow : y;
      rightM.speed = (x > 0) ? y * slow : y;
      climbL.speed = climbR.speed = 0;
    } else {
      // Climb mode: wheels stop, joystick Y drives the 2 climbing motors
      leftM.speed = rightM.speed = 0;
      climbL.speed = climbR.speed = y * CLIMB_SPEED;
    }

    // ---- Seat leveling with climbB (climb mode only) ----
    // Chassis tilts back by A degrees -> seat motor goes to A x SEAT_STEPS_PER_DEG steps.
    // Chassis returns to level -> target returns to 0 and the seat follows.
    // Positive climbB speed is assumed to tilt the seat forward.
    float seatCmd = 0;
    bool imuOk = imuReady && (millis() - lastImuOk < IMU_STALE_MS) &&
                 fabs(carAngle) < SEAT_MAX_VALID_DEG;

    if (climbMode && deviceConnected && imuOk) {
      float comp = constrain(carAngle, -SEAT_MAX_COMP_DEG, SEAT_MAX_COMP_DEG);
      long target = lround(comp * SEAT_STEPS_PER_DEG);
      if (SEAT_LIMIT_STEPS > 0) target = constrain(target, -SEAT_LIMIT_STEPS, SEAT_LIMIT_STEPS);

      long err  = target - climbB.pos;
      long band = lround(SEAT_DEADBAND_DEG * SEAT_STEPS_PER_DEG);
      if (!seatMoving && labs(err) > band) seatMoving = true;
      else if (seatMoving && labs(err) <= band / 3) seatMoving = false;

      if (seatMoving) seatCmd = (err > 0) ? SEAT_SPEED : -SEAT_SPEED;
    } else {
      seatMoving = false;            // not climbing / no connection / bad IMU: hold still
    }

    climbB.speed = seatCmd;
    climbB.hold = climbMode;   // keep the seat motor energized in climb mode so the seat doesn't drift
  }
  runMotor(leftM);
  runMotor(rightM);
  runMotor(climbL);
  runMotor(climbR);
  runMotor(climbB);
}