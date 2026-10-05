// MPU6050 -> roll/pitch/yaw, published in RAM for the host to read over SWD.
//
// Wiring (I2C1):
//   MPU6050 VCC -> 3V3
//   MPU6050 GND -> GND
//   MPU6050 SCL -> PB6
//   MPU6050 SDA -> PB7
//
// The host (ST-Link/OpenOCD) reads g_roll/g_pitch/g_yaw directly from RAM.
// Writing 1 to g_recenter_req re-levels roll/pitch and zeroes yaw.
//
// MPU6050 has no magnetometer, so yaw has no absolute reference: it is
// obtained purely by integrating the gyro Z rate and *will* drift over
// time. The 'c' command is how you compensate for that in practice.
// If you add a magnetometer (e.g. QMC5883L) later, replace the yaw
// integration below with a proper heading from the mag.

#include <Arduino.h>
#include <Wire.h>

static const uint8_t MPU_ADDR = 0x68;
static const uint8_t REG_PWR_MGMT_1 = 0x6B;
static const uint8_t REG_SMPLRT_DIV = 0x19;
static const uint8_t REG_CONFIG = 0x1A;
static const uint8_t REG_GYRO_CONFIG = 0x1B;
static const uint8_t REG_ACCEL_CONFIG = 0x1C;
static const uint8_t REG_ACCEL_XOUT_H = 0x3B;

// +-4g -> 8192 LSB/g, +-500 dps -> 65.5 LSB/(deg/s)
static const float ACCEL_SCALE = 8192.0f;
static const float GYRO_SCALE = 65.5f;

// Flip any of these to -1 if an axis moves the wrong way once mounted
// on the actual control frame.
static const float ROLL_SIGN = 1.0f;
static const float PITCH_SIGN = 1.0f;
static const float YAW_SIGN = 1.0f;

static const float COMPLEMENTARY_ALPHA = 0.98f;

volatile float g_roll = 0, g_pitch = 0, g_yaw = 0;
volatile uint8_t g_recenter_req = 0;

static float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;
static float roll = 0, pitch = 0, yaw = 0;
static uint32_t lastUpdateUs = 0;

static void mpuWrite(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

static void mpuReadRaw(int16_t &ax, int16_t &ay, int16_t &az,
                        int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14);

  ax = (Wire.read() << 8) | Wire.read();
  ay = (Wire.read() << 8) | Wire.read();
  az = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read(); // skip temperature
  gx = (Wire.read() << 8) | Wire.read();
  gy = (Wire.read() << 8) | Wire.read();
  gz = (Wire.read() << 8) | Wire.read();
}

static void calibrateGyro() {
  const int samples = 500;
  double sumX = 0, sumY = 0, sumZ = 0;
  int16_t ax, ay, az, gx, gy, gz;

  for (int i = 0; i < samples; i++) {
    mpuReadRaw(ax, ay, az, gx, gy, gz);
    sumX += gx;
    sumY += gy;
    sumZ += gz;
    delay(3);
  }

  gyroBiasX = sumX / samples;
  gyroBiasY = sumY / samples;
  gyroBiasZ = sumZ / samples;
}

static void levelFromAccel(int16_t ax, int16_t ay, int16_t az) {
  float axg = ax / ACCEL_SCALE;
  float ayg = ay / ACCEL_SCALE;
  float azg = az / ACCEL_SCALE;
  roll = ROLL_SIGN * atan2(ayg, azg) * 180.0f / PI;
  pitch = PITCH_SIGN * atan2(-axg, sqrt(ayg * ayg + azg * azg)) * 180.0f / PI;
  yaw = 0;
}

// Average several samples instead of trusting one noisy reading, so a
// recenter never leaves a spurious jump that gets fed straight to the sim.
static void recenter() {
  const int samples = 30;
  long sumAx = 0, sumAy = 0, sumAz = 0;
  int16_t ax, ay, az, gx, gy, gz;
  for (int i = 0; i < samples; i++) {
    mpuReadRaw(ax, ay, az, gx, gy, gz);
    sumAx += ax;
    sumAy += ay;
    sumAz += az;
    delay(2);
  }
  levelFromAccel(sumAx / samples, sumAy / samples, sumAz / samples);
}

void setup() {
  pinMode(PC13, OUTPUT);
  digitalWrite(PC13, LOW); // LED on (active low) while calibrating

  Wire.setSCL(PB6);
  Wire.setSDA(PB7);
  Wire.begin();
  Wire.setClock(400000);

  mpuWrite(REG_PWR_MGMT_1, 0x00);     // wake up
  mpuWrite(REG_SMPLRT_DIV, 0x00);
  mpuWrite(REG_CONFIG, 0x03);         // DLPF ~44 Hz
  mpuWrite(REG_GYRO_CONFIG, 0x08);    // +-500 dps
  mpuWrite(REG_ACCEL_CONFIG, 0x08);   // +-4g
  delay(100);

  calibrateGyro();
  recenter();

  digitalWrite(PC13, HIGH); // LED off = ready

  lastUpdateUs = micros();
}

void loop() {
  if (g_recenter_req) {
    g_recenter_req = 0;
    recenter();
  }

  int16_t ax, ay, az, gx, gy, gz;
  mpuReadRaw(ax, ay, az, gx, gy, gz);

  uint32_t nowUs = micros();
  float dt = (nowUs - lastUpdateUs) / 1000000.0f;
  lastUpdateUs = nowUs;
  if (dt <= 0 || dt > 0.1f) dt = 0.01f; // guard against overflow/first loop

  float gxDps = (gx - gyroBiasX) / GYRO_SCALE;
  float gyDps = (gy - gyroBiasY) / GYRO_SCALE;
  float gzDps = (gz - gyroBiasZ) / GYRO_SCALE;

  float axg = ax / ACCEL_SCALE;
  float ayg = ay / ACCEL_SCALE;
  float azg = az / ACCEL_SCALE;
  float accelRoll = ROLL_SIGN * atan2(ayg, azg) * 180.0f / PI;
  float accelPitch = PITCH_SIGN * atan2(-axg, sqrt(ayg * ayg + azg * azg)) * 180.0f / PI;

  roll = COMPLEMENTARY_ALPHA * (roll + ROLL_SIGN * gxDps * dt) +
         (1.0f - COMPLEMENTARY_ALPHA) * accelRoll;
  pitch = COMPLEMENTARY_ALPHA * (pitch + PITCH_SIGN * gyDps * dt) +
          (1.0f - COMPLEMENTARY_ALPHA) * accelPitch;
  yaw += YAW_SIGN * gzDps * dt; // no absolute reference -> will drift

  g_roll = roll;
  g_pitch = pitch;
  g_yaw = yaw;
}
