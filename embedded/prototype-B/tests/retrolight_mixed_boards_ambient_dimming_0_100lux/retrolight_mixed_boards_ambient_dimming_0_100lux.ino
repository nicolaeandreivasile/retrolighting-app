#include <Arduino.h>
#include <Wire.h>
#include <math.h>

/*
 * Retrolight central-unit sensor and LED bring-up test
 * Hardware reference: commit 912dfac
 * Arduino STM32 core: 3.0.0
 *
 * Central-unit sensor bus, I2C1:
 *   SCL      PB6
 *   SDA      PB7
 *   ACC_INT  PB8
 *   ALS_INT  PB9
 *
 * Central-unit external connector bus, I2C2:
 *   SCL      PB10
 *   SDA      PB11
 *
 * External retrolight-main LED board:
 *   TLC59108 I2C address 0x40 (A3:A0 strapped low)
 *   RESET connected to central-unit GPIO1 / PB1
 *   OUT0...OUT7 controlled together
 *   REXT = 374 ohms; IREF = 0xBF for approximately 25.1 mA
 *   peak sink current per active channel
 *
 * Behaviour:
 *   - Dark environment  -> high LED PWM duty cycle
 *   - Bright environment -> low LED PWM duty cycle
 *   - Movement detected -> 1.0 s hardware blink period, 50% duty cycle
 *   - No movement        -> steady light
 *
 * USB output uses the STM32 Arduino core's native USB CDC Serial object.
 */

namespace BoardPins {
// Generic STM32 variants define these as Arduino pin numbers. Keep them as
// pin_size_t for compatibility with STM32 Arduino core 3.0.0.
constexpr pin_size_t kSensorI2cScl = PB6;
constexpr pin_size_t kSensorI2cSda = PB7;
constexpr pin_size_t kAccInt = PB8;
constexpr pin_size_t kAlsInt = PB9;

// Prototype-B connector bus: I2C2 on PB10/PB11.
constexpr pin_size_t kLedI2cScl = PB10;
constexpr pin_size_t kLedI2cSda = PB11;

// Prototype-B PCB routes GPIO1 to MCU PB1 (LQFP64 pin 27).
// GPIO1 drives the TLC59108 active-low RESET input.
constexpr pin_size_t kLedReset = PB1;
}  // namespace BoardPins

// The default Wire object is used exclusively for the on-board sensors.
// TwoWire constructor order is SDA, then SCL.
TwoWire LedWire(BoardPins::kLedI2cSda, BoardPins::kLedI2cScl);

namespace Lis2hh12 {
constexpr uint8_t kCandidateAddresses[] = {0x1E, 0x1D};
constexpr uint8_t kWhoAmI = 0x0F;
constexpr uint8_t kExpectedWhoAmI = 0x41;
constexpr uint8_t kCtrl1 = 0x20;
constexpr uint8_t kCtrl3 = 0x22;
constexpr uint8_t kCtrl4 = 0x23;
constexpr uint8_t kOutXL = 0x28;
constexpr float kGPerCountAt2G = 0.000061f;
}  // namespace Lis2hh12

namespace Veml6030 {
constexpr uint8_t kCandidateAddresses[] = {0x10, 0x48};
constexpr uint8_t kAlsConf = 0x00;
constexpr uint8_t kAlsData = 0x04;
constexpr uint8_t kDeviceId = 0x07;

// Gain 1/4, integration time 100 ms, interrupt disabled, sensor enabled.
constexpr uint16_t kConfiguration = 0x1800;
constexpr float kLuxPerCount = 0.2688f;
}  // namespace Veml6030

namespace Tlc59108 {
constexpr uint8_t kAddress = 0x40;

constexpr uint8_t kMode1 = 0x00;
constexpr uint8_t kMode2 = 0x01;
constexpr uint8_t kPwm0 = 0x02;
constexpr uint8_t kGroupPwm = 0x0A;
constexpr uint8_t kGroupFrequency = 0x0B;
constexpr uint8_t kLedOut0 = 0x0C;
constexpr uint8_t kLedOut1 = 0x0D;
constexpr uint8_t kIref = 0x12;

// retrolight-main prototype-A uses REXT = 374 ohms.
// IREF = 0xBF: CM=1, HC=0, CC=63, current gain ~= 0.496.
// Approximate peak sink current:
//   IOUT ~= (1.26 V / 374 ohms) * 15 * 0.496 ~= 25.1 mA.
constexpr uint8_t kIrefFor374OhmApproximately25mA = 0xBF;

// MODE1: oscillator running; sub-call and all-call responses disabled.
constexpr uint8_t kMode1Normal = 0x00;

// MODE2 DMBLNK=1. Group blinking applies only to outputs whose LEDOUT field
// is 0b11; outputs configured as 0b10 remain steady individual PWM outputs.
constexpr uint8_t kMode2GroupBlink = 0x20;

// GRPPWM=128 gives approximately 50% on-time during grouped blinking.
constexpr uint8_t kBlinkDuty50Percent = 128;

// Blink period = (GRPFREQ + 1) / 24 seconds. 23 gives exactly 1.0 s.
constexpr uint8_t kOneSecondBlinkPeriod = 23;

// Four 2-bit fields per LEDOUT register:
//   0b10: individual PWM only (steady)
//   0b11: individual PWM plus group blink
constexpr uint8_t kAllChannelsSteadyPwm = 0xAA;
constexpr uint8_t kAllChannelsGroupBlink = 0xFF;
constexpr uint8_t kChannelCount = 8;
}  // namespace Tlc59108

constexpr uint32_t kUsbEnumerationWaitMs = 3000;
constexpr uint32_t kSensorSamplePeriodMs = 100;
constexpr uint32_t kDisplayPeriodMs = 2000;

// Motion detection parameters. Every new qualifying acceleration change
// extends blinking for this hold time, preventing rapid mode chatter.
constexpr float kMotionDeltaThresholdG = 0.050f;
constexpr uint32_t kMotionBlinkHoldMs = 3000;

// Ambient-light-to-PWM mapping.
//
// Automatic dimming is limited to the 0...100 lux range:
//   0 lux   -> maximum LED output
//   100 lux -> preset bright-environment output
//
// Any reading above 100 lux uses the same preset PWM value.
constexpr float kLuxAtMaximumLedOutput = 0.0f;
constexpr float kLuxAtPresetLedOutput = 100.0f;
constexpr uint8_t kPresetBrightLedPwm = 32;
constexpr uint8_t kMaximumLedPwm = 255;

// Low-pass filtering prevents small VEML6030 variations from causing visible
// LED brightness jitter. Higher alpha follows light changes more quickly.
constexpr float kAmbientFilterAlpha = 0.15f;
constexpr uint8_t kMinimumPwmUpdateStep = 2;

uint8_t g_lisAddress = 0;
uint8_t g_vemlAddress = 0;
bool g_ledDriverAvailable = false;
bool g_ledWriteHealthy = true;
const char* g_ledInitFailure = "not attempted";

bool g_havePreviousAcceleration = false;
float g_previousXG = 0.0f;
float g_previousYG = 0.0f;
float g_previousZG = 0.0f;

bool g_haveFilteredLux = false;
float g_filteredLux = 0.0f;

bool g_motionHasOccurred = false;
uint32_t g_lastMotionMs = 0;
bool g_ledBlinking = false;
int16_t g_appliedLedPwm = -1;

void printHexByte(uint8_t value) {
  Serial.print(F("0x"));
  if (value < 0x10) {
    Serial.print('0');
  }
  Serial.print(value, HEX);
}

void printHexWord(uint16_t value) {
  Serial.print(F("0x"));
  if (value < 0x1000) Serial.print('0');
  if (value < 0x0100) Serial.print('0');
  if (value < 0x0010) Serial.print('0');
  Serial.print(value, HEX);
}

uint8_t i2cProbe(TwoWire& bus, uint8_t address) {
  bus.beginTransmission(address);
  return bus.endTransmission();
}

bool i2cDeviceResponds(TwoWire& bus, uint8_t address) {
  return i2cProbe(bus, address) == 0;
}

bool writeRegister8(TwoWire& bus, uint8_t address,
                    uint8_t reg, uint8_t value) {
  bus.beginTransmission(address);
  bus.write(reg);
  bus.write(value);
  return bus.endTransmission() == 0;
}

bool writeRegister16LE(TwoWire& bus, uint8_t address,
                       uint8_t reg, uint16_t value) {
  bus.beginTransmission(address);
  bus.write(reg);
  bus.write(static_cast<uint8_t>(value & 0xFF));
  bus.write(static_cast<uint8_t>(value >> 8));
  return bus.endTransmission() == 0;
}

bool readRegisters(TwoWire& bus, uint8_t address, uint8_t startReg,
                   uint8_t* data, size_t length) {
  bus.beginTransmission(address);
  bus.write(startReg);
  if (bus.endTransmission(false) != 0) {
    return false;
  }

  const size_t received = bus.requestFrom(address, length);
  if (received != length) {
    while (bus.available()) {
      (void)bus.read();
    }
    return false;
  }

  for (size_t i = 0; i < length; ++i) {
    if (!bus.available()) {
      return false;
    }
    data[i] = static_cast<uint8_t>(bus.read());
  }

  return true;
}

bool readRegister8(TwoWire& bus, uint8_t address,
                   uint8_t reg, uint8_t& value) {
  return readRegisters(bus, address, reg, &value, 1);
}

bool readRegister16LE(TwoWire& bus, uint8_t address,
                      uint8_t reg, uint16_t& value) {
  uint8_t bytes[2] = {};
  if (!readRegisters(bus, address, reg, bytes, sizeof(bytes))) {
    return false;
  }

  value = static_cast<uint16_t>(bytes[0]) |
          (static_cast<uint16_t>(bytes[1]) << 8);
  return true;
}

void scanI2cBus(TwoWire& bus, const __FlashStringHelper* label) {
  Serial.println(label);
  bool foundAny = false;

  for (uint8_t address = 1; address < 0x7F; ++address) {
    const uint8_t status = i2cProbe(bus, address);
    if (status == 0) {
      Serial.print(F("  ACK at "));
      printHexByte(address);
      Serial.println();
      foundAny = true;
    }
  }

  if (!foundAny) {
    Serial.println(F("  No responding devices."));
  }
}

uint8_t detectLis2hh12() {
  for (uint8_t address : Lis2hh12::kCandidateAddresses) {
    uint8_t whoAmI = 0;
    if (readRegister8(Wire, address, Lis2hh12::kWhoAmI, whoAmI) &&
        whoAmI == Lis2hh12::kExpectedWhoAmI) {
      return address;
    }
  }
  return 0;
}

uint8_t detectVeml6030() {
  for (uint8_t address : Veml6030::kCandidateAddresses) {
    uint16_t deviceId = 0;
    if (readRegister16LE(Wire, address, Veml6030::kDeviceId, deviceId) &&
        (deviceId & 0x00FFU) == 0x81U) {
      return address;
    }
  }
  return 0;
}

bool initializeLis2hh12() {
  // CTRL4: +/-2 g full scale and register-address auto-increment enabled.
  if (!writeRegister8(Wire, g_lisAddress, Lis2hh12::kCtrl4, 0x04)) {
    return false;
  }

  // CTRL3: route data-ready signal to INT1 (PB8 on this board).
  if (!writeRegister8(Wire, g_lisAddress, Lis2hh12::kCtrl3, 0x01)) {
    return false;
  }

  // CTRL1: high-resolution, 100 Hz ODR, block-data update, XYZ enabled.
  if (!writeRegister8(Wire, g_lisAddress, Lis2hh12::kCtrl1, 0xBF)) {
    return false;
  }

  delay(20);
  return true;
}

bool initializeVeml6030() {
  if (!writeRegister16LE(Wire, g_vemlAddress, Veml6030::kAlsConf,
                         Veml6030::kConfiguration)) {
    return false;
  }

  // Allow at least one complete 100 ms conversion after changing ALS_CONF.
  delay(120);
  return true;
}

void pulseLedDriverReset() {
  // TLC59108 RESET is active low. On the prototype-B PCB, GPIO1 is PB1.
  pinMode(BoardPins::kLedReset, OUTPUT);
  digitalWrite(BoardPins::kLedReset, LOW);
  delay(5);
  digitalWrite(BoardPins::kLedReset, HIGH);
  delay(5);
}

bool writeAllLedPwm(uint8_t pwm) {
  for (uint8_t channel = 0; channel < Tlc59108::kChannelCount; ++channel) {
    if (!writeRegister8(LedWire, Tlc59108::kAddress,
                        static_cast<uint8_t>(Tlc59108::kPwm0 + channel),
                        pwm)) {
      return false;
    }
  }
  return true;
}

bool setLedOutputMode(bool blink) {
  const uint8_t outputMode = blink
                                 ? Tlc59108::kAllChannelsGroupBlink
                                 : Tlc59108::kAllChannelsSteadyPwm;

  if (!writeRegister8(LedWire, Tlc59108::kAddress, Tlc59108::kLedOut0, outputMode)) {
    return false;
  }
  if (!writeRegister8(LedWire, Tlc59108::kAddress, Tlc59108::kLedOut1, outputMode)) {
    return false;
  }

  g_ledBlinking = blink;
  return true;
}

bool initializeTlc59108() {
  g_ledInitFailure = "reset/probe not completed";

  pulseLedDriverReset();

  const uint8_t probeStatus = i2cProbe(LedWire, Tlc59108::kAddress);
  if (probeStatus != 0) {
    if (probeStatus == 2) {
      g_ledInitFailure = "no ACK at 0x40; verify I2C2 PB10/PB11 and RESET GPIO1/PB1";
    } else if (probeStatus == 4) {
      g_ledInitFailure = "I2C2 bus error or timeout while probing 0x40";
    } else {
      g_ledInitFailure = "unexpected I2C2 probe failure at 0x40";
    }
    return false;
  }

  if (!writeRegister8(LedWire, Tlc59108::kAddress,
                      Tlc59108::kMode1, Tlc59108::kMode1Normal)) {
    g_ledInitFailure = "MODE1 write failed";
    return false;
  }

  delay(1);

  if (!writeRegister8(LedWire, Tlc59108::kAddress,
                      Tlc59108::kMode2, Tlc59108::kMode2GroupBlink)) {
    g_ledInitFailure = "MODE2 write failed";
    return false;
  }

  if (!writeRegister8(
          LedWire,
          Tlc59108::kAddress,
          Tlc59108::kIref,
          Tlc59108::kIrefFor374OhmApproximately25mA)) {
    g_ledInitFailure = "IREF write failed";
    return false;
  }

  if (!writeRegister8(LedWire, Tlc59108::kAddress,
                      Tlc59108::kGroupPwm,
                      Tlc59108::kBlinkDuty50Percent)) {
    g_ledInitFailure = "GRPPWM write failed";
    return false;
  }

  if (!writeRegister8(LedWire, Tlc59108::kAddress,
                      Tlc59108::kGroupFrequency,
                      Tlc59108::kOneSecondBlinkPeriod)) {
    g_ledInitFailure = "GRPFREQ write failed";
    return false;
  }

  if (!writeAllLedPwm(0)) {
    g_ledInitFailure = "PWM0..PWM7 write failed";
    return false;
  }

  if (!setLedOutputMode(false)) {
    g_ledInitFailure = "LEDOUT0/LEDOUT1 write failed";
    return false;
  }

  uint8_t mode1 = 0;
  uint8_t iref = 0;
  if (!readRegister8(LedWire, Tlc59108::kAddress,
                     Tlc59108::kMode1, mode1) ||
      !readRegister8(LedWire, Tlc59108::kAddress,
                     Tlc59108::kIref, iref)) {
    g_ledInitFailure = "register readback failed";
    return false;
  }

  if (mode1 != Tlc59108::kMode1Normal ||
      iref != Tlc59108::kIrefFor374OhmApproximately25mA) {
    g_ledInitFailure = "register readback mismatch";
    return false;
  }

  g_appliedLedPwm = 0;
  g_ledInitFailure = "none";
  return true;
}

bool readAcceleration(float& xG, float& yG, float& zG) {
  uint8_t bytes[6] = {};
  if (!readRegisters(Wire, g_lisAddress, Lis2hh12::kOutXL, bytes, sizeof(bytes))) {
    return false;
  }

  const int16_t rawX = static_cast<int16_t>(
      static_cast<uint16_t>(bytes[0]) |
      (static_cast<uint16_t>(bytes[1]) << 8));
  const int16_t rawY = static_cast<int16_t>(
      static_cast<uint16_t>(bytes[2]) |
      (static_cast<uint16_t>(bytes[3]) << 8));
  const int16_t rawZ = static_cast<int16_t>(
      static_cast<uint16_t>(bytes[4]) |
      (static_cast<uint16_t>(bytes[5]) << 8));

  xG = rawX * Lis2hh12::kGPerCountAt2G;
  yG = rawY * Lis2hh12::kGPerCountAt2G;
  zG = rawZ * Lis2hh12::kGPerCountAt2G;
  return true;
}

bool readAmbientLight(uint16_t& rawAls, float& lux) {
  if (!readRegister16LE(Wire, g_vemlAddress, Veml6030::kAlsData, rawAls)) {
    return false;
  }

  lux = rawAls * Veml6030::kLuxPerCount;
  return true;
}

uint8_t calculateLedPwm(float lux) {
  if (lux <= kLuxAtMaximumLedOutput) {
    return kMaximumLedPwm;
  }

  if (lux >= kLuxAtPresetLedOutput) {
    return kPresetBrightLedPwm;
  }

  // Linear inverse mapping across 0...100 lux.
  const float normalized = lux / kLuxAtPresetLedOutput;

  const float pwm =
      static_cast<float>(kMaximumLedPwm) -
      normalized *
          static_cast<float>(kMaximumLedPwm - kPresetBrightLedPwm);

  return static_cast<uint8_t>(pwm + 0.5f);
}

bool updateLedControl(uint32_t nowMs, uint8_t targetPwm) {
  if (!g_ledDriverAvailable) {
    return false;
  }

  const bool motionActive =
      g_motionHasOccurred &&
      static_cast<uint32_t>(nowMs - g_lastMotionMs) < kMotionBlinkHoldMs;

  bool success = true;

  const int16_t pwmDifference =
      static_cast<int16_t>(targetPwm) - g_appliedLedPwm;
  const int16_t absoluteDifference =
      pwmDifference < 0 ? -pwmDifference : pwmDifference;

  if (g_appliedLedPwm < 0 ||
      absoluteDifference >= static_cast<int16_t>(kMinimumPwmUpdateStep)) {
    if (writeAllLedPwm(targetPwm)) {
      g_appliedLedPwm = targetPwm;
    } else {
      success = false;
    }
  }

  if (motionActive != g_ledBlinking) {
    if (!setLedOutputMode(motionActive)) {
      success = false;
    }
  }

  g_ledWriteHealthy = success;
  return success;
}

void printConfigurationResult() {
  if (g_lisAddress != 0) {
    uint8_t id = 0;
    (void)readRegister8(Wire, g_lisAddress, Lis2hh12::kWhoAmI, id);
    Serial.print(F("LIS2HH12 found at "));
    printHexByte(g_lisAddress);
    Serial.print(F(", WHO_AM_I="));
    printHexByte(id);
    Serial.println();
  } else {
    Serial.println(F("ERROR: LIS2HH12 not found at 0x1D or 0x1E."));
  }

  if (g_vemlAddress != 0) {
    uint16_t id = 0;
    (void)readRegister16LE(Wire, g_vemlAddress, Veml6030::kDeviceId, id);
    Serial.print(F("VEML6030 found at "));
    printHexByte(g_vemlAddress);
    Serial.print(F(", ID="));
    printHexWord(id);
    Serial.println();
  } else {
    Serial.println(F("ERROR: VEML6030 not found at 0x10 or 0x48."));
  }

  if (g_ledDriverAvailable) {
    Serial.print(F("TLC59108 LED driver initialized at "));
    printHexByte(Tlc59108::kAddress);
    Serial.println(F("."));

    uint8_t iref = 0;
    if (readRegister8(LedWire, Tlc59108::kAddress, Tlc59108::kIref, iref)) {
      Serial.print(F("TLC59108 IREF: "));
      printHexByte(iref);
      Serial.println(F(" (REXT=374 ohm, approximately 25.1 mA peak/channel)"));
    } else {
      Serial.println(F("WARNING: TLC59108 IREF readback failed."));
    }
  } else {
    Serial.println(F("WARNING: TLC59108 not found or initialization failed."));
    Serial.print(F("         Failure stage: "));
    Serial.println(g_ledInitFailure);
    Serial.println(F("         Sensor monitoring will continue with LEDs disabled."));
  }
}

void setup() {
  Serial.begin(115200);

  const uint32_t usbWaitStart = millis();
  while (!Serial && (millis() - usbWaitStart < kUsbEnumerationWaitMs)) {
    delay(10);
  }

  Serial.println();
  Serial.println(F("Retrolight mixed-board test - GPIO1/PB1 reset fix"));
  Serial.println(F("Sensor I2C1: PB6=SCL, PB7=SDA"));
  Serial.println(F("LED I2C2:    PB10=SCL, PB11=SDA"));
  Serial.println(F("LED driver:  TLC59108 at 0x40; RESET=GPIO1/PB1"));

  pinMode(BoardPins::kAccInt, INPUT);
  pinMode(BoardPins::kAlsInt, INPUT);

  // Release RESET before starting I2C. initializeTlc59108() then applies a
  // deliberate low pulse to reset every attached LED board simultaneously.
  pinMode(BoardPins::kLedReset, OUTPUT);
  digitalWrite(BoardPins::kLedReset, HIGH);
  delay(5);

  Serial.print(F("GPIO1/PB1 RESET release level: "));
  Serial.println(digitalRead(BoardPins::kLedReset) == HIGH ? F("HIGH") : F("LOW"));

  Wire.setSCL(BoardPins::kSensorI2cScl);
  Wire.setSDA(BoardPins::kSensorI2cSda);
  Wire.begin();
  Wire.setClock(400000);

  // The external connector is routed to I2C2, not to the sensor bus.
  LedWire.begin();
  LedWire.setClock(100000);

  scanI2cBus(Wire, F("Sensor bus scan, I2C1 PB6/PB7:"));
  scanI2cBus(
      LedWire,
      F("External LED bus scan before initialization, I2C2 PB10/PB11:"));

  g_ledDriverAvailable = initializeTlc59108();

  if (g_ledDriverAvailable) {
    scanI2cBus(LedWire, F("External LED bus scan after initialization:"));
  }

  g_lisAddress = detectLis2hh12();
  g_vemlAddress = detectVeml6030();
  printConfigurationResult();

  bool sensorInitializationOk = true;

  if (g_lisAddress == 0 || !initializeLis2hh12()) {
    Serial.println(F("ERROR: LIS2HH12 initialization failed."));
    sensorInitializationOk = false;
  }

  if (g_vemlAddress == 0 || !initializeVeml6030()) {
    Serial.println(F("ERROR: VEML6030 initialization failed."));
    sensorInitializationOk = false;
  }

  if (!sensorInitializationOk) {
    Serial.println(F("Sensor sampling is disabled; inspect the scan and wiring."));
    if (g_ledDriverAvailable) {
      (void)writeAllLedPwm(0);
    }
    return;
  }

  Serial.println(F("Sensors initialized successfully."));
  Serial.println(F("Readings will be displayed every 2 seconds."));
  Serial.println(F("LED dimming range: 0 to 100 lux."));
  Serial.println(F("Above 100 lux, LED PWM remains fixed at 32."));
  Serial.println(F("Movement enables a 1.0-second, 50%-duty hardware blink"));
  Serial.println(F("and each detected movement extends blinking for 3 seconds."));
}

void loop() {
  static uint32_t previousSampleMs = 0;
  static uint32_t previousDisplayMs = 0;
  static uint32_t previousErrorMs = 0;

  static uint16_t latestRawAls = 0;
  static float latestLux = 0.0f;
  static float latestXG = 0.0f;
  static float latestYG = 0.0f;
  static float latestZG = 0.0f;
  static float latestMagnitudeG = 0.0f;
  static float peakDeltaG = 0.0f;
  static bool motionDetectedDuringPeriod = false;
  static bool latestAlsOk = false;
  static bool latestAccelOk = false;
  static int latestAccIntLevel = LOW;
  static int latestAlsIntLevel = LOW;
  static uint8_t targetLedPwm = 0;

  if (g_lisAddress == 0 || g_vemlAddress == 0) {
    if (millis() - previousErrorMs >= 2000) {
      previousErrorMs = millis();
      Serial.println(F("ERROR: one or more sensors are unavailable."));
    }
    return;
  }

  const uint32_t nowMs = millis();

  // Sample frequently so short movement events are not lost merely because
  // the terminal output is intentionally slow.
  if (nowMs - previousSampleMs >= kSensorSamplePeriodMs) {
    previousSampleMs = nowMs;

    latestAccIntLevel = digitalRead(BoardPins::kAccInt);
    latestAlsIntLevel = digitalRead(BoardPins::kAlsInt);

    latestAlsOk = readAmbientLight(latestRawAls, latestLux);
    latestAccelOk = readAcceleration(latestXG, latestYG, latestZG);

    if (latestAlsOk) {
      if (!g_haveFilteredLux) {
        g_filteredLux = latestLux;
        g_haveFilteredLux = true;
      } else {
        g_filteredLux += kAmbientFilterAlpha * (latestLux - g_filteredLux);
      }

      targetLedPwm = calculateLedPwm(g_filteredLux);
    }

    if (latestAccelOk) {
      latestMagnitudeG = sqrtf(latestXG * latestXG +
                               latestYG * latestYG +
                               latestZG * latestZG);

      if (g_havePreviousAcceleration) {
        const float deltaX = latestXG - g_previousXG;
        const float deltaY = latestYG - g_previousYG;
        const float deltaZ = latestZG - g_previousZG;
        const float deltaG = sqrtf(deltaX * deltaX +
                                   deltaY * deltaY +
                                   deltaZ * deltaZ);

        if (deltaG > peakDeltaG) {
          peakDeltaG = deltaG;
        }
        if (deltaG >= kMotionDeltaThresholdG) {
          motionDetectedDuringPeriod = true;
          g_motionHasOccurred = true;
          g_lastMotionMs = nowMs;
        }
      }

      g_previousXG = latestXG;
      g_previousYG = latestYG;
      g_previousZG = latestZG;
      g_havePreviousAcceleration = true;
    }

    if (latestAlsOk && g_ledDriverAvailable) {
      (void)updateLedControl(nowMs, targetLedPwm);
    }
  }

  if (nowMs - previousDisplayMs < kDisplayPeriodMs) {
    return;
  }
  previousDisplayMs = nowMs;

  const bool motionBlinkActive =
      g_motionHasOccurred &&
      static_cast<uint32_t>(nowMs - g_lastMotionMs) < kMotionBlinkHoldMs;

  Serial.println();
  Serial.println(F("========================================"));
  Serial.print(F("Uptime:              "));
  Serial.print(nowMs / 1000.0f, 1);
  Serial.println(F(" s"));

  Serial.println(F("Ambient light:"));
  if (latestAlsOk) {
    Serial.print(F("  Illuminance:       "));
    Serial.print(latestLux, 1);
    Serial.println(F(" lux"));
    Serial.print(F("  Filtered value:    "));
    Serial.print(g_filteredLux, 1);
    Serial.println(F(" lux"));
    Serial.print(F("  Raw sensor value:  "));
    Serial.println(latestRawAls);
    Serial.print(F("  Sensor saturated:  "));
    Serial.println(latestRawAls >= 65000U ? F("YES") : F("no"));
  } else {
    Serial.println(F("  ERROR: VEML6030 read failed"));
  }

  Serial.println(F("Motion sensor:"));
  if (latestAccelOk) {
    Serial.print(F("  X acceleration:    "));
    Serial.print(latestXG, 3);
    Serial.println(F(" g"));
    Serial.print(F("  Y acceleration:    "));
    Serial.print(latestYG, 3);
    Serial.println(F(" g"));
    Serial.print(F("  Z acceleration:    "));
    Serial.print(latestZG, 3);
    Serial.println(F(" g"));
    Serial.print(F("  Vector magnitude:  "));
    Serial.print(latestMagnitudeG, 3);
    Serial.println(F(" g"));
    Serial.print(F("  Peak change:       "));
    Serial.print(peakDeltaG, 3);
    Serial.println(F(" g"));
    Serial.print(F("  Motion detected:   "));
    Serial.println(motionDetectedDuringPeriod ? F("YES") : F("no"));
  } else {
    Serial.println(F("  ERROR: LIS2HH12 read failed"));
  }

  Serial.println(F("LED output:"));
  if (!g_ledDriverAvailable) {
    Serial.println(F("  Driver status:     unavailable"));
  } else {
    Serial.print(F("  Driver status:     "));
    Serial.println(g_ledWriteHealthy ? F("OK") : F("I2C write error"));
    Serial.print(F("  Target PWM:        "));
    Serial.print(targetLedPwm);
    Serial.print(F(" / 255 ("));
    Serial.print((static_cast<float>(targetLedPwm) * 100.0f) / 255.0f, 1);
    Serial.println(F("%)"));
    Serial.print(F("  Applied PWM:       "));
    Serial.println(g_appliedLedPwm);
    Serial.print(F("  Ambient response:  "));
    if (g_filteredLux >= kLuxAtPresetLedOutput) {
      Serial.println(F("above 100 lux - preset output"));
    } else if (g_filteredLux <= kLuxAtMaximumLedOutput) {
      Serial.println(F("dark - maximum LED output"));
    } else {
      Serial.println(F("automatic dimming, 0-100 lux"));
    }
    Serial.print(F("  Output mode:       "));
    Serial.println(motionBlinkActive
                       ? F("BLINKING - 1.0 s period, 50% duty")
                       : F("steady"));
  }

  Serial.println(F("Interrupt inputs:"));
  Serial.print(F("  ACC_INT (PB8):     "));
  Serial.println(latestAccIntLevel == HIGH ? F("HIGH") : F("LOW"));
  Serial.print(F("  ALS_INT (PB9):     "));
  Serial.println(latestAlsIntLevel == HIGH ? F("HIGH") : F("LOW"));

  // Start a fresh motion-observation window for the next displayed report.
  peakDeltaG = 0.0f;
  motionDetectedDuringPeriod = false;
}
