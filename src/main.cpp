#include <Arduino.h>
#include <DallasTemperature.h>
#include <DNSServer.h>
#include <ModbusRTU.h>
#include <OneWire.h>
#include <Preferences.h>
#include <WebServer.h>
#include <WiFi.h>
#include <Wire.h>

static const uint8_t I2C_SDA = 4;
static const uint8_t I2C_SCL = 5;
static const uint8_t RELAY_EXPANDER_1 = 0x24; // Channels 1-8
static const uint8_t RELAY_EXPANDER_2 = 0x25; // Channels 9-16
static const uint8_t INPUT_EXPANDER = 0x22;
static const uint8_t FLOAT_SENSOR_PORT = 0;
static const uint8_t FLOAT_SENSOR_BIT = 7;
static const bool FLOAT_SENSOR_ACTIVE_LOW = true;
static const uint8_t DALLAS_PIN = 32;
static const uint32_t TEMPERATURE_READ_INTERVAL_MS = 1000;
static const uint8_t DRIVE_COUNT = 3;
static const uint8_t RELAY_CHANNEL_COUNT = 16;
static const uint16_t RELAY_DEADTIME_MS = 120;
static const uint8_t FILL_VALVE_CHANNEL = 9;
static const uint8_t DRAIN_VALVE_CHANNEL = 10;
static const uint8_t HEATER_PHASE_A_CHANNEL = 11;
static const uint8_t HEATER_PHASE_B_CHANNEL = 12;
static const uint8_t HEATER_PHASE_C_CHANNEL = 13;

static const uint8_t RS485_RX_PIN = 16;
static const uint8_t RS485_TX_PIN = 13;
static const int8_t RS485_DE_PIN = -1; // -1 = no DE/RE pin, transceiver must use auto-direction or fixed RX/TX enable
static const uint32_t RS485_BAUDRATE = 9600;
static const uint8_t MODBUS_SLAVE_ID = 1;

const char* ap_ssid = "dumplings";
const char* ap_password = "clickclick";

uint32_t lowerTimeMs = 3500;
uint32_t raiseFullTimeMs = 3500;
uint32_t raiseBeforeShakeTimeMs = 1750;
uint32_t shakeUpTimeMs = 250;
uint32_t shakeDownTimeMs = 250;
uint8_t shakeCount = 8;

WebServer server(80);
DNSServer dnsServer;
HardwareSerial RS485Serial(2);
ModbusRTU mb;
Preferences preferences;
OneWire oneWire(DALLAS_PIN);
DallasTemperature dallasSensors(&oneWire);

static const uint16_t DNS_PORT = 53;

enum ModbusReg : uint16_t {
  REG_GLOBAL_STATUS = 0,       // RO bit0 any_busy
  REG_COMMAND = 1,             // RW, resets to 0 after processing
  REG_COMMAND_DRIVE = 2,       // RW 1..3, 0 = all for stop
  REG_COMMAND_RESULT = 3,      // RO
  REG_LOWER_TIME_MS = 4,       // RW
  REG_RAISE_FULL_TIME_MS = 5,  // RW
  REG_RAISE_BEFORE_SHAKE_MS = 6, // RW
  REG_SHAKE_UP_TIME_MS = 7,    // RW
  REG_SHAKE_DOWN_TIME_MS = 8,  // RW
  REG_SHAKE_COUNT = 9,         // RW
  REG_DRIVE_BASE = 10,         // 5 regs per drive: status, sequence, direction, remaining_ms, position
  REG_VALVE_STATUS = 25,       // RO bit0 fill, bit1 drain
  REG_VALVE_CONTROL = 26,      // RW bit0 fill, bit1 drain
  REG_TEMPERATURE_X10 = 27,    // RO int16, 0x8000 = no data
  REG_DALLAS_COUNT = 28,       // RO
  REG_HEATER_STATUS = 29,      // RO 0/1
  REG_HEATER_CONTROL = 30,     // RW 0/1
  REG_REG_COUNT = 32
};

enum ModbusCommand : uint16_t {
  CMD_NONE = 0,
  CMD_LOWER = 1,
  CMD_RAISE = 2,
  CMD_STOP = 3,
  CMD_SAVE_SETTINGS = 4,
  CMD_STARTUP_RAISE = 5
};

enum ModbusCommandResult : uint16_t {
  CMD_RES_OK = 0,
  CMD_RES_UNKNOWN_CMD = 1,
  CMD_RES_BUSY = 2,
  CMD_RES_INVALID_ARG = 3
};

enum LiftDirection : uint8_t {
  LIFT_STOP = 0,
  LIFT_UP = 1,
  LIFT_DOWN = 2
};

enum LiftSequence : uint8_t {
  SEQUENCE_IDLE = 0,
  SEQUENCE_LOWERING = 1,
  SEQUENCE_RAISE_INITIAL = 2,
  SEQUENCE_SHAKE_UP = 3,
  SEQUENCE_SHAKE_DOWN = 4,
  SEQUENCE_RAISE_FINAL = 5,
  SEQUENCE_DONE = 6
};

enum LiftPosition : uint8_t {
  POSITION_UNKNOWN = 0,
  POSITION_TOP = 1,
  POSITION_BOTTOM = 2
};

struct DriveConfig {
  uint8_t upChannel;
  uint8_t downChannel;
};

struct DriveState {
  LiftDirection direction;
  LiftSequence sequence;
  LiftPosition position;
  uint32_t stepStartedAtMs;
  uint32_t stepDurationMs;
  uint32_t sequenceFinishedAtMs;
  uint8_t completedShakeCycles;
};

DriveConfig driveConfigs[DRIVE_COUNT] = {
  {1, 2},
  {3, 4},
  {5, 6}
};

DriveState drives[DRIVE_COUNT];
bool relayOn[RELAY_CHANNEL_COUNT] = {false};
bool floatSensorActive = false;
uint8_t inputExpanderRaw = 0xFF;
bool fillValveOn = false;
bool drainValveOn = false;
bool modbusReady = false;
bool heaterOn = false;
float waterTemperatureC = NAN;
uint8_t dallasDeviceCount = 0;
uint32_t lastTemperatureReadMs = 0;
bool temperatureConversionPending = false;
uint32_t temperatureConversionStartMs = 0;

void syncModbusRegistersFromState();
void syncValveModbusRegisters();
void syncHeaterModbusRegisters();
bool heaterInterlockOk();
void enforceHeaterInterlock();

uint16_t clampU16(uint32_t value) {
  return value > 65535 ? 65535 : static_cast<uint16_t>(value);
}

uint16_t floatToI16Reg(float value, float scale) {
  int32_t scaled = static_cast<int32_t>(lroundf(value * scale));
  if (scaled < -32768) {
    scaled = -32768;
  }
  if (scaled > 32767) {
    scaled = 32767;
  }
  return static_cast<uint16_t>(static_cast<int16_t>(scaled));
}

bool validDriveIndex(uint8_t driveIndex) {
  return driveIndex < DRIVE_COUNT;
}

bool validTimingSettings(
  uint32_t lowerMs,
  uint32_t raiseFullMs,
  uint32_t raiseBeforeShakeMs,
  uint32_t shakeUpMs,
  uint32_t shakeDownMs,
  uint16_t shakes
) {
  return lowerMs > 0 &&
         raiseFullMs > 0 &&
         raiseBeforeShakeMs <= raiseFullMs &&
         shakeUpMs <= 65535 &&
         shakeDownMs <= 65535 &&
         shakes <= 100;
}

void loadTimingSettings() {
  preferences.begin("lift", true);
  uint32_t savedLowerTimeMs = preferences.getUInt("lower", lowerTimeMs);
  uint32_t savedRaiseFullTimeMs = preferences.getUInt("raiseFull", raiseFullTimeMs);
  uint32_t savedRaiseBeforeShakeTimeMs = preferences.getUInt("raisePre", raiseBeforeShakeTimeMs);
  uint32_t savedShakeUpTimeMs = preferences.getUInt("shakeUp", shakeUpTimeMs);
  uint32_t savedShakeDownTimeMs = preferences.getUInt("shakeDown", shakeDownTimeMs);
  uint16_t savedShakeCount = preferences.getUShort("shakeCnt", shakeCount);
  preferences.end();

  if (!validTimingSettings(
        savedLowerTimeMs,
        savedRaiseFullTimeMs,
        savedRaiseBeforeShakeTimeMs,
        savedShakeUpTimeMs,
        savedShakeDownTimeMs,
        savedShakeCount
      )) {
    return;
  }

  lowerTimeMs = savedLowerTimeMs;
  raiseFullTimeMs = savedRaiseFullTimeMs;
  raiseBeforeShakeTimeMs = savedRaiseBeforeShakeTimeMs;
  shakeUpTimeMs = savedShakeUpTimeMs;
  shakeDownTimeMs = savedShakeDownTimeMs;
  shakeCount = static_cast<uint8_t>(savedShakeCount);
}

void saveTimingSettings() {
  preferences.begin("lift", false);
  preferences.putUInt("lower", lowerTimeMs);
  preferences.putUInt("raiseFull", raiseFullTimeMs);
  preferences.putUInt("raisePre", raiseBeforeShakeTimeMs);
  preferences.putUInt("shakeUp", shakeUpTimeMs);
  preferences.putUInt("shakeDown", shakeDownTimeMs);
  preferences.putUShort("shakeCnt", shakeCount);
  preferences.end();
}

void writeExpander(uint8_t address, uint8_t value) {
  Wire.beginTransmission(address);
  Wire.write(value);
  Wire.endTransmission();
}

void initInputExpander() {
  Wire.beginTransmission(INPUT_EXPANDER);
  Wire.write(0xFF);
  Wire.endTransmission();
}

void updateFloatSensor() {
  Wire.requestFrom(INPUT_EXPANDER, static_cast<uint8_t>(1));
  if (Wire.available() < 1) {
    return;
  }

  inputExpanderRaw = Wire.read();
  bool bitState = bitRead(inputExpanderRaw, FLOAT_SENSOR_BIT);
  floatSensorActive = FLOAT_SENSOR_ACTIVE_LOW ? !bitState : bitState;
}

void updateDallasTemperature(uint32_t nowMs) {
  if (temperatureConversionPending) {
    if (nowMs - temperatureConversionStartMs < 750) {
      return;
    }

    float temperatureC = dallasSensors.getTempCByIndex(0);
    if (temperatureC == DEVICE_DISCONNECTED_C) {
      waterTemperatureC = NAN;
    } else {
      waterTemperatureC = temperatureC;
    }
    temperatureConversionPending = false;
    return;
  }

  if (lastTemperatureReadMs != 0 && nowMs - lastTemperatureReadMs < TEMPERATURE_READ_INTERVAL_MS) {
    return;
  }

  lastTemperatureReadMs = nowMs;
  dallasSensors.requestTemperatures();
  temperatureConversionStartMs = nowMs;
  temperatureConversionPending = true;
}

void writeRelayOutputs() {
  uint8_t expander1Value = 0xFF;
  uint8_t expander2Value = 0xFF;

  for (uint8_t logicalChannel = 1; logicalChannel <= RELAY_CHANNEL_COUNT; logicalChannel++) {
    if (!relayOn[logicalChannel - 1]) {
      continue;
    }

    uint8_t hardwareChannel = (RELAY_CHANNEL_COUNT + 1) - logicalChannel;
    if (hardwareChannel <= 8) {
      bitClear(expander1Value, hardwareChannel - 1);
    } else {
      bitClear(expander2Value, hardwareChannel - 9);
    }
  }

  writeExpander(RELAY_EXPANDER_1, expander1Value);
  writeExpander(RELAY_EXPANDER_2, expander2Value);
}

void setRelayChannel(uint8_t channel, bool enabled) {
  if (channel < 1 || channel > RELAY_CHANNEL_COUNT) {
    return;
  }

  relayOn[channel - 1] = enabled;
}

void setAllRelaysOff() {
  for (uint8_t i = 0; i < RELAY_CHANNEL_COUNT; i++) {
    relayOn[i] = false;
  }
  fillValveOn = false;
  drainValveOn = false;
  heaterOn = false;
  writeRelayOutputs();
}

void initializeRelayExpandersSafe() {
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);
  initInputExpander();

  for (uint8_t i = 0; i < 3; i++) {
    setAllRelaysOff();
    delay(5);
  }
}

void writeDriveRelays(uint8_t driveIndex, bool upOn, bool downOn) {
  if (!validDriveIndex(driveIndex)) {
    return;
  }

  setRelayChannel(driveConfigs[driveIndex].upChannel, upOn);
  setRelayChannel(driveConfigs[driveIndex].downChannel, downOn);
  writeRelayOutputs();
}

void setFillValve(bool enabled) {
  fillValveOn = enabled;
  setRelayChannel(FILL_VALVE_CHANNEL, enabled);
  enforceHeaterInterlock();
  writeRelayOutputs();
  syncValveModbusRegisters();
}

void setDrainValve(bool enabled) {
  drainValveOn = enabled;
  setRelayChannel(DRAIN_VALVE_CHANNEL, enabled);
  enforceHeaterInterlock();
  writeRelayOutputs();
  syncValveModbusRegisters();
}

uint16_t valveStatusBits() {
  uint16_t bits = 0;
  if (fillValveOn) {
    bits |= (1U << 0);
  }
  if (drainValveOn) {
    bits |= (1U << 1);
  }
  return bits;
}

bool heaterInterlockOk() {
  return floatSensorActive && fillValveOn && drainValveOn;
}

void enforceHeaterInterlock() {
  if (!heaterInterlockOk()) {
    heaterOn = false;
    setRelayChannel(HEATER_PHASE_A_CHANNEL, false);
    setRelayChannel(HEATER_PHASE_B_CHANNEL, false);
    setRelayChannel(HEATER_PHASE_C_CHANNEL, false);
  }
}

void setHeater(bool enabled) {
  if (enabled && !heaterInterlockOk()) {
    heaterOn = false;
    setRelayChannel(HEATER_PHASE_A_CHANNEL, false);
    setRelayChannel(HEATER_PHASE_B_CHANNEL, false);
    setRelayChannel(HEATER_PHASE_C_CHANNEL, false);
    writeRelayOutputs();
    syncHeaterModbusRegisters();
    return;
  }

  heaterOn = enabled;
  setRelayChannel(HEATER_PHASE_A_CHANNEL, enabled);
  setRelayChannel(HEATER_PHASE_B_CHANNEL, enabled);
  setRelayChannel(HEATER_PHASE_C_CHANNEL, enabled);
  writeRelayOutputs();
  syncHeaterModbusRegisters();
}

void syncValveModbusRegisters() {
  if (!modbusReady) {
    return;
  }

  uint16_t bits = valveStatusBits();
  mb.Hreg(REG_VALVE_STATUS, bits);
  mb.Hreg(REG_VALVE_CONTROL, bits);
}

void syncHeaterModbusRegisters() {
  if (!modbusReady) {
    return;
  }

  uint16_t value = heaterOn ? 1 : 0;
  mb.Hreg(REG_HEATER_STATUS, value);
  mb.Hreg(REG_HEATER_CONTROL, value);
}

void stopDrive(uint8_t driveIndex) {
  if (!validDriveIndex(driveIndex)) {
    return;
  }

  writeDriveRelays(driveIndex, false, false);
  drives[driveIndex].direction = LIFT_STOP;
}

void setDriveDirectionSafe(uint8_t driveIndex, LiftDirection direction) {
  if (!validDriveIndex(driveIndex) || drives[driveIndex].direction == direction) {
    return;
  }

  stopDrive(driveIndex);
  delay(RELAY_DEADTIME_MS);

  if (direction == LIFT_UP) {
    writeDriveRelays(driveIndex, true, false);
  } else if (direction == LIFT_DOWN) {
    writeDriveRelays(driveIndex, false, true);
  }

  drives[driveIndex].direction = direction;
}

bool driveBusy(uint8_t driveIndex) {
  return validDriveIndex(driveIndex) &&
         drives[driveIndex].sequence != SEQUENCE_IDLE &&
         drives[driveIndex].sequence != SEQUENCE_DONE;
}

bool anyDriveBusy() {
  for (uint8_t i = 0; i < DRIVE_COUNT; i++) {
    if (driveBusy(i)) {
      return true;
    }
  }
  return false;
}

bool canLower(uint8_t driveIndex) {
  return validDriveIndex(driveIndex) && !driveBusy(driveIndex) && drives[driveIndex].position != POSITION_BOTTOM;
}

bool canRaise(uint8_t driveIndex) {
  return validDriveIndex(driveIndex) && !driveBusy(driveIndex) && drives[driveIndex].position != POSITION_TOP;
}

void resetDriveDoneState(uint8_t driveIndex) {
  drives[driveIndex].completedShakeCycles = 0;
  drives[driveIndex].sequenceFinishedAtMs = 0;
}

void startDriveStep(uint8_t driveIndex, LiftSequence sequence, LiftDirection direction, uint32_t durationMs) {
  DriveState& drive = drives[driveIndex];
  drive.sequence = sequence;
  drive.stepStartedAtMs = millis();
  drive.stepDurationMs = durationMs;
  setDriveDirectionSafe(driveIndex, direction);
}

void finishDriveSequence(uint8_t driveIndex) {
  DriveState& drive = drives[driveIndex];
  LiftSequence finishedSequence = drive.sequence;
  stopDrive(driveIndex);
  drive.sequence = SEQUENCE_DONE;
  drive.sequenceFinishedAtMs = millis();

  if (finishedSequence == SEQUENCE_LOWERING) {
    drive.position = POSITION_BOTTOM;
  } else if (finishedSequence == SEQUENCE_RAISE_FINAL) {
    drive.position = POSITION_TOP;
  }
}

void startLowerSequence(uint8_t driveIndex) {
  if (!canLower(driveIndex)) {
    return;
  }

  drives[driveIndex].position = POSITION_UNKNOWN;
  resetDriveDoneState(driveIndex);
  startDriveStep(driveIndex, SEQUENCE_LOWERING, LIFT_DOWN, lowerTimeMs);
}

void startRaiseSequence(uint8_t driveIndex) {
  if (!canRaise(driveIndex)) {
    return;
  }

  drives[driveIndex].position = POSITION_UNKNOWN;
  resetDriveDoneState(driveIndex);
  startDriveStep(driveIndex, SEQUENCE_RAISE_INITIAL, LIFT_UP, raiseBeforeShakeTimeMs);
}

void startStartupRaiseSequence(uint8_t driveIndex) {
  if (!canRaise(driveIndex)) {
    return;
  }

  drives[driveIndex].position = POSITION_UNKNOWN;
  resetDriveDoneState(driveIndex);
  startDriveStep(driveIndex, SEQUENCE_RAISE_FINAL, LIFT_UP, raiseFullTimeMs);
}

void stopDriveAndForgetPosition(uint8_t driveIndex) {
  if (!validDriveIndex(driveIndex)) {
    return;
  }

  stopDrive(driveIndex);
  drives[driveIndex].sequence = SEQUENCE_IDLE;
  drives[driveIndex].sequenceFinishedAtMs = 0;
  drives[driveIndex].position = POSITION_UNKNOWN;
}

void updateDriveSequence(uint8_t driveIndex) {
  if (!validDriveIndex(driveIndex)) {
    return;
  }

  DriveState& drive = drives[driveIndex];
  if (!driveBusy(driveIndex)) {
    if (drive.sequence == SEQUENCE_DONE && millis() - drive.sequenceFinishedAtMs > 2500) {
      drive.sequence = SEQUENCE_IDLE;
    }
    return;
  }

  if (millis() - drive.stepStartedAtMs < drive.stepDurationMs) {
    return;
  }

  if (drive.sequence == SEQUENCE_LOWERING) {
    finishDriveSequence(driveIndex);
    return;
  }

  if (drive.sequence == SEQUENCE_RAISE_INITIAL) {
    if (shakeCount == 0) {
      uint32_t finalRaiseTimeMs = raiseFullTimeMs > raiseBeforeShakeTimeMs ? raiseFullTimeMs - raiseBeforeShakeTimeMs : 0;
      startDriveStep(driveIndex, SEQUENCE_RAISE_FINAL, LIFT_UP, finalRaiseTimeMs);
      return;
    }

    startDriveStep(driveIndex, SEQUENCE_SHAKE_DOWN, LIFT_DOWN, shakeDownTimeMs);
    return;
  }

  if (drive.sequence == SEQUENCE_SHAKE_DOWN) {
    startDriveStep(driveIndex, SEQUENCE_SHAKE_UP, LIFT_UP, shakeUpTimeMs);
    return;
  }

  if (drive.sequence == SEQUENCE_SHAKE_UP) {
    drive.completedShakeCycles++;
    if (drive.completedShakeCycles < shakeCount) {
      startDriveStep(driveIndex, SEQUENCE_SHAKE_DOWN, LIFT_DOWN, shakeDownTimeMs);
      return;
    }

    uint32_t finalRaiseTimeMs = raiseFullTimeMs > raiseBeforeShakeTimeMs ? raiseFullTimeMs - raiseBeforeShakeTimeMs : 0;
    startDriveStep(driveIndex, SEQUENCE_RAISE_FINAL, LIFT_UP, finalRaiseTimeMs);
    return;
  }

  if (drive.sequence == SEQUENCE_RAISE_FINAL) {
    finishDriveSequence(driveIndex);
  }
}

uint32_t remainingStepMs(uint8_t driveIndex) {
  if (!driveBusy(driveIndex)) {
    return 0;
  }

  DriveState& drive = drives[driveIndex];
  uint32_t elapsedMs = millis() - drive.stepStartedAtMs;
  if (elapsedMs >= drive.stepDurationMs) {
    return 0;
  }
  return drive.stepDurationMs - elapsedMs;
}

const char* directionText(LiftDirection direction) {
  if (direction == LIFT_UP) {
    return "up";
  }
  if (direction == LIFT_DOWN) {
    return "down";
  }
  return "stop";
}

const char* sequenceText(LiftSequence sequence) {
  if (sequence == SEQUENCE_LOWERING) {
    return "lowering";
  }
  if (sequence == SEQUENCE_RAISE_INITIAL) {
    return "raise_initial";
  }
  if (sequence == SEQUENCE_SHAKE_UP || sequence == SEQUENCE_SHAKE_DOWN) {
    return "shake";
  }
  if (sequence == SEQUENCE_RAISE_FINAL) {
    return "raise_final";
  }
  if (sequence == SEQUENCE_DONE) {
    return "done";
  }
  return "idle";
}

const char* positionText(LiftPosition position) {
  if (position == POSITION_TOP) {
    return "top";
  }
  if (position == POSITION_BOTTOM) {
    return "bottom";
  }
  return "unknown";
}

uint16_t driveStatusBits(uint8_t driveIndex) {
  uint16_t statusBits = 0;
  if (driveBusy(driveIndex)) {
    statusBits |= (1U << 0);
  }
  if (drives[driveIndex].sequence == SEQUENCE_DONE) {
    statusBits |= (1U << 1);
  }
  if (drives[driveIndex].position == POSITION_TOP) {
    statusBits |= (1U << 2);
  }
  if (drives[driveIndex].position == POSITION_BOTTOM) {
    statusBits |= (1U << 3);
  }
  if (relayOn[driveConfigs[driveIndex].upChannel - 1]) {
    statusBits |= (1U << 4);
  }
  if (relayOn[driveConfigs[driveIndex].downChannel - 1]) {
    statusBits |= (1U << 5);
  }
  return statusBits;
}

void syncModbusRegistersFromState() {
  uint16_t globalStatus = 0;
  if (anyDriveBusy()) {
    globalStatus |= (1U << 0);
  }
  if (heaterInterlockOk()) {
    globalStatus |= (1U << 1);
  }
  if (!heaterInterlockOk()) {
    globalStatus |= (1U << 2);
  }
  mb.Hreg(REG_GLOBAL_STATUS, globalStatus);
  mb.Hreg(REG_LOWER_TIME_MS, clampU16(lowerTimeMs));
  mb.Hreg(REG_RAISE_FULL_TIME_MS, clampU16(raiseFullTimeMs));
  mb.Hreg(REG_RAISE_BEFORE_SHAKE_MS, clampU16(raiseBeforeShakeTimeMs));
  mb.Hreg(REG_SHAKE_UP_TIME_MS, clampU16(shakeUpTimeMs));
  mb.Hreg(REG_SHAKE_DOWN_TIME_MS, clampU16(shakeDownTimeMs));
  mb.Hreg(REG_SHAKE_COUNT, shakeCount);
  mb.Hreg(REG_VALVE_STATUS, valveStatusBits());
  mb.Hreg(REG_VALVE_CONTROL, valveStatusBits());
  uint16_t temperatureReg = 0x8000;
  if (!isnan(waterTemperatureC)) {
    temperatureReg = floatToI16Reg(waterTemperatureC, 10.0f);
  }
  mb.Hreg(REG_TEMPERATURE_X10, temperatureReg);
  mb.Hreg(REG_DALLAS_COUNT, dallasDeviceCount);
  mb.Hreg(REG_HEATER_STATUS, heaterOn ? 1 : 0);
  mb.Hreg(REG_HEATER_CONTROL, heaterOn ? 1 : 0);

  for (uint8_t driveIndex = 0; driveIndex < DRIVE_COUNT; driveIndex++) {
    uint16_t base = REG_DRIVE_BASE + (driveIndex * 5);
    mb.Hreg(base + 0, driveStatusBits(driveIndex));
    mb.Hreg(base + 1, static_cast<uint16_t>(drives[driveIndex].sequence));
    mb.Hreg(base + 2, static_cast<uint16_t>(drives[driveIndex].direction));
    mb.Hreg(base + 3, clampU16(remainingStepMs(driveIndex)));
    mb.Hreg(base + 4, static_cast<uint16_t>(drives[driveIndex].position));
  }
}

void applyModbusWritableRegisters() {
  uint16_t requestedValveControl = mb.Hreg(REG_VALVE_CONTROL);
  bool requestedFillValveOn = (requestedValveControl & (1U << 0)) != 0;
  bool requestedDrainValveOn = (requestedValveControl & (1U << 1)) != 0;
  if (requestedFillValveOn != fillValveOn) {
    setFillValve(requestedFillValveOn);
  }
  if (requestedDrainValveOn != drainValveOn) {
    setDrainValve(requestedDrainValveOn);
  }

  bool requestedHeaterOn = mb.Hreg(REG_HEATER_CONTROL) != 0;
  if (requestedHeaterOn && !heaterInterlockOk()) {
    mb.Hreg(REG_COMMAND_RESULT, CMD_RES_INVALID_ARG);
    setHeater(false);
  } else if (requestedHeaterOn != heaterOn) {
    setHeater(requestedHeaterOn);
  }

  uint32_t requestedLowerTimeMs = mb.Hreg(REG_LOWER_TIME_MS);
  uint32_t requestedRaiseFullTimeMs = mb.Hreg(REG_RAISE_FULL_TIME_MS);
  uint32_t requestedRaiseBeforeShakeMs = mb.Hreg(REG_RAISE_BEFORE_SHAKE_MS);
  uint32_t requestedShakeUpTimeMs = mb.Hreg(REG_SHAKE_UP_TIME_MS);
  uint32_t requestedShakeDownTimeMs = mb.Hreg(REG_SHAKE_DOWN_TIME_MS);
  uint16_t requestedShakeCount = mb.Hreg(REG_SHAKE_COUNT);

  bool changed = requestedLowerTimeMs != lowerTimeMs ||
                 requestedRaiseFullTimeMs != raiseFullTimeMs ||
                 requestedRaiseBeforeShakeMs != raiseBeforeShakeTimeMs ||
                 requestedShakeUpTimeMs != shakeUpTimeMs ||
                 requestedShakeDownTimeMs != shakeDownTimeMs ||
                 requestedShakeCount != shakeCount;

  if (!changed) {
    return;
  }

  if (anyDriveBusy()) {
    mb.Hreg(REG_COMMAND_RESULT, CMD_RES_BUSY);
    return;
  }

  if (!validTimingSettings(
        requestedLowerTimeMs,
        requestedRaiseFullTimeMs,
        requestedRaiseBeforeShakeMs,
        requestedShakeUpTimeMs,
        requestedShakeDownTimeMs,
        requestedShakeCount
      )) {
    mb.Hreg(REG_COMMAND_RESULT, CMD_RES_INVALID_ARG);
    return;
  }

  lowerTimeMs = requestedLowerTimeMs;
  raiseFullTimeMs = requestedRaiseFullTimeMs;
  raiseBeforeShakeTimeMs = requestedRaiseBeforeShakeMs;
  shakeUpTimeMs = requestedShakeUpTimeMs;
  shakeDownTimeMs = requestedShakeDownTimeMs;
  shakeCount = static_cast<uint8_t>(requestedShakeCount);
  mb.Hreg(REG_COMMAND_RESULT, CMD_RES_OK);
}

uint16_t handleDriveCommand(uint16_t cmd, uint8_t driveIndex) {
  if (!validDriveIndex(driveIndex)) {
    return CMD_RES_INVALID_ARG;
  }

  if (cmd == CMD_LOWER) {
    if (driveBusy(driveIndex)) {
      return CMD_RES_BUSY;
    }
    if (!canLower(driveIndex)) {
      return CMD_RES_INVALID_ARG;
    }
    startLowerSequence(driveIndex);
    return CMD_RES_OK;
  }

  if (cmd == CMD_RAISE) {
    if (driveBusy(driveIndex)) {
      return CMD_RES_BUSY;
    }
    if (!canRaise(driveIndex)) {
      return CMD_RES_INVALID_ARG;
    }
    startRaiseSequence(driveIndex);
    return CMD_RES_OK;
  }

  if (cmd == CMD_STARTUP_RAISE) {
    if (driveBusy(driveIndex)) {
      return CMD_RES_BUSY;
    }
    if (!canRaise(driveIndex)) {
      return CMD_RES_INVALID_ARG;
    }
    startStartupRaiseSequence(driveIndex);
    return CMD_RES_OK;
  }

  return CMD_RES_UNKNOWN_CMD;
}

void handleModbusCommand() {
  uint16_t cmd = mb.Hreg(REG_COMMAND);
  if (cmd == CMD_NONE) {
    return;
  }

  uint16_t result = CMD_RES_OK;
  uint16_t driveArg = mb.Hreg(REG_COMMAND_DRIVE);

  if (cmd == CMD_STOP) {
    if (driveArg == 0) {
      for (uint8_t i = 0; i < DRIVE_COUNT; i++) {
        stopDriveAndForgetPosition(i);
      }
    } else if (driveArg >= 1 && driveArg <= DRIVE_COUNT) {
      stopDriveAndForgetPosition(static_cast<uint8_t>(driveArg - 1));
    } else {
      result = CMD_RES_INVALID_ARG;
    }
  } else if (cmd == CMD_SAVE_SETTINGS) {
    if (anyDriveBusy()) {
      result = CMD_RES_BUSY;
    } else {
      saveTimingSettings();
    }
  } else if (driveArg >= 1 && driveArg <= DRIVE_COUNT) {
    result = handleDriveCommand(cmd, static_cast<uint8_t>(driveArg - 1));
  } else {
    result = CMD_RES_INVALID_ARG;
  }

  mb.Hreg(REG_COMMAND_RESULT, result);
  mb.Hreg(REG_COMMAND, CMD_NONE);
}

void initDriveStates() {
  for (uint8_t i = 0; i < DRIVE_COUNT; i++) {
    drives[i].direction = LIFT_STOP;
    drives[i].sequence = SEQUENCE_IDLE;
    drives[i].position = POSITION_UNKNOWN;
    drives[i].stepStartedAtMs = 0;
    drives[i].stepDurationMs = 0;
    drives[i].sequenceFinishedAtMs = 0;
    drives[i].completedShakeCycles = 0;
  }
}

void closeValvesByDefault() {
  setFillValve(true);
  setDrainValve(true);
}

String driveJson(uint8_t driveIndex) {
  String json = "{";
  json += "\"index\":" + String(driveIndex + 1) + ",";
  json += "\"sequence\":\"" + String(sequenceText(drives[driveIndex].sequence)) + "\",";
  json += "\"direction\":\"" + String(directionText(drives[driveIndex].direction)) + "\",";
  json += "\"position\":\"" + String(positionText(drives[driveIndex].position)) + "\",";
  json += "\"remainingSeconds\":" + String((remainingStepMs(driveIndex) + 999) / 1000) + ",";
  json += "\"busy\":" + String(driveBusy(driveIndex) ? "true" : "false") + ",";
  json += "\"canLower\":" + String(canLower(driveIndex) ? "true" : "false") + ",";
  json += "\"canRaise\":" + String(canRaise(driveIndex) ? "true" : "false");
  json += "}";
  return json;
}

String statusJson() {
  String json = "{";
  json += "\"anyBusy\":" + String(anyDriveBusy() ? "true" : "false") + ",";
  json += "\"floatSensorActive\":" + String(floatSensorActive ? "true" : "false") + ",";
  json += "\"heaterAllowed\":" + String(heaterInterlockOk() ? "true" : "false") + ",";
  json += "\"inputRaw\":" + String(inputExpanderRaw) + ",";
  json += "\"temperatureC\":";
  json += isnan(waterTemperatureC) ? "null" : String(waterTemperatureC, 2);
  json += ",";
  json += "\"dallasDeviceCount\":" + String(dallasDeviceCount) + ",";
  json += "\"fillValveOn\":" + String(fillValveOn ? "true" : "false") + ",";
  json += "\"drainValveOn\":" + String(drainValveOn ? "true" : "false") + ",";
  json += "\"heaterOn\":" + String(heaterOn ? "true" : "false") + ",";
  json += "\"drives\":[";
  for (uint8_t i = 0; i < DRIVE_COUNT; i++) {
    if (i > 0) {
      json += ",";
    }
    json += driveJson(i);
  }
  json += "]}";
  return json;
}

String generateHTML() {
  String html = R"rawliteral(
<!doctype html>
<html lang="ru">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>Dumplings lift</title>
  <style>
    :root {
      --bg: #f3f4f6;
      --card: #ffffff;
      --txt: #111827;
      --muted: #4b5563;
      --line: #d1d5db;
      --up: #166534;
      --down: #b91c1c;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      min-height: 100vh;
      padding: 16px;
      font-family: "Segoe UI", Tahoma, sans-serif;
      color: var(--txt);
      background: #f3f4f6;
    }
    main {
      width: min(960px, 100%);
      margin: 0 auto;
    }
    h1 { margin: 0 0 16px; font-size: 26px; }
    .grid {
      display: grid;
      grid-template-columns: repeat(3, minmax(0, 1fr));
      gap: 12px;
    }
    .panel {
      background: var(--card);
      border: 1px solid var(--line);
      border-radius: 10px;
      padding: 16px;
      box-shadow: 0 10px 24px rgba(17, 24, 39, 0.08);
    }
    h2 { margin: 0 0 12px; font-size: 20px; }
    .status {
      margin-bottom: 10px;
      padding: 10px;
      border-radius: 8px;
      border: 1px solid var(--line);
      background: #f9fafb;
      color: var(--muted);
      font-size: 14px;
    }
    .status b { color: var(--txt); }
    .buttons {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 8px;
      margin-top: 12px;
    }
    button {
      width: 100%;
      min-height: 52px;
      border: 0;
      border-radius: 8px;
      color: #fff;
      font-size: 18px;
      font-weight: 700;
      cursor: pointer;
    }
    button:disabled {
      opacity: 0.45;
      cursor: not-allowed;
    }
    .down { background: var(--down); }
    .up { background: var(--up); }
    .valves {
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 12px;
      margin: 12px 0;
    }
    .valve-on { background: #1d4ed8; }
    .valve-off { background: #6b7280; }
    .heater-on { background: #b91c1c; }
    .heater-off { background: #6b7280; }
    @media (max-width: 760px) {
      .grid, .valves { grid-template-columns: 1fr; }
    }
  </style>
</head>
<body>
  <main>
    <h1>&#1055;&#1077;&#1083;&#1100;&#1084;&#1077;&#1085;&#1077;&#1074;&#1072;&#1088;&#1082;&#1072;</h1>
    <div class="status">&#1055;&#1086;&#1087;&#1083;&#1072;&#1074;&#1082;&#1086;&#1074;&#1099;&#1081; &#1076;&#1072;&#1090;&#1095;&#1080;&#1082;: <b id="float-sensor">)rawliteral";
  html += floatSensorActive ? "ok" : "low";
  html += R"rawliteral(</b>, RAW: <b id="input-raw">)rawliteral";
  html += String(inputExpanderRaw);
  html += R"rawliteral(</b></div>
    <div class="status">&#1058;&#1077;&#1084;&#1087;&#1077;&#1088;&#1072;&#1090;&#1091;&#1088;&#1072; &#1074;&#1086;&#1076;&#1099;: <b id="water-temperature">)rawliteral";
  html += isnan(waterTemperatureC) ? "N/A" : String(waterTemperatureC, 2);
  html += R"rawliteral(</b> &#176;C, Dallas: <b id="dallas-count">)rawliteral";
  html += String(dallasDeviceCount);
  html += R"rawliteral(</b></div>
    <section class="valves">
      <article class="panel">
        <h2>&#1047;&#1072;&#1083;&#1080;&#1074; &#1074;&#1086;&#1076;&#1099;</h2>
        <div class="status">&#1057;&#1086;&#1089;&#1090;&#1086;&#1103;&#1085;&#1080;&#1077;: <b id="fill-valve">)rawliteral";
  html += fillValveOn ? "closed" : "open";
  html += R"rawliteral(</b></div>
        <div class="buttons">
          <form action="/valve" method="post">
            <input type="hidden" name="valve" value="fill">
            <input type="hidden" name="state" value="1">
            <button class="valve-on" type="submit">&#1047;&#1072;&#1082;&#1088;&#1099;&#1090;&#1100;</button>
          </form>
          <form action="/valve" method="post">
            <input type="hidden" name="valve" value="fill">
            <input type="hidden" name="state" value="0">
            <button class="valve-off" type="submit">&#1054;&#1090;&#1082;&#1088;&#1099;&#1090;&#1100;</button>
          </form>
        </div>
      </article>
      <article class="panel">
        <h2>&#1057;&#1083;&#1080;&#1074; &#1074;&#1086;&#1076;&#1099;</h2>
        <div class="status">&#1057;&#1086;&#1089;&#1090;&#1086;&#1103;&#1085;&#1080;&#1077;: <b id="drain-valve">)rawliteral";
  html += drainValveOn ? "closed" : "open";
  html += R"rawliteral(</b></div>
        <div class="buttons">
          <form action="/valve" method="post">
            <input type="hidden" name="valve" value="drain">
            <input type="hidden" name="state" value="1">
            <button class="valve-on" type="submit">&#1047;&#1072;&#1082;&#1088;&#1099;&#1090;&#1100;</button>
          </form>
          <form action="/valve" method="post">
            <input type="hidden" name="valve" value="drain">
            <input type="hidden" name="state" value="0">
            <button class="valve-off" type="submit">&#1054;&#1090;&#1082;&#1088;&#1099;&#1090;&#1100;</button>
          </form>
        </div>
      </article>
    </section>
    <section class="panel">
      <h2>&#1058;&#1069;&#1053;</h2>
      <div class="status">&#1057;&#1086;&#1089;&#1090;&#1086;&#1103;&#1085;&#1080;&#1077;: <b id="heater-state">)rawliteral";
  html += heaterOn ? "on" : "off";
  html += R"rawliteral(</b></div>
      <div class="status">&#1041;&#1083;&#1086;&#1082;&#1080;&#1088;&#1086;&#1074;&#1082;&#1072;: <b id="heater-interlock">)rawliteral";
  html += heaterInterlockOk() ? "ready" : "blocked";
  html += R"rawliteral(</b></div>
      <div class="buttons">
        <form action="/heater" method="post">
          <input type="hidden" name="state" value="1">
          <button id="heater-enable" class="heater-on" type="submit" )rawliteral";
  if (!heaterInterlockOk()) {
    html += "disabled";
  }
  html += R"rawliteral(>&#1042;&#1082;&#1083;&#1102;&#1095;&#1080;&#1090;&#1100; &#1058;&#1069;&#1053;</button>
        </form>
        <form action="/heater" method="post">
          <input type="hidden" name="state" value="0">
          <button class="heater-off" type="submit">&#1042;&#1099;&#1082;&#1083;&#1102;&#1095;&#1080;&#1090;&#1100; &#1058;&#1069;&#1053;</button>
        </form>
      </div>
    </section>
    <section class="grid">
)rawliteral";

  for (uint8_t i = 0; i < DRIVE_COUNT; i++) {
    html += R"rawliteral(
      <article class="panel">
        <h2>&#1055;&#1088;&#1080;&#1074;&#1086;&#1076; )rawliteral";
    html += String(i + 1);
    html += R"rawliteral(</h2>
        <div class="status">&#1057;&#1094;&#1077;&#1085;&#1072;&#1088;&#1080;&#1081;: <b id="sequence-)rawliteral";
    html += String(i);
    html += R"rawliteral(">)rawliteral";
    html += sequenceText(drives[i].sequence);
    html += R"rawliteral(</b></div>
        <div class="status">&#1055;&#1086;&#1083;&#1086;&#1078;&#1077;&#1085;&#1080;&#1077;: <b id="position-)rawliteral";
    html += String(i);
    html += R"rawliteral(">)rawliteral";
    html += positionText(drives[i].position);
    html += R"rawliteral(</b></div>
        <div class="status">&#1053;&#1072;&#1087;&#1088;&#1072;&#1074;&#1083;&#1077;&#1085;&#1080;&#1077;: <b id="direction-)rawliteral";
    html += String(i);
    html += R"rawliteral(">)rawliteral";
    html += directionText(drives[i].direction);
    html += R"rawliteral(</b></div>
        <div class="status">&#1054;&#1089;&#1090;&#1072;&#1083;&#1086;&#1089;&#1100;: <b id="remaining-)rawliteral";
    html += String(i);
    html += R"rawliteral(">)rawliteral";
    html += String((remainingStepMs(i) + 999) / 1000);
    html += R"rawliteral( &#1089;&#1077;&#1082;</b></div>
        <div class="buttons">
          <form action="/lower" method="post">
            <input type="hidden" name="drive" value=")rawliteral";
    html += String(i);
    html += R"rawliteral(">
            <button id="lower-)rawliteral";
    html += String(i);
    html += R"rawliteral(" class="down" type="submit" )rawliteral";
    if (!canLower(i)) {
      html += "disabled";
    }
    html += R"rawliteral(>&#1054;&#1087;&#1091;&#1089;&#1090;&#1080;&#1090;&#1100;</button>
          </form>
          <form action="/raise" method="post">
            <input type="hidden" name="drive" value=")rawliteral";
    html += String(i);
    html += R"rawliteral(">
            <button id="raise-)rawliteral";
    html += String(i);
    html += R"rawliteral(" class="up" type="submit" )rawliteral";
    if (!canRaise(i)) {
      html += "disabled";
    }
    html += R"rawliteral(>&#1055;&#1086;&#1076;&#1085;&#1103;&#1090;&#1100;</button>
          </form>
        </div>
      </article>
)rawliteral";
  }

  html += R"rawliteral(
    </section>
  </main>
  <script>
    const sequences = {
      idle: '\u041e\u0436\u0438\u0434\u0430\u043d\u0438\u0435',
      lowering: '\u041e\u043f\u0443\u0441\u043a\u0430\u0435\u0442\u0441\u044f',
      raise_initial: '\u041f\u043e\u0434\u043d\u0438\u043c\u0430\u0435\u0442\u0441\u044f \u0434\u043e \u0432\u0441\u0442\u0440\u044f\u0445\u0438\u0432\u0430\u043d\u0438\u044f',
      shake: '\u0412\u0441\u0442\u0440\u044f\u0445\u0438\u0432\u0430\u043d\u0438\u0435',
      raise_final: '\u041f\u043e\u0434\u043d\u0438\u043c\u0430\u0435\u0442\u0441\u044f \u0434\u043e \u0432\u0435\u0440\u0445\u0430',
      done: '\u0413\u043e\u0442\u043e\u0432\u043e'
    };
    const directions = {
      stop: '\u0421\u0442\u043e\u043f',
      up: '\u0412\u0432\u0435\u0440\u0445',
      down: '\u0412\u043d\u0438\u0437'
    };
    const positions = {
      unknown: '\u041d\u0435\u0438\u0437\u0432\u0435\u0441\u0442\u043d\u043e',
      top: '\u0412\u0432\u0435\u0440\u0445\u0443',
      bottom: '\u0412\u043d\u0438\u0437\u0443'
    };

    async function refreshStatus() {
      try {
        const response = await fetch('/status', { cache: 'no-store' });
        if (!response.ok) return;
        const status = await response.json();
        document.getElementById('float-sensor').textContent = status.floatSensorActive
          ? '\u0412\u043e\u0434\u0430 \u043d\u0430 \u043d\u0443\u0436\u043d\u043e\u043c \u0443\u0440\u043e\u0432\u043d\u0435'
          : '\u0412\u043e\u0434\u044b \u043d\u0435\u0434\u043e\u0441\u0442\u0430\u0442\u043e\u0447\u043d\u043e';
        document.getElementById('input-raw').textContent = `0x${status.inputRaw.toString(16).padStart(2, '0').toUpperCase()}`;
        document.getElementById('water-temperature').textContent = status.temperatureC === null
          ? 'N/A'
          : status.temperatureC.toFixed(2);
        document.getElementById('dallas-count').textContent = status.dallasDeviceCount;
        document.getElementById('fill-valve').textContent = status.fillValveOn
          ? '\u0417\u0430\u043a\u0440\u044b\u0442'
          : '\u041e\u0442\u043a\u0440\u044b\u0442';
        document.getElementById('drain-valve').textContent = status.drainValveOn
          ? '\u0417\u0430\u043a\u0440\u044b\u0442'
          : '\u041e\u0442\u043a\u0440\u044b\u0442';
        document.getElementById('heater-state').textContent = status.heaterOn
          ? '\u0412\u043a\u043b\u044e\u0447\u0435\u043d'
          : '\u0412\u044b\u043a\u043b\u044e\u0447\u0435\u043d';
        document.getElementById('heater-interlock').textContent = status.heaterAllowed
          ? '\u0413\u043e\u0442\u043e\u0432\u043e: \u0443\u0440\u043e\u0432\u0435\u043d\u044c \u0435\u0441\u0442\u044c, \u043a\u0440\u0430\u043d\u044b \u0437\u0430\u043a\u0440\u044b\u0442\u044b'
          : '\u0417\u0430\u043f\u0440\u0435\u0442: \u043d\u0443\u0436\u0435\u043d \u0443\u0440\u043e\u0432\u0435\u043d\u044c \u0432\u043e\u0434\u044b \u0438 \u0437\u0430\u043a\u0440\u044b\u0442\u044b\u0435 \u043a\u0440\u0430\u043d\u044b';
        document.getElementById('heater-enable').disabled = !status.heaterAllowed;
        status.drives.forEach((drive, index) => {
          document.getElementById(`sequence-${index}`).textContent = sequences[drive.sequence] || drive.sequence;
          document.getElementById(`position-${index}`).textContent = positions[drive.position] || drive.position;
          document.getElementById(`direction-${index}`).textContent = directions[drive.direction] || drive.direction;
          document.getElementById(`remaining-${index}`).textContent = `${drive.remainingSeconds} \u0441\u0435\u043a`;
          document.getElementById(`lower-${index}`).disabled = !drive.canLower;
          document.getElementById(`raise-${index}`).disabled = !drive.canRaise;
        });
      } catch (error) {
      }
    }

    document.querySelectorAll('form').forEach((form) => {
      form.addEventListener('submit', async (event) => {
        event.preventDefault();
        const body = new URLSearchParams(new FormData(form));
        await fetch(form.action, { method: 'POST', body, cache: 'no-store' });
        await refreshStatus();
      });
    });

    setInterval(refreshStatus, 300);
    refreshStatus();
  </script>
</body>
</html>
)rawliteral";

  return html;
}

uint8_t webDriveArg() {
  if (!server.hasArg("drive")) {
    return 255;
  }
  int driveIndex = server.arg("drive").toInt();
  if (driveIndex < 0 || driveIndex >= DRIVE_COUNT) {
    return 255;
  }
  return static_cast<uint8_t>(driveIndex);
}

void handleRoot() {
  server.send(200, "text/html; charset=UTF-8", generateHTML());
}

void handleStatus() {
  server.send(200, "application/json; charset=UTF-8", statusJson());
}

void handleLower() {
  uint8_t driveIndex = webDriveArg();
  if (validDriveIndex(driveIndex)) {
    startLowerSequence(driveIndex);
  }
  server.send(204, "text/plain", "");
}

void handleRaise() {
  uint8_t driveIndex = webDriveArg();
  if (validDriveIndex(driveIndex)) {
    startRaiseSequence(driveIndex);
  }
  server.send(204, "text/plain", "");
}

void handleValve() {
  if (server.hasArg("valve") && server.hasArg("state")) {
    bool enabled = server.arg("state").toInt() != 0;
    if (server.arg("valve") == "fill") {
      setFillValve(enabled);
    } else if (server.arg("valve") == "drain") {
      setDrainValve(enabled);
    }
  }
  server.send(204, "text/plain", "");
}

void handleHeater() {
  if (server.hasArg("state")) {
    setHeater(server.arg("state").toInt() != 0);
  }
  server.send(204, "text/plain", "");
}

void handleRedirectToRoot() {
  server.sendHeader("Location", String("http://") + WiFi.softAPIP().toString() + "/", true);
  server.send(302, "text/plain", "");
}

void setup() {
  initializeRelayExpandersSafe();
  Serial.begin(115200);
  loadTimingSettings();
  initDriveStates();
  closeValvesByDefault();
  updateFloatSensor();
  dallasSensors.begin();
  dallasSensors.setWaitForConversion(false);
  dallasDeviceCount = dallasSensors.getDeviceCount();
  updateDallasTemperature(millis());

  WiFi.softAP(ap_ssid, ap_password);
  delay(200);

  RS485Serial.begin(RS485_BAUDRATE, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
  if (RS485_DE_PIN >= 0) {
    mb.begin(&RS485Serial, RS485_DE_PIN);
  } else {
    mb.begin(&RS485Serial);
  }
  mb.slave(MODBUS_SLAVE_ID);
  mb.addHreg(REG_GLOBAL_STATUS, 0, REG_REG_COUNT);
  modbusReady = true;
  mb.Hreg(REG_COMMAND_RESULT, CMD_RES_OK);
  syncModbusRegistersFromState();

  dnsServer.start(DNS_PORT, "*", WiFi.softAPIP());

  server.on("/", HTTP_GET, handleRoot);
  server.on("/status", HTTP_GET, handleStatus);
  server.on("/lower", HTTP_POST, handleLower);
  server.on("/raise", HTTP_POST, handleRaise);
  server.on("/valve", HTTP_POST, handleValve);
  server.on("/heater", HTTP_POST, handleHeater);
  server.on("/generate_204", HTTP_GET, handleRedirectToRoot);
  server.on("/hotspot-detect.html", HTTP_GET, handleRedirectToRoot);
  server.on("/fwlink", HTTP_GET, handleRedirectToRoot);
  server.onNotFound(handleRedirectToRoot);

  server.begin();

  for (uint8_t i = 0; i < DRIVE_COUNT; i++) {
    startStartupRaiseSequence(i);
  }
  syncModbusRegistersFromState();

  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());
  Serial.print("Modbus RTU slave ");
  Serial.print(MODBUS_SLAVE_ID);
  Serial.print(" ready: ");
  Serial.print(RS485_BAUDRATE);
  Serial.print(" 8N1, RX=GPIO");
  Serial.print(RS485_RX_PIN);
  Serial.print(", TX=GPIO");
  Serial.print(RS485_TX_PIN);
  Serial.print(", DE=");
  Serial.println(RS485_DE_PIN);
}

void loop() {
  mb.task();
  dnsServer.processNextRequest();

  for (uint8_t i = 0; i < DRIVE_COUNT; i++) {
    updateDriveSequence(i);
  }

  uint32_t nowMs = millis();
  updateDallasTemperature(nowMs);
  updateFloatSensor();
  if (heaterOn && !heaterInterlockOk()) {
    setHeater(false);
  }
  handleModbusCommand();
  applyModbusWritableRegisters();
  syncModbusRegistersFromState();
  server.handleClient();
  mb.task();
}
