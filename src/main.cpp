#include <Arduino.h>
#include <DallasTemperature.h>
#include <DNSServer.h>
#include <ETH.h>
#include <ModbusTCP.h>
#include <OneWire.h>
#include <Preferences.h>
#include <WebServer.h>
#include <WiFi.h>
#include <Wire.h>
#include <esp_timer.h>

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
static const uint32_t VALVE_LOCK_MS = 10000;
static const uint8_t HEATER_PHASE_A_CHANNEL = 11;
static const uint8_t HEATER_PHASE_B_CHANNEL = 12;
static const uint8_t HEATER_PHASE_C_CHANNEL = 13;

static const uint8_t KC_ETH_PHY_ADDRESS = 0;
static const int8_t KC_ETH_POWER_PIN = -1;
static const uint8_t KC_ETH_MDC_PIN = 23;
static const uint8_t KC_ETH_MDIO_PIN = 18;
static const uint16_t MODBUS_TCP_PORT = 502;
static const IPAddress DEFAULT_ETHERNET_IP(192, 168, 1, 240);
static const IPAddress DEFAULT_ETHERNET_GATEWAY(192, 168, 1, 1);
static const IPAddress DEFAULT_ETHERNET_SUBNET(255, 255, 255, 0);

struct NetworkSettings {
  uint32_t marker;
  uint32_t ip;
  uint32_t subnet;
  uint32_t gateway;
  uint32_t dns;
  uint32_t useDhcp;
};

static const uint32_t NETWORK_SETTINGS_MARKER = 0x4E455431;
NetworkSettings networkSettings = {
  NETWORK_SETTINGS_MARKER,
  static_cast<uint32_t>(DEFAULT_ETHERNET_IP),
  static_cast<uint32_t>(DEFAULT_ETHERNET_SUBNET),
  static_cast<uint32_t>(DEFAULT_ETHERNET_GATEWAY),
  static_cast<uint32_t>(DEFAULT_ETHERNET_GATEWAY),
  0
};
bool networkApplyPending = false;
uint32_t networkApplyRequestedAtMs = 0;

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
ModbusTCP mb;
Preferences preferences;
OneWire oneWire(DALLAS_PIN);
DallasTemperature dallasSensors(&oneWire);

static const uint16_t DNS_PORT = 53;

enum ModbusReg : uint16_t {
  REG_STAT_MODE = 0,
  REG_STAT_ERROR = 1,
  REG_LOWER_TIME_MS = 7,
  REG_RAISE_FULL_TIME_MS = 8,
  REG_RAISE_BEFORE_SHAKE_MS = 9,
  REG_SHAKE_UP_TIME_MS = 10,
  REG_SHAKE_DOWN_TIME_MS = 11,
  REG_SHAKE_COUNT = 12,
  REG_VALVE_CONTROL = 13,
  REG_HEATER_CONTROL = 14,
  REG_SERVICE_COMMAND = 15,
  REG_COMMAND_RESULT = 16,
  REG_PROTOCOL_VERSION = 17,
  REG_COMMAND_DRIVE = 18
};

enum ModbusInputReg : uint16_t {
  INPUT_UPTIME_HOURS = 0,
  INPUT_END_BASE = 20,
  INPUT_RELAY_BASE = 50,
  INPUT_TEMPERATURE_X10 = 70,
  INPUT_DALLAS_COUNT = 71,
  INPUT_FLOAT_SENSOR = 72,
  INPUT_VALVE_STATUS = 73,
  INPUT_HEATER_STATUS = 74,
  INPUT_GLOBAL_STATUS = 75,
  INPUT_FILL_STATE = 76,
  INPUT_DRAIN_STATE = 77,
  INPUT_HEATER_STATE = 78,
  INPUT_VALVE_LOCK_SECONDS = 79,
  INPUT_DRIVE_BASE = 80
};

enum ModbusCommand : uint16_t {
  CMD_NONE = 0,
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
uint32_t valveChangedAtMs[2] = {0, 0};
bool valveMotionStarted[2] = {false, false};
bool startupInProgress = true;
bool modbusReady = false;
bool heaterOn = false;
bool ethernetStarted = false;
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

bool parseIPv4(const String& text, IPAddress& address) {
  String value = text;
  value.trim();
  return address.fromString(value);
}

bool validNetworkSettings(const NetworkSettings& settings) {
  if (settings.marker != NETWORK_SETTINGS_MARKER || settings.useDhcp > 1) {
    return false;
  }
  IPAddress ip(settings.ip);
  IPAddress subnet(settings.subnet);
  IPAddress gateway(settings.gateway);
  IPAddress dns(settings.dns);
  uint32_t mask = (static_cast<uint32_t>(subnet[0]) << 24) |
                  (static_cast<uint32_t>(subnet[1]) << 16) |
                  (static_cast<uint32_t>(subnet[2]) << 8) | subnet[3];
  uint32_t wildcard = ~mask;
  if (mask == 0 || mask == 0xFFFFFFFF || (wildcard & (wildcard + 1)) != 0) {
    return false;
  }
  if (ip[0] == 0 || ip[0] >= 224 || gateway[0] == 0 || gateway[0] >= 224 ||
      dns[0] == 0 || dns[0] >= 224 || ip == gateway) {
    return false;
  }
  return (settings.ip & settings.subnet) == (settings.gateway & settings.subnet);
}

void loadNetworkSettings() {
  Preferences storage;
  if (!storage.begin("ethernet", true)) {
    return;
  }
  NetworkSettings saved;
  if (storage.getBytesLength("config") == sizeof(saved) &&
      storage.getBytes("config", &saved, sizeof(saved)) == sizeof(saved) &&
      validNetworkSettings(saved)) {
    networkSettings = saved;
  }
  storage.end();
}

bool saveNetworkSettings(const NetworkSettings& settings) {
  Preferences storage;
  if (!storage.begin("ethernet", false)) {
    return false;
  }
  bool saved = storage.putBytes("config", &settings, sizeof(settings)) == sizeof(settings);
  storage.end();
  return saved;
}

bool applyEthernetSettings() {
  if (networkSettings.useDhcp) {
    return ETH.config(IPAddress(0, 0, 0, 0), IPAddress(0, 0, 0, 0), IPAddress(0, 0, 0, 0));
  }
  return ETH.config(IPAddress(networkSettings.ip), IPAddress(networkSettings.gateway),
                    IPAddress(networkSettings.subnet), IPAddress(networkSettings.dns));
}

void onNetworkEvent(WiFiEvent_t event) {
  switch (event) {
    case ARDUINO_EVENT_ETH_START:
      Serial.println("Ethernet: started");
      break;
    case ARDUINO_EVENT_ETH_CONNECTED:
      Serial.println("Ethernet: cable link up");
      break;
    case ARDUINO_EVENT_ETH_GOT_IP:
      Serial.print("Ethernet IP: ");
      Serial.println(ETH.localIP());
      break;
    case ARDUINO_EVENT_ETH_DISCONNECTED:
      Serial.println("Ethernet: cable link down");
      break;
    case ARDUINO_EVENT_ETH_STOP:
      Serial.println("Ethernet: stopped");
      break;
    default:
      break;
  }
}

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
  bool changed = fillValveOn != enabled;
  fillValveOn = enabled;
  setRelayChannel(FILL_VALVE_CHANNEL, enabled);
  enforceHeaterInterlock();
  writeRelayOutputs();
  if (changed) {
    valveChangedAtMs[0] = millis();
    valveMotionStarted[0] = true;
  }
  syncValveModbusRegisters();
}

void setDrainValve(bool enabled) {
  bool changed = drainValveOn != enabled;
  drainValveOn = enabled;
  setRelayChannel(DRAIN_VALVE_CHANNEL, enabled);
  enforceHeaterInterlock();
  writeRelayOutputs();
  if (changed) {
    valveChangedAtMs[1] = millis();
    valveMotionStarted[1] = true;
  }
  syncValveModbusRegisters();
}

uint32_t valveLockRemainingMs(uint8_t valveIndex) {
  if (valveIndex >= 2 || !valveMotionStarted[valveIndex]) {
    return 0;
  }
  uint32_t elapsedMs = millis() - valveChangedAtMs[valveIndex];
  return elapsedMs >= VALVE_LOCK_MS ? 0 : VALVE_LOCK_MS - elapsedMs;
}

bool canChangeValve(uint8_t valveIndex, bool enabled) {
  if (valveIndex >= 2) {
    return false;
  }
  bool currentState = valveIndex == 0 ? fillValveOn : drainValveOn;
  return enabled != currentState && valveLockRemainingMs(valveIndex) == 0;
}

bool requestValveState(uint8_t valveIndex, bool enabled) {
  if (!canChangeValve(valveIndex, enabled)) {
    return false;
  }
  if (valveIndex == 0) {
    setFillValve(enabled);
  } else {
    setDrainValve(enabled);
  }
  return true;
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
  mb.Hreg(REG_VALVE_CONTROL, bits);
}

void syncHeaterModbusRegisters() {
  if (!modbusReady) {
    return;
  }

  uint16_t value = heaterOn ? 1 : 0;
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
  if (startupInProgress) {
    globalStatus |= (1U << 3);
  }
  uint16_t mode = 1;
  for (uint8_t driveIndex = 0; driveIndex < DRIVE_COUNT; driveIndex++) {
    LiftSequence sequence = drives[driveIndex].sequence;
    if (sequence == SEQUENCE_LOWERING) {
      mode = 11 + driveIndex;
      break;
    }
    if (sequence >= SEQUENCE_RAISE_INITIAL && sequence <= SEQUENCE_RAISE_FINAL) {
      mode = 14 + driveIndex;
      break;
    }
    if (sequence == SEQUENCE_DONE) {
      mode = 19;
    }
  }
  mb.Hreg(REG_STAT_MODE, mode);
  mb.Hreg(REG_STAT_ERROR, 0);
  mb.Hreg(REG_LOWER_TIME_MS, clampU16(lowerTimeMs));
  mb.Hreg(REG_RAISE_FULL_TIME_MS, clampU16(raiseFullTimeMs));
  mb.Hreg(REG_RAISE_BEFORE_SHAKE_MS, clampU16(raiseBeforeShakeTimeMs));
  mb.Hreg(REG_SHAKE_UP_TIME_MS, clampU16(shakeUpTimeMs));
  mb.Hreg(REG_SHAKE_DOWN_TIME_MS, clampU16(shakeDownTimeMs));
  mb.Hreg(REG_SHAKE_COUNT, shakeCount);
  mb.Hreg(REG_VALVE_CONTROL, valveStatusBits());
  uint16_t temperatureReg = 0x8000;
  if (!isnan(waterTemperatureC)) {
    temperatureReg = floatToI16Reg(waterTemperatureC, 10.0f);
  }
  mb.Hreg(REG_HEATER_CONTROL, heaterOn ? 1 : 0);
  mb.Hreg(REG_PROTOCOL_VERSION, 2);

  uint64_t uptimeHours = static_cast<uint64_t>(esp_timer_get_time()) / 3600000000ULL;
  mb.Ireg(INPUT_UPTIME_HOURS, uptimeHours > 65535 ? 65535 : static_cast<uint16_t>(uptimeHours));
  for (uint8_t driveIndex = 0; driveIndex < DRIVE_COUNT; driveIndex++) {
    mb.Ireg(INPUT_END_BASE + driveIndex * 2, drives[driveIndex].position == POSITION_TOP ? 1 : 0);
    mb.Ireg(INPUT_END_BASE + driveIndex * 2 + 1, drives[driveIndex].position == POSITION_BOTTOM ? 1 : 0);
  }
  for (uint8_t channel = 0; channel < RELAY_CHANNEL_COUNT; channel++) {
    mb.Ireg(INPUT_RELAY_BASE + channel, relayOn[channel] ? 1 : 0);
  }
  mb.Ireg(INPUT_TEMPERATURE_X10, temperatureReg);
  mb.Ireg(INPUT_DALLAS_COUNT, dallasDeviceCount);
  mb.Ireg(INPUT_FLOAT_SENSOR, floatSensorActive ? 1 : 0);
  mb.Ireg(INPUT_VALVE_STATUS, valveStatusBits());
  mb.Ireg(INPUT_HEATER_STATUS, heaterOn ? 1 : 0);
  mb.Ireg(INPUT_GLOBAL_STATUS, globalStatus);
  mb.Ireg(INPUT_FILL_STATE, fillValveOn ? 1 : 0);
  mb.Ireg(INPUT_DRAIN_STATE, drainValveOn ? 1 : 0);
  mb.Ireg(INPUT_HEATER_STATE, heaterOn ? 1 : (heaterInterlockOk() ? 0 : 2));
  uint16_t fillLockSeconds = (valveLockRemainingMs(0) + 999) / 1000;
  uint16_t drainLockSeconds = (valveLockRemainingMs(1) + 999) / 1000;
  mb.Ireg(INPUT_VALVE_LOCK_SECONDS, fillLockSeconds | (drainLockSeconds << 8));

  for (uint8_t driveIndex = 0; driveIndex < DRIVE_COUNT; driveIndex++) {
    uint16_t base = INPUT_DRIVE_BASE + (driveIndex * 5);
    mb.Ireg(base + 0, driveStatusBits(driveIndex));
    mb.Ireg(base + 1, static_cast<uint16_t>(drives[driveIndex].sequence));
    mb.Ireg(base + 2, static_cast<uint16_t>(drives[driveIndex].direction));
    mb.Ireg(base + 3, clampU16(remainingStepMs(driveIndex)));
    mb.Ireg(base + 4, static_cast<uint16_t>(drives[driveIndex].position));
  }
}

void applyModbusWritableRegisters() {
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
  uint16_t cmd = mb.Hreg(REG_SERVICE_COMMAND);
  if (cmd == CMD_NONE) {
    return;
  }

  uint16_t result = CMD_RES_OK;
  uint16_t driveArg = mb.Hreg(REG_COMMAND_DRIVE);

  if (cmd == CMD_SAVE_SETTINGS) {
    if (anyDriveBusy()) {
      result = CMD_RES_BUSY;
    } else {
      saveTimingSettings();
    }
  } else if (cmd == CMD_STARTUP_RAISE) {
    if (driveArg >= 1 && driveArg <= DRIVE_COUNT) {
      result = handleDriveCommand(cmd, static_cast<uint8_t>(driveArg - 1));
    } else {
      result = CMD_RES_INVALID_ARG;
    }
  } else {
    result = CMD_RES_UNKNOWN_CMD;
  }

  mb.Hreg(REG_COMMAND_RESULT, result);
  mb.Hreg(REG_SERVICE_COMMAND, CMD_NONE);
}

bool readableHoldingRange(uint16_t address, uint16_t count) {
  return count > 0 && ((address <= REG_STAT_ERROR && address + count <= REG_STAT_ERROR + 1) ||
                       (address >= REG_LOWER_TIME_MS && address + count <= REG_COMMAND_DRIVE + 1));
}

bool readableInputRange(uint16_t address, uint16_t count) {
  if (count == 0) {
    return false;
  }
  uint32_t end = static_cast<uint32_t>(address) + count;
  return (address == INPUT_UPTIME_HOURS && end == 1) ||
         (address >= INPUT_END_BASE && end <= INPUT_END_BASE + DRIVE_COUNT * 2) ||
         (address >= INPUT_RELAY_BASE && end <= INPUT_RELAY_BASE + RELAY_CHANNEL_COUNT) ||
         (address >= INPUT_TEMPERATURE_X10 && end <= INPUT_VALVE_LOCK_SECONDS + 1) ||
         (address >= INPUT_DRIVE_BASE && end <= INPUT_DRIVE_BASE + DRIVE_COUNT * 5);
}

Modbus::ResultCode validateModbusRequest(Modbus::FunctionCode function, const Modbus::RequestData data) {
  uint16_t address = data.reg.address;
  if (function == Modbus::FC_READ_REGS) {
    if (address <= 6 && static_cast<uint32_t>(address) + data.regCount > 2) {
      return Modbus::EX_ILLEGAL_VALUE;
    }
    return readableHoldingRange(address, data.regCount) ? Modbus::EX_SUCCESS : Modbus::EX_ILLEGAL_ADDRESS;
  }
  if (function == Modbus::FC_READ_INPUT_REGS) {
    return readableInputRange(address, data.regCount) ? Modbus::EX_SUCCESS : Modbus::EX_ILLEGAL_ADDRESS;
  }
  if (function == Modbus::FC_WRITE_REGS) {
    return Modbus::EX_ILLEGAL_VALUE;
  }
  if (function != Modbus::FC_WRITE_REG) {
    return Modbus::EX_ILLEGAL_FUNCTION;
  }
  if (startupInProgress) {
    return Modbus::EX_SLAVE_DEVICE_BUSY;
  }
  uint16_t value = data.regCount;
  if (address >= 2 && address <= 6) {
    return Modbus::EX_ILLEGAL_VALUE;
  }
  if (address == REG_STAT_MODE) {
    if (value < 11 || value > 16 || mb.Hreg(REG_STAT_MODE) != 1 || anyDriveBusy()) {
      return Modbus::EX_ILLEGAL_VALUE;
    }
    uint8_t driveIndex = (value - 11) % DRIVE_COUNT;
    bool canMove = value <= 13 ? canLower(driveIndex) : canRaise(driveIndex);
    return canMove ? Modbus::EX_SUCCESS : Modbus::EX_ILLEGAL_VALUE;
  }
  if (address == REG_STAT_ERROR || address == REG_COMMAND_RESULT || address == REG_PROTOCOL_VERSION) {
    return Modbus::EX_ILLEGAL_VALUE;
  }
  if (address >= REG_LOWER_TIME_MS && address <= REG_SHAKE_COUNT) {
    uint32_t lower = address == REG_LOWER_TIME_MS ? value : lowerTimeMs;
    uint32_t raiseFull = address == REG_RAISE_FULL_TIME_MS ? value : raiseFullTimeMs;
    uint32_t raiseBefore = address == REG_RAISE_BEFORE_SHAKE_MS ? value : raiseBeforeShakeTimeMs;
    uint32_t shakeUp = address == REG_SHAKE_UP_TIME_MS ? value : shakeUpTimeMs;
    uint32_t shakeDown = address == REG_SHAKE_DOWN_TIME_MS ? value : shakeDownTimeMs;
    uint16_t shakes = address == REG_SHAKE_COUNT ? value : shakeCount;
    return !anyDriveBusy() && validTimingSettings(lower, raiseFull, raiseBefore, shakeUp, shakeDown, shakes)
             ? Modbus::EX_SUCCESS : Modbus::EX_ILLEGAL_VALUE;
  }
  if (address == REG_VALVE_CONTROL) {
    if (value > 3 || value == valveStatusBits()) {
      return Modbus::EX_ILLEGAL_VALUE;
    }
    bool fillChanged = (value & 1U) != (fillValveOn ? 1U : 0U);
    bool drainChanged = ((value >> 1) & 1U) != (drainValveOn ? 1U : 0U);
    if ((fillChanged && valveLockRemainingMs(0) != 0) ||
        (drainChanged && valveLockRemainingMs(1) != 0)) {
      return Modbus::EX_ILLEGAL_VALUE;
    }
    return Modbus::EX_SUCCESS;
  }
  if (address == REG_HEATER_CONTROL) {
    return value <= 1 && (value == 0 || heaterInterlockOk()) ? Modbus::EX_SUCCESS : Modbus::EX_ILLEGAL_VALUE;
  }
  if (address == REG_SERVICE_COMMAND) {
    bool valid = value == CMD_SAVE_SETTINGS || (value == CMD_STARTUP_RAISE &&
      mb.Hreg(REG_COMMAND_DRIVE) >= 1 && mb.Hreg(REG_COMMAND_DRIVE) <= DRIVE_COUNT &&
      canRaise(mb.Hreg(REG_COMMAND_DRIVE) - 1));
    return valid && !anyDriveBusy() ? Modbus::EX_SUCCESS : Modbus::EX_ILLEGAL_VALUE;
  }
  if (address == REG_COMMAND_DRIVE) {
    return value >= 1 && value <= DRIVE_COUNT ? Modbus::EX_SUCCESS : Modbus::EX_ILLEGAL_VALUE;
  }
  return Modbus::EX_ILLEGAL_ADDRESS;
}

Modbus::ResultCode onModbusRequestSuccess(Modbus::FunctionCode function, const Modbus::RequestData data) {
  if (function == Modbus::FC_WRITE_REG && data.reg.address == REG_VALVE_CONTROL) {
    uint16_t value = data.regCount;
    bool fillRequested = (value & 1U) != 0;
    bool drainRequested = (value & 2U) != 0;
    if (fillRequested != fillValveOn) {
      requestValveState(0, fillRequested);
    }
    if (drainRequested != drainValveOn) {
      requestValveState(1, drainRequested);
    }
  } else if (function == Modbus::FC_WRITE_REG && data.reg.address == REG_STAT_MODE) {
    uint16_t value = data.regCount;
    uint8_t driveIndex = (value - 11) % DRIVE_COUNT;
    if (value <= 13) {
      startLowerSequence(driveIndex);
    } else {
      startRaiseSequence(driveIndex);
    }
  }
  return Modbus::EX_SUCCESS;
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
  json += "\"startupReady\":" + String(startupInProgress ? "false" : "true") + ",";
  json += "\"ethernetStarted\":" + String(ethernetStarted ? "true" : "false") + ",";
  json += "\"ethernetLinkUp\":" + String(ETH.linkUp() ? "true" : "false") + ",";
  json += "\"ethernetIp\":\"" + ETH.localIP().toString() + "\",";
  json += "\"floatSensorActive\":" + String(floatSensorActive ? "true" : "false") + ",";
  json += "\"heaterAllowed\":" + String(heaterInterlockOk() ? "true" : "false") + ",";
  json += "\"inputRaw\":" + String(inputExpanderRaw) + ",";
  json += "\"temperatureC\":";
  json += isnan(waterTemperatureC) ? "null" : String(waterTemperatureC, 2);
  json += ",";
  json += "\"dallasDeviceCount\":" + String(dallasDeviceCount) + ",";
  json += "\"fillValveOn\":" + String(fillValveOn ? "true" : "false") + ",";
  json += "\"drainValveOn\":" + String(drainValveOn ? "true" : "false") + ",";
  json += "\"fillValveLockSeconds\":" + String((valveLockRemainingMs(0) + 999) / 1000) + ",";
  json += "\"drainValveLockSeconds\":" + String((valveLockRemainingMs(1) + 999) / 1000) + ",";
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
    .network-fields {
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 10px;
    }
    .network-fields label { display: grid; gap: 4px; font-size: 14px; }
    .network-fields input, .network-fields select {
      min-width: 0;
      padding: 9px;
      border: 1px solid var(--line);
      border-radius: 6px;
      font: inherit;
    }
    .network-save { background: #1d4ed8; margin-top: 12px; }
    #network-message { margin-top: 8px; min-height: 20px; }
    @media (max-width: 760px) {
      .grid, .valves, .network-fields { grid-template-columns: 1fr; }
    }
  </style>
</head>
<body>
  <main>
    <h1>&#1055;&#1077;&#1083;&#1100;&#1084;&#1077;&#1085;&#1077;&#1074;&#1072;&#1088;&#1082;&#1072;</h1>
    <div class="status">Режим: <b id="startup-status">)rawliteral";
  html += startupInProgress ? "Инициализация" : "Готов";
  html += R"rawliteral(</b></div>
    <div class="status">Ethernet: <b id="ethernet-link">)rawliteral";
  html += ETH.linkUp() ? "link up" : "link down";
  html += R"rawliteral(</b>, IP: <b id="ethernet-ip">)rawliteral";
  html += ETH.localIP().toString();
  html += R"rawliteral(</b>, Modbus TCP: 502</div>
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
        <div class="status">Ожидание перед переключением: <b id="fill-lock">)rawliteral";
  html += String((valveLockRemainingMs(0) + 999) / 1000);
  html += R"rawliteral(</b> с</div>
        <div class="buttons">
          <form action="/valve" method="post">
            <input type="hidden" name="valve" value="fill">
            <input type="hidden" name="state" value="1">
            <button id="fill-close" class="valve-on" type="submit">&#1047;&#1072;&#1082;&#1088;&#1099;&#1090;&#1100;</button>
          </form>
          <form action="/valve" method="post">
            <input type="hidden" name="valve" value="fill">
            <input type="hidden" name="state" value="0">
            <button id="fill-open" class="valve-off" type="submit">&#1054;&#1090;&#1082;&#1088;&#1099;&#1090;&#1100;</button>
          </form>
        </div>
      </article>
      <article class="panel">
        <h2>&#1057;&#1083;&#1080;&#1074; &#1074;&#1086;&#1076;&#1099;</h2>
        <div class="status">&#1057;&#1086;&#1089;&#1090;&#1086;&#1103;&#1085;&#1080;&#1077;: <b id="drain-valve">)rawliteral";
  html += drainValveOn ? "closed" : "open";
  html += R"rawliteral(</b></div>
        <div class="status">Ожидание перед переключением: <b id="drain-lock">)rawliteral";
  html += String((valveLockRemainingMs(1) + 999) / 1000);
  html += R"rawliteral(</b> с</div>
        <div class="buttons">
          <form action="/valve" method="post">
            <input type="hidden" name="valve" value="drain">
            <input type="hidden" name="state" value="1">
            <button id="drain-close" class="valve-on" type="submit">&#1047;&#1072;&#1082;&#1088;&#1099;&#1090;&#1100;</button>
          </form>
          <form action="/valve" method="post">
            <input type="hidden" name="valve" value="drain">
            <input type="hidden" name="state" value="0">
            <button id="drain-open" class="valve-off" type="submit">&#1054;&#1090;&#1082;&#1088;&#1099;&#1090;&#1100;</button>
          </form>
        </div>
      </article>
    </section>
    <div id="valve-command-message" role="status"></div>
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
          <button id="heater-disable" class="heater-off" type="submit">&#1042;&#1099;&#1082;&#1083;&#1102;&#1095;&#1080;&#1090;&#1100; &#1058;&#1069;&#1053;</button>
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
    <section class="panel" style="margin-top: 12px">
      <h2>Ethernet / Modbus TCP</h2>
      <form id="network-form" action="/network" method="post">
        <div class="network-fields">
          <label>Режим адресации
            <select id="network-mode" name="mode">
              <option value="static")rawliteral";
  if (!networkSettings.useDhcp) html += " selected";
  html += R"rawliteral(>Статический IP</option>
              <option value="dhcp")rawliteral";
  if (networkSettings.useDhcp) html += " selected";
  html += R"rawliteral(>DHCP</option>
            </select>
          </label>
          <label>IP-адрес <input class="static-network-field" name="ip" value=")rawliteral";
  html += IPAddress(networkSettings.ip).toString();
  html += R"rawliteral("></label>
          <label>Маска подсети <input class="static-network-field" name="subnet" value=")rawliteral";
  html += IPAddress(networkSettings.subnet).toString();
  html += R"rawliteral("></label>
          <label>Шлюз <input class="static-network-field" name="gateway" value=")rawliteral";
  html += IPAddress(networkSettings.gateway).toString();
  html += R"rawliteral("></label>
          <label>DNS <input class="static-network-field" name="dns" value=")rawliteral";
  html += IPAddress(networkSettings.dns).toString();
  html += R"rawliteral("></label>
        </div>
        <button id="network-save" class="network-save" type="submit">Сохранить и применить</button>
        <div id="network-message" role="status"></div>
      </form>
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
        document.getElementById('startup-status').textContent = status.startupReady
          ? '\u0413\u043e\u0442\u043e\u0432'
          : '\u0418\u043d\u0438\u0446\u0438\u0430\u043b\u0438\u0437\u0430\u0446\u0438\u044f: \u043a\u043e\u043c\u0430\u043d\u0434\u044b \u043d\u0435\u0434\u043e\u0441\u0442\u0443\u043f\u043d\u044b';
        document.getElementById('network-save').disabled = !status.startupReady;
        document.getElementById('ethernet-link').textContent = status.ethernetLinkUp
          ? 'Кабель подключен'
          : 'Нет связи по кабелю';
        document.getElementById('ethernet-ip').textContent = status.ethernetIp;
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
        document.getElementById('fill-lock').textContent = status.fillValveLockSeconds;
        document.getElementById('drain-lock').textContent = status.drainValveLockSeconds;
        document.getElementById('fill-close').disabled =
          !status.startupReady || status.fillValveOn || status.fillValveLockSeconds > 0;
        document.getElementById('fill-open').disabled =
          !status.startupReady || !status.fillValveOn || status.fillValveLockSeconds > 0;
        document.getElementById('drain-close').disabled =
          !status.startupReady || status.drainValveOn || status.drainValveLockSeconds > 0;
        document.getElementById('drain-open').disabled =
          !status.startupReady || !status.drainValveOn || status.drainValveLockSeconds > 0;
        document.getElementById('heater-state').textContent = status.heaterOn
          ? '\u0412\u043a\u043b\u044e\u0447\u0435\u043d'
          : '\u0412\u044b\u043a\u043b\u044e\u0447\u0435\u043d';
        document.getElementById('heater-interlock').textContent = status.heaterAllowed
          ? '\u0413\u043e\u0442\u043e\u0432\u043e: \u0443\u0440\u043e\u0432\u0435\u043d\u044c \u0435\u0441\u0442\u044c, \u043a\u0440\u0430\u043d\u044b \u0437\u0430\u043a\u0440\u044b\u0442\u044b'
          : '\u0417\u0430\u043f\u0440\u0435\u0442: \u043d\u0443\u0436\u0435\u043d \u0443\u0440\u043e\u0432\u0435\u043d\u044c \u0432\u043e\u0434\u044b \u0438 \u0437\u0430\u043a\u0440\u044b\u0442\u044b\u0435 \u043a\u0440\u0430\u043d\u044b';
        document.getElementById('heater-enable').disabled =
          !status.startupReady || !status.heaterAllowed;
        document.getElementById('heater-disable').disabled = !status.startupReady;
        status.drives.forEach((drive, index) => {
          document.getElementById(`sequence-${index}`).textContent = sequences[drive.sequence] || drive.sequence;
          document.getElementById(`position-${index}`).textContent = positions[drive.position] || drive.position;
          document.getElementById(`direction-${index}`).textContent = directions[drive.direction] || drive.direction;
          document.getElementById(`remaining-${index}`).textContent = `${drive.remainingSeconds} \u0441\u0435\u043a`;
          document.getElementById(`lower-${index}`).disabled =
            !status.startupReady || !drive.canLower;
          document.getElementById(`raise-${index}`).disabled =
            !status.startupReady || !drive.canRaise;
        });
      } catch (error) {
      }
    }

    function updateNetworkMode() {
      const useDhcp = document.getElementById('network-mode').value === 'dhcp';
      document.querySelectorAll('.static-network-field').forEach((field) => {
        field.disabled = useDhcp;
      });
    }
    document.getElementById('network-mode').addEventListener('change', updateNetworkMode);
    updateNetworkMode();

    document.getElementById('network-form').addEventListener('submit', async (event) => {
      event.preventDefault();
      const form = event.currentTarget;
      const message = document.getElementById('network-message');
      try {
        const response = await fetch(form.action, {
          method: 'POST', body: new URLSearchParams(new FormData(form)), cache: 'no-store'
        });
        message.textContent = await response.text();
        message.style.color = response.ok ? 'var(--up)' : 'var(--down)';
      } catch (error) {
        message.textContent = 'Связь прервана. Проверьте новый IP или подключитесь к Wi-Fi dumplings.';
        message.style.color = 'var(--down)';
      }
    });

    document.querySelectorAll('form:not(#network-form)').forEach((form) => {
      form.addEventListener('submit', async (event) => {
        event.preventDefault();
        const isValve = form.getAttribute('action') === '/valve';
        const message = document.getElementById('valve-command-message');
        try {
          const body = new URLSearchParams(new FormData(form));
          const response = await fetch(form.action, {
            method: 'POST', body, cache: 'no-store'
          });
          if (isValve) {
            message.textContent = response.ok ? '' : await response.text();
            message.style.color = 'var(--down)';
          }
          await refreshStatus();
        } catch (error) {
          if (isValve) message.textContent = 'Нет связи с устройством';
        }
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

bool rejectCommandDuringStartup() {
  if (!startupInProgress) {
    return false;
  }
  server.sendHeader("Retry-After", "1");
  server.send(503, "text/plain; charset=UTF-8", "Инициализация: дождитесь закрытия кранов и подъёма приводов");
  return true;
}

void handleLower() {
  if (rejectCommandDuringStartup()) {
    return;
  }
  uint8_t driveIndex = webDriveArg();
  if (validDriveIndex(driveIndex)) {
    startLowerSequence(driveIndex);
  }
  server.send(204, "text/plain", "");
}

void handleRaise() {
  if (rejectCommandDuringStartup()) {
    return;
  }
  uint8_t driveIndex = webDriveArg();
  if (validDriveIndex(driveIndex)) {
    startRaiseSequence(driveIndex);
  }
  server.send(204, "text/plain", "");
}

void handleValve() {
  if (rejectCommandDuringStartup()) {
    return;
  }
  if (!server.hasArg("valve") || !server.hasArg("state") ||
      (server.arg("state") != "0" && server.arg("state") != "1") ||
      (server.arg("valve") != "fill" && server.arg("valve") != "drain")) {
    server.send(400, "text/plain; charset=UTF-8", "Неверная команда крана");
    return;
  }
  uint8_t valveIndex = server.arg("valve") == "fill" ? 0 : 1;
  bool enabled = server.arg("state") == "1";
  if (!canChangeValve(valveIndex, enabled)) {
    uint32_t remainingSeconds = (valveLockRemainingMs(valveIndex) + 999) / 1000;
    String error = remainingSeconds > 0
      ? "Кран занят: повторите через " + String(remainingSeconds) + " с"
      : "Кран уже находится в запрошенном состоянии";
    server.send(409, "text/plain; charset=UTF-8", error);
    return;
  }
  requestValveState(valveIndex, enabled);
  server.send(204, "text/plain", "");
}

void handleHeater() {
  if (rejectCommandDuringStartup()) {
    return;
  }
  if (server.hasArg("state")) {
    setHeater(server.arg("state").toInt() != 0);
  }
  server.send(204, "text/plain", "");
}

void handleNetworkSettings() {
  if (rejectCommandDuringStartup()) {
    return;
  }
  if (!server.hasArg("mode") || (server.arg("mode") != "static" && server.arg("mode") != "dhcp")) {
    server.send(400, "text/plain; charset=UTF-8", "Выберите статический IP или DHCP.");
    return;
  }

  NetworkSettings requested = networkSettings;
  requested.useDhcp = server.arg("mode") == "dhcp" ? 1 : 0;
  if (!requested.useDhcp) {
    IPAddress ip, subnet, gateway, dns;
    if (!server.hasArg("ip") || !server.hasArg("subnet") ||
        !server.hasArg("gateway") || !server.hasArg("dns") ||
        !parseIPv4(server.arg("ip"), ip) || !parseIPv4(server.arg("subnet"), subnet) ||
        !parseIPv4(server.arg("gateway"), gateway) || !parseIPv4(server.arg("dns"), dns)) {
      server.send(400, "text/plain; charset=UTF-8", "Проверьте формат IPv4-адресов.");
      return;
    }
    requested.ip = static_cast<uint32_t>(ip);
    requested.subnet = static_cast<uint32_t>(subnet);
    requested.gateway = static_cast<uint32_t>(gateway);
    requested.dns = static_cast<uint32_t>(dns);
  }
  if (!validNetworkSettings(requested)) {
    server.send(400, "text/plain; charset=UTF-8", "Недопустимые IP, маска или шлюз.");
    return;
  }
  if (!saveNetworkSettings(requested)) {
    server.send(500, "text/plain; charset=UTF-8", "Не удалось сохранить настройки в память ESP32.");
    return;
  }
  networkSettings = requested;
  networkApplyPending = true;
  networkApplyRequestedAtMs = millis();
  server.send(200, "text/plain; charset=UTF-8",
              "Настройки сохранены. Ethernet переключится примерно через секунду; подключитесь к новому IP.");
}

void handleRedirectToRoot() {
  server.sendHeader("Location", String("http://") + WiFi.softAPIP().toString() + "/", true);
  server.send(302, "text/plain", "");
}

void setup() {
  initializeRelayExpandersSafe();
  Serial.begin(115200);
  loadTimingSettings();
  loadNetworkSettings();
  initDriveStates();
  closeValvesByDefault();
  updateFloatSensor();
  dallasSensors.begin();
  dallasSensors.setWaitForConversion(false);
  dallasDeviceCount = dallasSensors.getDeviceCount();
  updateDallasTemperature(millis());

  WiFi.onEvent(onNetworkEvent);
  ethernetStarted = ETH.begin(KC_ETH_PHY_ADDRESS, KC_ETH_POWER_PIN, KC_ETH_MDC_PIN, KC_ETH_MDIO_PIN,
                               ETH_PHY_LAN8720, ETH_CLOCK_GPIO17_OUT);
  if (!ethernetStarted) {
    Serial.println("Ethernet: start failed");
  } else if (!applyEthernetSettings()) {
    Serial.println("Ethernet: IP configuration failed");
  }
  WiFi.softAP(ap_ssid, ap_password);
  delay(200);
  mb.server(MODBUS_TCP_PORT);
  mb.addHreg(REG_STAT_MODE, 1, 2);
  mb.addHreg(REG_LOWER_TIME_MS, 0, REG_COMMAND_DRIVE - REG_LOWER_TIME_MS + 1);
  mb.addIreg(INPUT_UPTIME_HOURS);
  mb.addIreg(INPUT_END_BASE, 0, DRIVE_COUNT * 2);
  mb.addIreg(INPUT_RELAY_BASE, 0, RELAY_CHANNEL_COUNT);
  mb.addIreg(INPUT_TEMPERATURE_X10, 0, INPUT_VALVE_LOCK_SECONDS - INPUT_TEMPERATURE_X10 + 1);
  mb.addIreg(INPUT_DRIVE_BASE, 0, DRIVE_COUNT * 5);
  mb.onRequest(validateModbusRequest);
  mb.onRequestSuccess(onModbusRequestSuccess);
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
  server.on("/network", HTTP_POST, handleNetworkSettings);
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
  Serial.print("Modbus TCP port: ");
  Serial.println(MODBUS_TCP_PORT);
}

void loop() {
  static IPAddress lastEthernetIp;
  static bool lastEthernetLinkUp = false;
  bool ethernetLinkUp = ETH.linkUp();
  if (ethernetLinkUp != lastEthernetLinkUp) {
    Serial.println(ethernetLinkUp ? "Ethernet: cable link up" : "Ethernet: cable link down");
    lastEthernetLinkUp = ethernetLinkUp;
  }
  IPAddress ethernetIp = ETH.localIP();
  if (ethernetIp != lastEthernetIp) {
    Serial.print("Ethernet IP: ");
    Serial.println(ethernetIp);
    lastEthernetIp = ethernetIp;
  }
  mb.task();
  dnsServer.processNextRequest();

  for (uint8_t i = 0; i < DRIVE_COUNT; i++) {
    updateDriveSequence(i);
  }
  if (startupInProgress && !anyDriveBusy() &&
      valveLockRemainingMs(0) == 0 && valveLockRemainingMs(1) == 0) {
    startupInProgress = false;
    Serial.println("Startup complete: commands enabled");
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
  if (networkApplyPending && millis() - networkApplyRequestedAtMs >= 1000) {
    networkApplyPending = false;
    if (!applyEthernetSettings()) {
      Serial.println("Ethernet: updated IP configuration failed");
    } else {
      Serial.print("Ethernet: updated IP mode: ");
      Serial.println(networkSettings.useDhcp ? "DHCP" : "static");
    }
  }
  mb.task();
}
