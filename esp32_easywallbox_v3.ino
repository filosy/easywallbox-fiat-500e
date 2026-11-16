/*
 * ======================================================================================
 * CONTROLLER FIAT 500e + EASYWALLBOX (eWB07491) — [REV 3.1 - BLE PARING + DPM AD]
 * ======================================================================================
 * RIEPILOGO FUNZIONI (REV 3.1):
 * - Connessione BLE sicura alla easyWallbox (eWB07491) con pairing passkey 01234.
 * - Autenticazione applicativa con $BLE,AUTH,9093.
 * - Polling periodico $DATA,READ,AD per leggere il consumo del sensore DPM.
 * - Pagina 2 aggiornata con stima potenza domestica e totale DPM.
 * - Tensione nominale della stima DPM configurabile in DPM_NOMINAL_VOLTAGE_V.
 * - Ripristino DPM ON dopo timeout e stop automatico all'80%.
 * ======================================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <NimBLEDevice.h>
#include <stdlib.h>

#define FW_REVISION       "REV 3.1"
#define EWB_NAME          "eWB07491"
#define EWB_MAC_ADDR      "60:a4:23:7c:af:b1"
#define EWB_BLE_PIN       "9093"
#define EWB_PAIRING_CODE  1234UL

static NimBLEUUID serviceUUID("331a36f5-2459-45ea-9d95-6142f0c4b307");

// Wallbox RX: characteristic used by this ESP32 to write commands.
static NimBLEUUID charWriteUUID("a9da6040-0823-4995-94ec-9ce41ca28833");

// Wallbox TX and status characteristics: notifications from the wallbox.
static NimBLEUUID charNotifyUUID("a73e9a10-628f-4494-a099-12efaf72258f");
static NimBLEUUID charStatusUUID("75a9f022-af03-4e41-b4bc-9de90a47d50b");

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define ENC_CLK 32
#define ENC_DT  33
#define ENC_SW  25

const float NET_BATTERY_KWH = 37.3f;
const int TARGET_SOC = 80;
const float AC_LOSS_FACTOR = 1.12f;
const float FALLBACK_POWER_KW = 2.05f;
const float NOMINAL_VOLTAGE_V = 230.0f;
const float DPM_NOMINAL_VOLTAGE_V = 230.0f;  // da regolare in base alla tensione domestica

const unsigned long DPM_OFF_DURATION_SEC = 15UL * 60UL;
const unsigned long OLED_TIMEOUT_MS = 20UL * 60UL * 1000UL;
const unsigned long LONG_PRESS_MS = 2000UL;
const unsigned long PAGE1_DURATION_MS = 30000UL;
const unsigned long PAGE2_DURATION_MS = 10000UL;
const unsigned long AUTH_DELAY_MS = 8000UL;
const unsigned long AD_POLL_INTERVAL_MS = 10000UL;

enum SystemState {
  STATE_SELECT_SOC,
  STATE_WAITING_START,
  STATE_CHARGING,
  STATE_COMPLETED
};

SystemState currentState = STATE_SELECT_SOC;

int startSoC = 45;
int lastCLKState = HIGH;

bool btnIsDown = false;
bool longPressHandled = false;
unsigned long btnDownTime = 0;

float targetGrossKwh = 0.0f;
float sessionStartKwh = -1.0f;
float deliveredKwh = 0.0f;
float liveCurrentA = 0.0f;
float wallboxPowerKw = 0.0f;
float homePowerKw = 0.0f;
float dpmTotalPowerKw = 0.0f;

bool hasLiveEnergyData = false;
bool hasHomePowerData = false;

unsigned long fallbackDurationSec = 0;
unsigned long elapsedChargingSec = 0;
unsigned long lastTickMillis = 0;

bool dpmTemporarilyOff = false;
unsigned long dpmOffStartMillis = 0;
bool hourglassAnimFrame = false;

NimBLEClient* pClient = nullptr;
NimBLERemoteCharacteristic* pCharWrite = nullptr;
NimBLERemoteCharacteristic* pCharNotify = nullptr;
NimBLERemoteCharacteristic* pCharStatus = nullptr;

bool bleConnected = false;
bool bleAuthenticated = false;
unsigned long lastReconnectAttempt = 0;
unsigned long lastAdPollMillis = 0;
String bleRxBuffer;

bool screenNeedsUpdate = true;
bool oledIsOn = true;
unsigned long lastUserActivityMs = 0;
int activeDisplayPage = 1;

int ticker1X = 128;
int ticker2X = 128;
unsigned long lastTickerMillis = 0;
String currentTicker1Text = "";
String currentTicker2Text = "";

class WallboxClientCallbacks : public NimBLEClientCallbacks {
public:
  void onPassKeyEntry(NimBLEConnInfo& connInfo) override {
    Serial.println("[BLE] Pairing passkey requested; providing 01234.");
    NimBLEDevice::injectPassKey(connInfo, EWB_PAIRING_CODE);
  }

  void onAuthenticationComplete(NimBLEConnInfo& connInfo) override {
    Serial.printf("[BLE] Pairing complete; encrypted: %s\n",
                  connInfo.isEncrypted() ? "YES" : "NO");
  }
};

static WallboxClientCallbacks wallboxClientCallbacks;

void wakeUpOledAndResetPage() {
  lastUserActivityMs = millis();

  if (!oledIsOn) {
    display.ssd1306_command(SSD1306_DISPLAYON);
    oledIsOn = true;
  }

  if (activeDisplayPage != 1) {
    activeDisplayPage = 1;
  }

  screenNeedsUpdate = true;
}

void drawHourglass(int x, int y, bool frame) {
  display.drawLine(x, y, x + 8, y, SSD1306_WHITE);
  display.drawLine(x, y + 11, x + 8, y + 11, SSD1306_WHITE);
  display.drawLine(x + 1, y + 1, x + 4, y + 5, SSD1306_WHITE);
  display.drawLine(x + 7, y + 1, x + 4, y + 5, SSD1306_WHITE);
  display.drawLine(x + 4, y + 6, x + 1, y + 10, SSD1306_WHITE);
  display.drawLine(x + 4, y + 6, x + 7, y + 10, SSD1306_WHITE);

  if (!frame) {
    display.drawLine(x + 2, y + 2, x + 6, y + 2, SSD1306_WHITE);
    display.drawLine(x + 3, y + 3, x + 5, y + 3, SSD1306_WHITE);
    display.drawPixel(x + 4, y + 7, SSD1306_WHITE);
    display.drawLine(x + 3, y + 10, x + 5, y + 10, SSD1306_WHITE);
  } else {
    display.drawPixel(x + 4, y + 4, SSD1306_WHITE);
    display.drawPixel(x + 4, y + 8, SSD1306_WHITE);
    display.drawLine(x + 3, y + 9, x + 5, y + 9, SSD1306_WHITE);
    display.drawLine(x + 2, y + 10, x + 6, y + 10, SSD1306_WHITE);
  }
}

void sendBLECommand(const String& cmd) {
  if (!bleConnected || pCharWrite == nullptr) {
    Serial.printf("[EWB TX] skipped (not connected): %s\n", cmd.c_str());
    return;
  }

  String fullCmd = cmd + "\n";
  bool ok = pCharWrite->writeValue((uint8_t*)fullCmd.c_str(), fullCmd.length(), false);

  if (!ok) {
    ok = pCharWrite->writeValue((uint8_t*)fullCmd.c_str(), fullCmd.length(), true);
  }

  Serial.printf("[EWB TX] %s -> %s\n", cmd.c_str(), ok ? "OK" : "ERR");
}

unsigned long getDpmRemainingSec() {
  if (!dpmTemporarilyOff) return 0;

  unsigned long elapsed = (millis() - dpmOffStartMillis) / 1000UL;
  if (elapsed >= DPM_OFF_DURATION_SEC) return 0;

  return DPM_OFF_DURATION_SEC - elapsed;
}

void toggleDpmOff15Min() {
  if (!dpmTemporarilyOff) {
    dpmTemporarilyOff = true;
    dpmOffStartMillis = millis();
    sendBLECommand("$CMD,DPM,0");
    Serial.println("DPM OFF enabled for 15 minutes.");
  } else {
    dpmTemporarilyOff = false;
    sendBLECommand("$CMD,DPM,1");
    Serial.println("DPM OFF cancelled; restoring DPM ON.");
  }

  screenNeedsUpdate = true;
}

String getCsvField(const String& data, int targetIndex) {
  int fieldIdx = 0;
  int startPos = 0;
  int len = data.length();

  for (int i = 0; i <= len; i++) {
    if (i == len || data.charAt(i) == ',') {
      if (fieldIdx == targetIndex) {
        return data.substring(startPos, i);
      }
      fieldIdx++;
      startPos = i + 1;
    }
  }

  return "";
}

bool isUnsignedInteger(const String& value) {
  if (value.length() == 0) return false;

  for (int i = 0; i < value.length(); i++) {
    if (!isDigit(value.charAt(i))) return false;
  }

  return true;
}

void processWallboxLine(String resp) {
  resp.trim();
  if (resp.length() == 0) return;

  Serial.println("[EWB RX] " + resp);

  if (resp.indexOf("$BLE,AUTH,OK") >= 0) {
    bleAuthenticated = true;
    lastAdPollMillis = millis() - AD_POLL_INTERVAL_MS;
    Serial.println("[BLE] Application authentication confirmed.");
    screenNeedsUpdate = true;
  }

  if (resp.indexOf("$BLE,AUTH,FAIL") >= 0 || resp.indexOf("$ERR,") >= 0) {
    bleAuthenticated = false;
    Serial.println("[BLE] Wallbox reported an error.");
  }

  /*
   * Observed reply format:
   * $DATA,READ,AD,<field0>,...,<field21>
   *
   * Absolute CSV indexes:
   * 7  = charge state
   * 10 = charging current in tenths of an amp
   * 11 = charging power in watts
   * 14 = DPM measured current in tenths of an amp
   * 22 = session energy in 0.1 kWh
   */
  if (resp.startsWith("$DATA,READ,AD,")) {
    String stateField = getCsvField(resp, 7);
    String carCurrentField = getCsvField(resp, 10);
    String carPowerField = getCsvField(resp, 11);
    String dpmCurrentField = getCsvField(resp, 14);
    String energyField = getCsvField(resp, 22);

    int stateVal = stateField.toInt();

    if (isUnsignedInteger(carCurrentField)) {
      float currentA = carCurrentField.toFloat() / 10.0f;
      if (currentA >= 0.0f && currentA < 40.0f) {
        liveCurrentA = currentA;
      }
    }

    if (isUnsignedInteger(carPowerField)) {
      float powerW = carPowerField.toFloat();
      if (powerW >= 0.0f) {
        wallboxPowerKw = powerW / 1000.0f;
      }
    } else {
      wallboxPowerKw = liveCurrentA * NOMINAL_VOLTAGE_V / 1000.0f;
    }

    float sessionKwh = -1.0f;
    if (isUnsignedInteger(energyField)) {
      sessionKwh = energyField.toFloat() / 10.0f;
    }

    if (stateVal == 8) {
      if (currentState == STATE_WAITING_START) {
        currentState = STATE_CHARGING;
        lastTickMillis = millis();

        sessionStartKwh = sessionKwh;
        deliveredKwh = 0.0f;
        hasLiveEnergyData = sessionKwh >= 0.0f;
      } else if (currentState == STATE_CHARGING &&
                 sessionKwh >= 0.0f &&
                 sessionStartKwh >= 0.0f) {
        deliveredKwh = sessionKwh - sessionStartKwh;
        if (deliveredKwh < 0.0f) deliveredKwh = sessionKwh;
        hasLiveEnergyData = true;
      }

      screenNeedsUpdate = true;
    } else if (currentState == STATE_CHARGING && liveCurrentA < 0.5f) {
      wallboxPowerKw = 0.0f;
    }

    if (isUnsignedInteger(dpmCurrentField)) {
      float dpmCurrentDa = dpmCurrentField.toFloat();

      // Formula con tensione nominale configurabile.
      dpmTotalPowerKw = (dpmCurrentDa * DPM_NOMINAL_VOLTAGE_V) / 10000.0f;

      // Il sensore DPM misura il totale; stimo la casa sottraendo la potenza carica dell'auto.
      float carPowerKw = (stateVal == 8) ? wallboxPowerKw : 0.0f;
      homePowerKw = dpmTotalPowerKw - carPowerKw;
      if (homePowerKw < 0.0f) homePowerKw = 0.0f;

      hasHomePowerData = true;

      Serial.printf("[DPM AD] dpm=%s dA, total=%.2f kW, house-estimate=%.2f kW\n",
                    dpmCurrentField.c_str(),
                    dpmTotalPowerKw,
                    homePowerKw);
    }

    screenNeedsUpdate = true;
  }

  // Legacy support for $STS if the wallbox sends it.
  if (resp.startsWith("$STS,")) {
    String fieldState = getCsvField(resp, 1);
    String fieldCurr = getCsvField(resp, 2);
    String fieldPowW = getCsvField(resp, 3);
    String fieldWh = getCsvField(resp, 4);

    if (fieldState.length() > 0 && fieldCurr.length() > 0) {
      int stateVal = fieldState.toInt();
      float currA = fieldCurr.toFloat() / 10.0f;

      if (currA >= 0.0f && currA < 40.0f) {
        liveCurrentA = currA;

        if (fieldPowW.length() > 0 && fieldPowW.toFloat() > 0.0f) {
          wallboxPowerKw = fieldPowW.toFloat() / 1000.0f;
        } else {
          wallboxPowerKw = liveCurrentA * NOMINAL_VOLTAGE_V / 1000.0f;
        }
      }

      if ((stateVal == 8 || liveCurrentA > 3.0f) &&
          currentState == STATE_WAITING_START) {
        currentState = STATE_CHARGING;
        lastTickMillis = millis();
        screenNeedsUpdate = true;
      } else if (currentState == STATE_CHARGING && liveCurrentA < 0.5f) {
        wallboxPowerKw = 0.0f;
      }
    }

    if (fieldWh.length() > 0) {
      float rawWh = fieldWh.toFloat();
      if (rawWh >= 0.0f) {
        float currentKwh = rawWh / 1000.0f;

        if (sessionStartKwh < 0.0f) {
          sessionStartKwh = currentKwh;
        }

        deliveredKwh = (currentKwh >= sessionStartKwh)
          ? currentKwh - sessionStartKwh
          : currentKwh;

        hasLiveEnergyData = true;
        screenNeedsUpdate = true;
      }
    }
  }
}

void notifyCallback(NimBLERemoteCharacteristic* characteristic, uint8_t* data, size_t length, bool isNotify) {
  (void)characteristic;
  (void)isNotify;

  for (size_t i = 0; i < length; i++) {
    char c = (char)data[i];

    if (c == '\r') continue;

    if (c == '\n') {
      processWallboxLine(bleRxBuffer);
      bleRxBuffer = "";
    } else if (bleRxBuffer.length() < 512) {
      bleRxBuffer += c;
    } else {
      Serial.println("[BLE] Notification buffer overflow; clearing it.");
      bleRxBuffer = "";
    }
  }
}

bool connectToWallbox() {
  Serial.println("Scanning BLE for eWB07491...");

  NimBLEScan* pScan = NimBLEDevice::getScan();
  pScan->setActiveScan(true);
  pScan->setInterval(100);
  pScan->setWindow(99);

  NimBLEScanResults results = pScan->getResults(4000, false);
  const NimBLEAdvertisedDevice* targetDevice = nullptr;

  for (int i = 0; i < results.getCount(); i++) {
    const NimBLEAdvertisedDevice* dev = results.getDevice(i);
    String devAddr = String(dev->getAddress().toString().c_str());
    devAddr.toLowerCase();

    if (devAddr == EWB_MAC_ADDR) {
      targetDevice = dev;
      break;
    }
  }

  if (targetDevice == nullptr) {
    pScan->clearResults();
    Serial.println("[BLE] Wallbox not found.");
    return false;
  }

  if (pClient == nullptr) {
    pClient = NimBLEDevice::createClient();
  }

  pClient->setClientCallbacks(&wallboxClientCallbacks, false);

  if (!pClient->connect(targetDevice)) {
    pScan->clearResults();
    bleConnected = false;
    bleAuthenticated = false;
    Serial.println("[BLE] Connection failed.");
    return false;
  }

  pScan->clearResults();

  Serial.println("[BLE] Connected; starting secure pairing...");

  if (!pClient->secureConnection()) {
    Serial.printf("[BLE] secureConnection failed; NimBLE error=%d\n", pClient->getLastError());
    pClient->disconnect();
    bleConnected = false;
    bleAuthenticated = false;
    return false;
  }

  Serial.println("[BLE] Secure pairing completed.");

  NimBLERemoteService* pService = pClient->getService(serviceUUID);
  if (pService == nullptr) {
    Serial.println("[BLE] EasyWallbox service not found.");
    pClient->disconnect();
    bleConnected = false;
    return false;
  }

  pCharWrite = pService->getCharacteristic(charWriteUUID);
  pCharNotify = pService->getCharacteristic(charNotifyUUID);
  pCharStatus = pService->getCharacteristic(charStatusUUID);

  if (pCharWrite == nullptr) {
    Serial.println("[BLE] Command-write characteristic not found.");
    pClient->disconnect();
    bleConnected = false;
    return false;
  }

  if (pCharNotify != nullptr && pCharNotify->canNotify()) {
    bool subscribed = pCharNotify->subscribe(true, notifyCallback, true);
    Serial.printf("[BLE] Main notification subscription: %s\n", subscribed ? "OK" : "FAILED");
  } else {
    Serial.println("[BLE] Main notification characteristic unavailable.");
  }

  if (pCharStatus != nullptr && pCharStatus->canNotify()) {
    bool subscribed = pCharStatus->subscribe(true, notifyCallback, true);
    Serial.printf("[BLE] Status notification subscription: %s\n", subscribed ? "OK" : "FAILED");
  } else {
    Serial.println("[BLE] Status notification characteristic unavailable.");
  }

  bleConnected = true;
  bleAuthenticated = false;
  bleRxBuffer = "";

  Serial.println("[BLE] Waiting 8 seconds before application AUTH...");
  delay(AUTH_DELAY_MS);

  if (!pClient->isConnected()) {
    Serial.println("[BLE] Connection lost before application AUTH.");
    bleConnected = false;
    return false;
  }

  Serial.println("[BLE] Sending application PIN.");
  sendBLECommand(String("$BLE,AUTH,") + EWB_BLE_PIN);
  return true;
}

String getDpmStatusPrefix() {
  return dpmTemporarilyOff ? "DPM: OFF (15m) | " : "DPM: ON | ";
}

String getTicker1Message() {
  String prefix = getDpmStatusPrefix();

  if (currentState == STATE_SELECT_SOC) {
    float netNeeded = ((TARGET_SOC - startSoC) / 100.0f) * NET_BATTERY_KWH;
    if (netNeeded < 0.0f) netNeeded = 0.0f;

    float grossNeeded = netNeeded * AC_LOSS_FACTOR;
    return prefix + "Target 80%: +" + String(grossNeeded, 1) + " kWh gross";
  }

  if (currentState == STATE_WAITING_START || currentState == STATE_CHARGING) {
    int estSoc = startSoC + (int)((deliveredKwh / AC_LOSS_FACTOR / NET_BATTERY_KWH) * 100.0f);
    estSoc = constrain(estSoc, startSoC, TARGET_SOC);
    return prefix + "Estimated battery: " + String(estSoc) + "% -> 80%";
  }

  return prefix + "Energy delivered: " + String(deliveredKwh, 2) + " kWh";
}

String getTicker2Message() {
  if (currentState == STATE_SELECT_SOC) {
    return "Rotate=SoC | Click=Start | Hold 2s=DPM OFF 15m";
  }

  if (currentState == STATE_WAITING_START || currentState == STATE_CHARGING) {
    return "Click=STOP charge | Hold 2s=DPM OFF/ON";
  }

  return "Charge complete! Click for new session";
}

void drawPage1() {
  display.setCursor(0, 0);

  if (bleConnected) {
    display.print(dpmTemporarilyOff ? "EWB:OK | DPM:OFF" : "EWB:OK | DPM:ON");
  } else {
    display.print(FW_REVISION);
    display.print(dpmTemporarilyOff ? " | DPM:OFF" : " | DPM:ON");
  }

  display.drawLine(0, 9, 127, 9, SSD1306_WHITE);

  if (currentState == STATE_SELECT_SOC) {
    display.setCursor(0, 11);
    display.print("BATTERY CURRENT:");
    display.setTextSize(3);
    display.setCursor(6, 20);
    display.printf("%d%%", startSoC);

    if (dpmTemporarilyOff) {
      unsigned long dpmRem = getDpmRemainingSec();
      drawHourglass(104, 13, hourglassAnimFrame);
      display.setTextSize(1);
      display.setCursor(95, 28);
      display.printf("%02lu:%02lu", dpmRem / 60, dpmRem % 60);
    }
  } else if (currentState == STATE_WAITING_START || currentState == STATE_CHARGING) {
    if (currentState == STATE_WAITING_START) {
      if (dpmTemporarilyOff) {
        unsigned long dpmRem = getDpmRemainingSec();
        drawHourglass(0, 11, hourglassAnimFrame);
        display.setCursor(14, 13);
        display.printf("DPM ON IN %02lu:%02lu", dpmRem / 60, dpmRem % 60);
      } else {
        display.setCursor(0, 13);
        display.print("WAITING FOR AUTO TIMER...");
      }
    } else {
      display.setCursor(0, 13);
      display.printf("CHARGE:%.1fA DPM:%s", liveCurrentA, dpmTemporarilyOff ? "OFF" : "ON");
    }

    display.setTextSize(2);
    display.setCursor(4, 25);
    display.printf("%.1f/%.1fkWh", deliveredKwh, targetGrossKwh);
  } else if (currentState == STATE_COMPLETED) {
    display.setTextSize(2);
    display.setCursor(6, 18);
    display.print("STOP 80%!");
  }

  String msg1 = getTicker1Message();
  if (msg1 != currentTicker1Text) {
    currentTicker1Text = msg1;
    ticker1X = 128;
  }

  display.setTextSize(1);
  display.setCursor(ticker1X, 44);
  display.print(currentTicker1Text);
  display.drawLine(0, 53, 127, 53, SSD1306_WHITE);

  String msg2 = getTicker2Message();
  if (msg2 != currentTicker2Text) {
    currentTicker2Text = msg2;
    ticker2X = 128;
  }

  display.setCursor(ticker2X, 55);
  display.print(currentTicker2Text);
}

void drawPage2() {
  bool isDpmOn = !dpmTemporarilyOff;
  bool isCharging = currentState == STATE_CHARGING;

  display.setCursor(0, 0);
  display.printf("CHARGE INFO | DPM:%s", isDpmOn ? "ON" : "OFF");
  display.drawLine(0, 9, 127, 9, SSD1306_WHITE);

  display.setCursor(0, 12);
  if (isDpmOn && hasHomePowerData) {
    display.printf("House est.: %.2f kW", homePowerKw);
  } else if (isDpmOn) {
    display.print("House est.: --.-- kW");
  } else {
    display.print("House est.: N/A (DPM OFF)");
  }

  float activeEwbKw = 0.0f;
  if (isCharging) {
    activeEwbKw = wallboxPowerKw > 0.1f ? wallboxPowerKw : FALLBACK_POWER_KW * AC_LOSS_FACTOR;
  }

  display.setCursor(0, 22);
  if (isCharging) {
    display.printf("EWB:     %.2f kW", activeEwbKw);
  } else {
    display.print("EWB:     0.00 kW (Standby)");
  }

  display.setCursor(0, 32);
  if (isDpmOn && hasHomePowerData) {
    display.printf("DPM total: %.2f kW", dpmTotalPowerKw);
  } else {
    display.printf("DPM total: N/A");
  }

  display.drawLine(0, 41, 127, 41, SSD1306_WHITE);

  float plannedGross = currentState == STATE_SELECT_SOC
    ? ((TARGET_SOC - startSoC) / 100.0f) * NET_BATTERY_KWH * AC_LOSS_FACTOR
    : targetGrossKwh;

  if (plannedGross < 0.0f) plannedGross = 0.0f;

  float remainingKwh = plannedGross - deliveredKwh;
  if (remainingKwh < 0.0f) remainingKwh = 0.0f;

  display.setCursor(0, 44);
  display.printf("In:%.1fkWh | Rem:%.1fkWh", deliveredKwh, remainingKwh);

  float activeKwForEta = activeEwbKw > 0.5f ? activeEwbKw : FALLBACK_POWER_KW * AC_LOSS_FACTOR;
  unsigned long remMinutes = (unsigned long)((remainingKwh / activeKwForEta) * 60.0f);

  display.setCursor(0, 55);
  if (currentState == STATE_COMPLETED || remainingKwh <= 0.05f) {
    display.print("Time left: 0h 00m (OK)");
  } else {
    display.printf("Time left: %luh %02lum", remMinutes / 60, remMinutes % 60);
  }
}

void updateDisplay() {
  if (!oledIsOn) return;

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setTextWrap(false);

  if (activeDisplayPage == 1) {
    drawPage1();
  } else {
    drawPage2();
  }

  display.display();
  screenNeedsUpdate = false;
}

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("Starting Fiat 500e + EasyWallbox controller - " + String(FW_REVISION));

  pinMode(ENC_CLK, INPUT_PULLUP);
  pinMode(ENC_DT, INPUT_PULLUP);
  pinMode(ENC_SW, INPUT_PULLUP);
  lastCLKState = digitalRead(ENC_CLK);

  Wire.begin(21, 22);

  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println("SSD1306 OLED error.");
  }

  display.clearDisplay();
  display.display();

  NimBLEDevice::init("ESP32_500e_Controller");
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);
  NimBLEDevice::setSecurityAuth(true, true, true);
  NimBLEDevice::setSecurityIOCap(BLE_HS_IO_KEYBOARD_ONLY);

  lastUserActivityMs = millis();
  updateDisplay();
}

void loop() {
  unsigned long now = millis();

  if (oledIsOn && now - lastUserActivityMs > OLED_TIMEOUT_MS) {
    display.ssd1306_command(SSD1306_DISPLAYOFF);
    oledIsOn = false;
  }

  if (oledIsOn) {
    unsigned long idleTime = now - lastUserActivityMs;

    if (idleTime >= PAGE1_DURATION_MS) {
      unsigned long cyclePos = (idleTime - PAGE1_DURATION_MS) % (PAGE2_DURATION_MS + PAGE1_DURATION_MS);
      int desiredPage = cyclePos < PAGE2_DURATION_MS ? 2 : 1;

      if (desiredPage != activeDisplayPage) {
        activeDisplayPage = desiredPage;
        screenNeedsUpdate = true;
      }
    }
  }

  if (oledIsOn && activeDisplayPage == 1 && now - lastTickerMillis >= 80UL) {
    lastTickerMillis = now;

    ticker1X -= 2;
    if (ticker1X < -((int)currentTicker1Text.length() * 6)) {
      ticker1X = 128;
    }

    ticker2X -= 2;
    if (ticker2X < -((int)currentTicker2Text.length() * 6)) {
      ticker2X = 128;
    }

    screenNeedsUpdate = true;
  }

  int currentCLK = digitalRead(ENC_CLK);

  if (currentCLK != lastCLKState && currentCLK == LOW) {
    wakeUpOledAndResetPage();

    if (currentState == STATE_SELECT_SOC) {
      if (digitalRead(ENC_DT) != currentCLK) {
        startSoC++;
      } else {
        startSoC--;
      }

      startSoC = constrain(startSoC, 5, 79);
      screenNeedsUpdate = true;
    }
  }

  lastCLKState = currentCLK;

  bool btnReading = digitalRead(ENC_SW) == LOW;

  if (btnReading && !btnIsDown) {
    if (now - btnDownTime > 80UL) {
      btnIsDown = true;
      longPressHandled = false;
      btnDownTime = now;
      wakeUpOledAndResetPage();
    }
  } else if (btnReading && btnIsDown) {
    if (!longPressHandled && now - btnDownTime >= LONG_PRESS_MS) {
      longPressHandled = true;
      toggleDpmOff15Min();
    }
  } else if (!btnReading && btnIsDown) {
    btnIsDown = false;

    if (!longPressHandled && now - btnDownTime >= 80UL) {
      if (currentState == STATE_SELECT_SOC) {
        float netKwh = ((TARGET_SOC - startSoC) / 100.0f) * NET_BATTERY_KWH;

        targetGrossKwh = netKwh * AC_LOSS_FACTOR;
        fallbackDurationSec = (unsigned long)((netKwh / FALLBACK_POWER_KW) * 3600.0f);

        elapsedChargingSec = 0;
        deliveredKwh = 0.0f;
        sessionStartKwh = -1.0f;
        hasLiveEnergyData = false;
        lastTickMillis = now;
        currentState = STATE_WAITING_START;
        screenNeedsUpdate = true;
      } else if (currentState == STATE_COMPLETED) {
        currentState = STATE_SELECT_SOC;
        screenNeedsUpdate = true;
      } else if (currentState == STATE_WAITING_START || currentState == STATE_CHARGING) {
        if (dpmTemporarilyOff) {
          dpmTemporarilyOff = false;
          sendBLECommand("$CMD,DPM,1");
          delay(200);
        }

        sendBLECommand("$CMD,CHARGE,0");
        currentState = STATE_COMPLETED;
        screenNeedsUpdate = true;
      }
    }
  }

  if (dpmTemporarilyOff && getDpmRemainingSec() == 0) {
    Serial.println("DPM OFF timer expired; restoring DPM ON.");
    dpmTemporarilyOff = false;
    sendBLECommand("$CMD,DPM,1");
    wakeUpOledAndResetPage();
    screenNeedsUpdate = true;
  }

  if (pClient == nullptr || !pClient->isConnected()) {
    if (bleConnected) {
      bleConnected = false;
      bleAuthenticated = false;
      screenNeedsUpdate = true;
      Serial.println("[BLE] Connection lost.");
    }

    if (now - lastReconnectAttempt > 7000UL) {
      lastReconnectAttempt = now;
      connectToWallbox();
      screenNeedsUpdate = true;
    }
  }

  if (bleConnected && bleAuthenticated && now - lastAdPollMillis >= AD_POLL_INTERVAL_MS) {
    lastAdPollMillis = now;
    sendBLECommand("$DATA,READ,AD");
  }

  if (now - lastTickMillis >= 1000UL) {
    lastTickMillis += 1000UL;

    if (dpmTemporarilyOff) {
      hourglassAnimFrame = !hourglassAnimFrame;
      if (oledIsOn) screenNeedsUpdate = true;
    }

    if (currentState == STATE_CHARGING) {
      elapsedChargingSec++;

      if (!hasLiveEnergyData && fallbackDurationSec > 0) {
        deliveredKwh = ((float)elapsedChargingSec / (float)fallbackDurationSec) * targetGrossKwh;
      }

      if (elapsedChargingSec % 5 == 0 && oledIsOn) {
        screenNeedsUpdate = true;
      }

      bool kwhReached = hasLiveEnergyData && deliveredKwh >= targetGrossKwh;
      bool timeReached = !hasLiveEnergyData && fallbackDurationSec > 0 && elapsedChargingSec >= fallbackDurationSec;

      if (kwhReached || timeReached) {
        Serial.println("Target reached; sending STOP.");

        if (dpmTemporarilyOff) {
          dpmTemporarilyOff = false;
          sendBLECommand("$CMD,DPM,1");
          delay(200);
        }

        sendBLECommand("$CMD,CHARGE,0");
        delay(500);
        sendBLECommand("$CMD,CHARGE,0");

        currentState = STATE_COMPLETED;
        wakeUpOledAndResetPage();
        screenNeedsUpdate = true;
      }
    }
  }

  if (screenNeedsUpdate) {
    updateDisplay();
  }
}
