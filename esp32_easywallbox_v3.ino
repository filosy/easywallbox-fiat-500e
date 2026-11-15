/*
 * ======================================================================================
 * CONTROLLER FIAT 500e + EASYWALLBOX (eWB07491) — [REV 3.0 - BASE STABILE]
 * ======================================================================================
 * RIEPILOGO FUNZIONI (REV 3.0):
 * - Connessione BLE pulita alla easyWallbox (eWB07491) senza comandi superflui all'avvio.
 * - Pagina 1 (30s): Selezione SoC iniziale (5%..79%), target 80%, clessidra DPM OFF 15m
 *   su pressione lunga (2s) e 2 righe scorrevoli informative.
 * - Pagina 2 (10s): Riepilogo stato DPM, potenza colonnina/casa, kWh entrati/mancanti
 *   e tempo stimato al completamento.
 * - Arresto automatico ricarica all'80% ($CMD,CHARGE,0) e ripristino DPM ON.
 * - Spegnimento automatico display OLED dopo 20 minuti di inattività.
 * ======================================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <NimBLEDevice.h>

#define FW_REVISION     "REV 3.0"
#define EWB_NAME        "eWB07491"
#define EWB_MAC_ADDR    "60:a4:23:7c:af:b1"
#define EWB_BLE_PIN     "9093"

static NimBLEUUID serviceUUID("331a36f5-2459-45ea-9d95-6142f0c4b307");
static NimBLEUUID charTX_UUID("a9da6040-0823-4995-94ec-9ce41ca28833");
static NimBLEUUID charRX_UUID("a73e9a10-628f-4494-a099-12efaf72258f");

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define ENC_CLK      32
#define ENC_DT       33
#define ENC_SW       25

const float NET_BATTERY_KWH          = 37.3;
const int   TARGET_SOC               = 80;
const float AC_LOSS_FACTOR           = 1.12;
const float FALLBACK_POWER_KW        = 2.05;
const float NOMINAL_VOLTAGE_V        = 230.0;
const unsigned long DPM_OFF_DURATION_SEC = 15UL * 60UL;        // 15 minuti
const unsigned long OLED_TIMEOUT_MS      = 20UL * 60UL * 1000UL; // 20 minuti
const unsigned long LONG_PRESS_MS        = 2000;               // 2 secondi

const unsigned long PAGE1_DURATION_MS    = 30000UL; // 30s Pagina 1
const unsigned long PAGE2_DURATION_MS    = 10000UL; // 10s Pagina 2

enum SystemState {
  STATE_SELECT_SOC,
  STATE_WAITING_START,
  STATE_CHARGING,
  STATE_COMPLETED
};
SystemState currentState = STATE_SELECT_SOC;

int startSoC = 45;
int lastCLKState;

bool btnIsDown = false;
bool longPressHandled = false;
unsigned long btnDownTime = 0;

// Dati Energetici
float targetGrossKwh = 0.0;
float sessionStartKwh = -1.0;
float deliveredKwh = 0.0;
float liveCurrentA = 0.0;
float wallboxPowerKw = 0.0;
float homePowerKw = 0.0;
bool  hasLiveEnergyData = false;
bool  hasHomePowerData = false;

unsigned long fallbackDurationSec = 0;
unsigned long elapsedChargingSec = 0;
unsigned long lastTickMillis = 0;

// Gestione DPM
bool dpmTemporarilyOff = false;
unsigned long dpmOffStartMillis = 0;
bool hourglassAnimFrame = false;

// Connessione BLE
NimBLEClient* pClient = nullptr;
NimBLERemoteCharacteristic* pCharTX = nullptr;
NimBLERemoteCharacteristic* pCharRX = nullptr;
bool bleConnected = false;
bool bleAuthenticated = false;
unsigned long lastReconnectAttempt = 0;

// Display & Pagine
bool screenNeedsUpdate = true;
bool oledIsOn = true;
unsigned long lastUserActivityMs = 0;
int activeDisplayPage = 1;

int ticker1X = 128;
int ticker2X = 128;
unsigned long lastTickerMillis = 0;
String currentTicker1Text = "";
String currentTicker2Text = "";

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

void sendBLECommand(String cmd) {
  if (bleConnected && pCharTX != nullptr) {
    String fullCmd = cmd + "\n";
    bool ok = pCharTX->writeValue((uint8_t*)fullCmd.c_str(), fullCmd.length(), false);
    if (!ok) {
      ok = pCharTX->writeValue((uint8_t*)fullCmd.c_str(), fullCmd.length(), true);
    }
    Serial.printf("[EWB TX] %s -> %s\n", cmd.c_str(), ok ? "OK" : "ERR");
  }
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
    Serial.println("Pressione 2s: DPM OFF attivato per 15 minuti.");
  } else {
    dpmTemporarilyOff = false;
    sendBLECommand("$CMD,DPM,1");
    Serial.println("Pressione 2s: DPM OFF annullato -> DPM ON.");
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

void notifyCallback(NimBLERemoteCharacteristic* pRemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
  String resp = "";
  for (size_t i = 0; i < length; i++) {
    resp += (char)pData[i];
  }
  resp.trim();
  if (resp.length() == 0) return;

  Serial.println("[EWB RX] " + resp);

  if (resp.indexOf("$BLE,AUTH,OK") >= 0) {
    bleAuthenticated = true;
    if (dpmTemporarilyOff) {
      sendBLECommand("$CMD,DPM,0");
    }
    screenNeedsUpdate = true;
  }

  if (resp.startsWith("$STS,")) {
    String fieldState = getCsvField(resp, 1);
    String fieldCurr  = getCsvField(resp, 2);
    String fieldPowW  = getCsvField(resp, 3);
    String fieldWh    = getCsvField(resp, 4);
    String fieldHome  = getCsvField(resp, 5);

    if (fieldState.length() > 0 && fieldCurr.length() > 0) {
      int stateVal = fieldState.toInt();
      float currA = fieldCurr.toFloat() / 10.0;
      if (currA >= 0.0 && currA < 40.0) {
        liveCurrentA = currA;
        if (fieldPowW.length() > 0 && fieldPowW.toFloat() > 0) {
          wallboxPowerKw = fieldPowW.toFloat() / 1000.0;
        } else {
          wallboxPowerKw = (liveCurrentA * NOMINAL_VOLTAGE_V) / 1000.0;
        }
      }

      if (stateVal == 3 || fieldState.indexOf("CHARG") >= 0 || liveCurrentA > 3.0) {
        if (currentState == STATE_WAITING_START) {
          currentState = STATE_CHARGING;
          lastTickMillis = millis();
          screenNeedsUpdate = true;
        }
      } else if (currentState == STATE_CHARGING && liveCurrentA < 0.5) {
        wallboxPowerKw = 0.0;
      }
    }

    if (fieldWh.length() > 0) {
      float rawWh = fieldWh.toFloat();
      if (rawWh >= 0.0) {
        float currentKwh = rawWh / 1000.0;
        if (sessionStartKwh < 0.0) {
          sessionStartKwh = currentKwh;
        }
        deliveredKwh = (currentKwh >= sessionStartKwh) ? (currentKwh - sessionStartKwh) : currentKwh;
        hasLiveEnergyData = true;
      }
    }

    if (fieldHome.length() > 0) {
      float rawHome = fieldHome.toFloat();
      if (rawHome >= 0.0) {
        homePowerKw = (rawHome > 50.0) ? (rawHome / 1000.0) : ((rawHome / 10.0) * NOMINAL_VOLTAGE_V / 1000.0);
        hasHomePowerData = true;
        screenNeedsUpdate = true;
      }
    }
  }
}

bool connectToWallbox() {
  Serial.println("Scansione BLE per eWB07491...");
  
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
    return false;
  }

  if (pClient == nullptr) {
    pClient = NimBLEDevice::createClient();
  }

  if (!pClient->connect(targetDevice)) {
    pScan->clearResults();
    bleConnected = false;
    bleAuthenticated = false;
    return false;
  }

  pScan->clearResults();

  NimBLERemoteService* pService = pClient->getService(serviceUUID);
  if (pService == nullptr) {
    pClient->disconnect();
    return false;
  }

  pCharTX = pService->getCharacteristic(charTX_UUID);
  pCharRX = pService->getCharacteristic(charRX_UUID);

  if (pCharRX != nullptr && pCharRX->canNotify()) {
    pCharRX->subscribe(true, notifyCallback, true);
  }

  bleConnected = true;
  Serial.println("Connesso a eWB07491! Invio PIN...");
  delay(400);
  sendBLECommand(String("$BLE,AUTH,") + EWB_BLE_PIN);
  return true;
}

String getDpmStatusPrefix() {
  return dpmTemporarilyOff ? "DPM: OFF (15m) | " : "DPM: ON | ";
}

String getTicker1Message() {
String prefix = getDpmStatusPrefix();
if (currentState == STATE_SELECT_SOC) {
float netNeeded = ((TARGET_SOC - startSoC) / 100.0) * NET_BATTERY_KWH;
if (netNeeded < 0) netNeeded = 0;
float grossNeeded = netNeeded * AC_LOSS_FACTOR;
return prefix + "Target " + String(TARGET_SOC) + "%: +" + String(grossNeeded, 1) + " kWh lordi";
} else if (currentState == STATE_WAITING_START || currentState == STATE_CHARGING) {
int estSoc = startSoC + (int)((deliveredKwh / AC_LOSS_FACTOR / NET_BATTERY_KWH) * 100.0);
estSoc = constrain(estSoc, startSoC, TARGET_SOC);
return prefix + "Stima batteria: " + String(estSoc) + "% -> " + String(TARGET_SOC) + "%";
} else {
return prefix + "Energia erogata: " + String(deliveredKwh, 2) + " kWh";
}
}
String getTicker2Message() {
if (currentState == STATE_SELECT_SOC) {
return "Ruota=SoC | Click=Avvia | Tieni 2s=DPM OFF 15m";
} else if (currentState == STATE_WAITING_START || currentState == STATE_CHARGING) {
return "Click=STOP Ricarica | Tieni 2s=DPM OFF/ON 15m";
} else {
return "Ricarica completata! Click per nuova sessione";
}
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
display.print("BATTERIA ATTUALE:");
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
}
else if (currentState == STATE_WAITING_START || currentState == STATE_CHARGING) {
if (currentState == STATE_WAITING_START) {
if (dpmTemporarilyOff) {
unsigned long dpmRem = getDpmRemainingSec();
drawHourglass(0, 11, hourglassAnimFrame);
display.setCursor(14, 13);
display.printf("DPM ON TRA %02lu:%02lu", dpmRem / 60, dpmRem % 60);
} else {
display.setCursor(0, 13);
display.print("ATTESA TIMER AUTO...");
}
} else {
display.setCursor(0, 13);
display.printf("CARICA:%.1fA DPM:%s", liveCurrentA, dpmTemporarilyOff ? "OFF" : "ON");
}
display.setTextSize(2);
display.setCursor(4, 25);
display.printf("%.1f/%.1fkWh", deliveredKwh, targetGrossKwh);
}
else if (currentState == STATE_COMPLETED) {
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
bool isCharging = (currentState == STATE_CHARGING);
display.setCursor(0, 0);
display.printf("INFO CARICHI | DPM:%s", isDpmOn ? "ON" : "OFF");
display.drawLine(0, 9, 127, 9, SSD1306_WHITE);
display.setCursor(0, 12);
if (isDpmOn) {
if (hasHomePowerData) {
display.printf("Casa:    %.2f kW", homePowerKw);
} else {
display.print("Casa:    --.-- kW");
}
} else {
display.print("Casa:    N/D (DPM OFF)");
}
float activeEwbKw = isCharging ? ((wallboxPowerKw > 0.1) ? wallboxPowerKw : (FALLBACK_POWER_KW * AC_LOSS_FACTOR)) : 0.0;
display.setCursor(0, 22);
if (isCharging) {
display.printf("EWB:     %.2f kW", activeEwbKw);
} else {
display.print("EWB:     0.00 kW (Standby)");
}
display.setCursor(0, 32);
if (isDpmOn && hasHomePowerData) {
display.printf("Totale:  %.2f kW", homePowerKw + activeEwbKw);
} else {
display.printf("Totale:  %.2f kW (EWB)", activeEwbKw);
}
display.drawLine(0, 41, 127, 41, SSD1306_WHITE);
float plannedGross = (currentState == STATE_SELECT_SOC)
? (((TARGET_SOC - startSoC) / 100.0) * NET_BATTERY_KWH * AC_LOSS_FACTOR)
: targetGrossKwh;
if (plannedGross < 0) plannedGross = 0;
float remainingKwh = plannedGross - deliveredKwh;
if (remainingKwh < 0) remainingKwh = 0;
display.setCursor(0, 44);
display.printf("In:%.1fkWh | Rem:%.1fkWh", deliveredKwh, remainingKwh);
float activeKwForEta = (activeEwbKw > 0.5) ? activeEwbKw : (FALLBACK_POWER_KW * AC_LOSS_FACTOR);
unsigned long remMinutes = (unsigned long)((remainingKwh / activeKwForEta) * 60.0);
display.setCursor(0, 55);
if (currentState == STATE_COMPLETED || remainingKwh <= 0.05) {
display.print("Tempo rimasto: 0h 00m (OK)");
} else {
display.printf("Tempo rimasto: %luh %02lum", remMinutes / 60, remMinutes % 60);
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
Serial.println("Avvio Controller 500e + easyWallbox - " + String(FW_REVISION));
pinMode(ENC_CLK, INPUT_PULLUP);
pinMode(ENC_DT, INPUT_PULLUP);
pinMode(ENC_SW, INPUT_PULLUP);
lastCLKState = digitalRead(ENC_CLK);
Wire.begin(21, 22);
if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
Serial.println("Errore OLED SSD1306!");
}
display.clearDisplay();
display.display();
NimBLEDevice::init("ESP32_500e_Controller");
NimBLEDevice::setPower(ESP_PWR_LVL_P9);
lastUserActivityMs = millis();
updateDisplay();
}
void loop() {
unsigned long now = millis();
// 0. Stand-by OLED dopo 20 minuti di inattività
if (oledIsOn && (now - lastUserActivityMs > OLED_TIMEOUT_MS)) {
display.ssd1306_command(SSD1306_DISPLAYOFF);
oledIsOn = false;
}
// 0a. Rotazione automatica tra Pagina 1 (30s) e Pagina 2 (10s)
if (oledIsOn) {
unsigned long idleTime = now - lastUserActivityMs;
if (idleTime >= PAGE1_DURATION_MS) {
unsigned long cyclePos = (idleTime - PAGE1_DURATION_MS) % (PAGE2_DURATION_MS + PAGE1_DURATION_MS);
int desiredPage = (cyclePos < PAGE2_DURATION_MS) ? 2 : 1;
if (desiredPage != activeDisplayPage) {
activeDisplayPage = desiredPage;
screenNeedsUpdate = true;
}
}
}
// 0b. Avanzamento righe scorrevoli sulla Pagina 1
if (oledIsOn && activeDisplayPage == 1 && (now - lastTickerMillis >= 80)) {
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
// 1. Rotazione Encoder EC11
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
// 2. Pulsante Encoder EC11 (Click Breve vs Pressione Lunga 2.0s)
bool btnReading = (digitalRead(ENC_SW) == LOW);
if (btnReading && !btnIsDown) {
if (now - btnDownTime > 80) {
btnIsDown = true;
longPressHandled = false;
btnDownTime = now;
wakeUpOledAndResetPage();
}
}
else if (btnReading && btnIsDown) {
if (!longPressHandled && (now - btnDownTime >= LONG_PRESS_MS)) {
longPressHandled = true;
toggleDpmOff15Min();
}
}
else if (!btnReading && btnIsDown) {
btnIsDown = false;
if (!longPressHandled && (now - btnDownTime >= 80)) {
if (currentState == STATE_SELECT_SOC) {
float netKwh = ((TARGET_SOC - startSoC) / 100.0) * NET_BATTERY_KWH;
targetGrossKwh = netKwh * AC_LOSS_FACTOR;
fallbackDurationSec = (unsigned long)((netKwh / FALLBACK_POWER_KW) * 3600.0);
elapsedChargingSec = 0;
deliveredKwh = 0.0;
sessionStartKwh = -1.0;
hasLiveEnergyData = false;
lastTickMillis = now;
currentState = STATE_WAITING_START;
screenNeedsUpdate = true;
}
else if (currentState == STATE_COMPLETED) {
currentState = STATE_SELECT_SOC;
screenNeedsUpdate = true;
}
else if (currentState == STATE_WAITING_START || currentState == STATE_CHARGING) {
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
// 3. Controllo scadenza 15 minuti DPM OFF
if (dpmTemporarilyOff && getDpmRemainingSec() == 0) {
Serial.println("15 minuti trascorsi: invio $CMD,DPM,1 (ripristino DPM ON)...");
dpmTemporarilyOff = false;
sendBLECommand("$CMD,DPM,1");
wakeUpOledAndResetPage();
screenNeedsUpdate = true;
}
// 4. Connessione BLE discreta (attiva quando serve o in attesa)
if (pClient == nullptr || !pClient->isConnected()) {
if (bleConnected) {
bleConnected = false;
bleAuthenticated = false;
screenNeedsUpdate = true;
}
if (now - lastReconnectAttempt > 7000) {
lastReconnectAttempt = now;
connectToWallbox();
screenNeedsUpdate = true;
}
}
// 5. Timer Clessidra e Arresto all'80%
if (now - lastTickMillis >= 1000) {
lastTickMillis += 1000;
if (dpmTemporarilyOff) {
hourglassAnimFrame = !hourglassAnimFrame;
if (oledIsOn) {
screenNeedsUpdate = true;
}
}
if (currentState == STATE_CHARGING) {
elapsedChargingSec++;
if (!hasLiveEnergyData) {
deliveredKwh = ((float)elapsedChargingSec / (float)fallbackDurationSec) * targetGrossKwh;
}
if (elapsedChargingSec % 5 == 0 && oledIsOn) {
screenNeedsUpdate = true;
}
bool kwhReached = (hasLiveEnergyData && deliveredKwh >= targetGrossKwh);
bool timeReached = (!hasLiveEnergyData && elapsedChargingSec >= fallbackDurationSec);
if (kwhReached || timeReached) {
Serial.println("Target 80% raggiunto! Ripristino DPM e invio STOP.");
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
