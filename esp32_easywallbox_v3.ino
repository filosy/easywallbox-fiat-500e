/*
 * CONTROLLER FIAT 500e + EASYWALLBOX — REV 3.3 WEB
 * - BLE pairing and application authentication
 * - Local Wi-Fi web page without HTTP authentication
 * - SoC selection with graphical confirmation that ESP32 armed the session
 * - AD telemetry and estimated charging/household power
 * - Early interruption detection after two consecutive AD frames with zero car current
 * - Native STOP; success is reported only after two AD frames confirm zero car current
 * - DPM switch via EEPROM index 178 (this is not the DPM current limit)
 *
 * Safety: run only on a trusted local network. Monitor charging during testing.
 * Current/energy telemetry and SoC target are estimates; this is not a safety device.
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <NimBLEDevice.h>
#include <WiFi.h>
#include <WebServer.h>

// Replace locally before upload. Wi-Fi must support 2.4 GHz.
const char* WIFI_SSID = "NOME_WIFI";
const char* WIFI_PASS = "PASSWORD_WIFI";

#define FW_REVISION "REV 3.3 WEB"
#define EWB_MAC_ADDR "60:a4:23:7c:af:b1"
#define EWB_BLE_PIN "9093"
#define EWB_PAIRING_CODE 1234UL

static NimBLEUUID serviceUUID("331a36f5-2459-45ea-9d95-6142f0c4b307");
static NimBLEUUID charWriteUUID("a9da6040-0823-4995-94ec-9ce41ca28833");
static NimBLEUUID charNotifyUUID("a73e9a10-628f-4494-a099-12efaf72258f");
static NimBLEUUID charStatusUUID("75a9f022-af03-4e41-b4bc-9de90a47d50b");

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C
#define ENC_CLK 32
#define ENC_DT 33
#define ENC_SW 25

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
WebServer server(80);

const float NET_BATTERY_KWH = 37.3f;
const int TARGET_SOC = 80;
const float AC_LOSS_FACTOR = 1.12f;
const float FALLBACK_POWER_KW = 2.05f;
const float NOMINAL_VOLTAGE_V = 230.0f;
const float DPM_NOMINAL_VOLTAGE_V = 230.0f;

const unsigned long DPM_OFF_DURATION_SEC = 15UL * 60UL;
const unsigned long OLED_TIMEOUT_MS = 20UL * 60UL * 1000UL;
const unsigned long LONG_PRESS_MS = 2000UL;
const unsigned long AUTH_DELAY_MS = 8000UL;
const unsigned long AD_POLL_INTERVAL_MS = 10000UL;
const unsigned long STOP_CONFIRM_TIMEOUT_MS = 30000UL;
const unsigned long DISPLAY_PAGE1_MS = 30000UL;
const unsigned long DISPLAY_PAGE2_MS = 10000UL;
const uint8_t ZERO_CURRENT_CONFIRM_FRAMES = 2;

enum SystemState {
  STATE_SELECT_SOC,
  STATE_WAITING_START,
  STATE_CHARGING,
  STATE_INTERRUPTED,
  STATE_STOPPING,
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
bool chargeWasObserved = false;
bool targetStopRequested = false;
bool manualStopRequested = false;
bool nativeStopPending = false;
bool nativeStopAck = false;
bool lastAdFrameValid = false;
uint8_t zeroCurrentFrames = 0;

unsigned long fallbackDurationSec = 0;
unsigned long elapsedChargingSec = 0;
unsigned long lastTickMillis = 0;
unsigned long lastAdPollMillis = 0;
unsigned long lastReconnectAttempt = 0;
unsigned long lastWifiAttempt = 0;
unsigned long nativeStopRequestedAt = 0;
unsigned long lastUserActivityMs = 0;
unsigned long lastPageSwitchMs = 0;

bool dpmTemporarilyOff = false;
unsigned long dpmOffStartMillis = 0;
int activeDisplayPage = 1;
bool screenNeedsUpdate = true;
bool oledIsOn = true;

NimBLEClient* pClient = nullptr;
NimBLERemoteCharacteristic* pCharWrite = nullptr;
NimBLERemoteCharacteristic* pCharNotify = nullptr;
NimBLERemoteCharacteristic* pCharStatus = nullptr;
bool bleConnected = false;
bool bleAuthenticated = false;

String bleRxBuffer;
String lastCommand = "Nessuno";
String lastReply = "In attesa";
String lastTxResult = "N/D";
String sessionMessage = "Seleziona il SoC iniziale";

class WallboxClientCallbacks : public NimBLEClientCallbacks {
public:
  void onPassKeyEntry(NimBLEConnInfo& connInfo) override {
    Serial.println("[BLE] Pairing passkey richiesta; invio 01234.");
    NimBLEDevice::injectPassKey(connInfo, EWB_PAIRING_CODE);
  }

  void onAuthenticationComplete(NimBLEConnInfo& connInfo) override {
    Serial.printf("[BLE] Pairing completato; encrypted: %s\n",
                  connInfo.isEncrypted() ? "YES" : "NO");
  }
};

static WallboxClientCallbacks wallboxClientCallbacks;

const char* stateName() {
  switch (currentState) {
    case STATE_SELECT_SOC: return "Selezione SoC";
    case STATE_WAITING_START: return "In attesa dell'auto";
    case STATE_CHARGING: return "Ricarica in corso";
    case STATE_INTERRUPTED: return "Ricarica interrotta";
    case STATE_STOPPING: return "Verifica arresto";
    case STATE_COMPLETED: return "Ricarica fermata";
    default: return "Sconosciuto";
  }
}

String jsonEscape(const String& input) {
  String out;
  out.reserve(input.length() + 8);
  for (size_t i = 0; i < input.length(); i++) {
    char c = input[i];
    if (c == '"' || c == '\\') {
      out += '\\';
      out += c;
    } else if (c == '\n') {
      out += "\\n";
    } else if (c == '\r') {
      out += "\\r";
    } else if ((uint8_t)c >= 0x20) {
      out += c;
    }
  }
  return out;
}

void wakeUpOled() {
  lastUserActivityMs = millis();
  lastPageSwitchMs = lastUserActivityMs;
  activeDisplayPage = 1;
  if (!oledIsOn) {
    display.ssd1306_command(SSD1306_DISPLAYON);
    oledIsOn = true;
  }
  screenNeedsUpdate = true;
}

void sendBLECommand(const String& cmd) {
  lastCommand = cmd;
  if (!bleConnected || pCharWrite == nullptr) {
    lastTxResult = "Non inviato: BLE non connesso";
    Serial.printf("[EWB TX] skipped (not connected): %s\n", cmd.c_str());
    return;
  }

  String fullCmd = cmd + "\n";
  bool ok = pCharWrite->writeValue((uint8_t*)fullCmd.c_str(), fullCmd.length(), false);
  if (!ok) {
    ok = pCharWrite->writeValue((uint8_t*)fullCmd.c_str(), fullCmd.length(), true);
  }
  lastTxResult = ok ? "Scrittura BLE OK; esecuzione non confermata" : "Scrittura BLE fallita";
  Serial.printf("[EWB TX] %s -> %s\n", cmd.c_str(), ok ? "OK" : "ERR");
}

unsigned long getDpmRemainingSec() {
  if (!dpmTemporarilyOff) return 0;
  unsigned long elapsed = (millis() - dpmOffStartMillis) / 1000UL;
  if (elapsed >= DPM_OFF_DURATION_SEC) return 0;
  return DPM_OFF_DURATION_SEC - elapsed;
}

void setDpmEnabled(bool enabled) {
  if (!bleConnected || !bleAuthenticated) {
    lastReply = "DPM non modificato: BLE non autenticato";
    return;
  }
  // EEPROM index 178 toggles DPM enable; it does not set the DPM current limit.
  sendBLECommand(enabled ? "$EEP,WRITE,IDX,178,1" : "$EEP,WRITE,IDX,178,0");
}

void toggleDpmOff15Min() {
  if (!dpmTemporarilyOff) {
    dpmTemporarilyOff = true;
    dpmOffStartMillis = millis();
    setDpmEnabled(false);
    lastReply = "Richiesto DPM OFF temporaneo; protezione DPM disabilitata";
  } else {
    dpmTemporarilyOff = false;
    setDpmEnabled(true);
    lastReply = "Richiesto ripristino DPM ON";
  }
  screenNeedsUpdate = true;
}

String getCsvField(const String& data, int targetIndex) {
  int fieldIdx = 0;
  int startPos = 0;
  int len = data.length();
  for (int i = 0; i <= len; i++) {
    if (i == len || data.charAt(i) == ',') {
      if (fieldIdx == targetIndex) return data.substring(startPos, i);
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

void requestStop(bool targetReached) {
  if (!bleConnected || !bleAuthenticated) {
    lastReply = "STOP non inviato: BLE non autenticato";
    sessionMessage = lastReply;
    lastTxResult = "Nessuna scrittura";
    Serial.println("[CHARGE] STOP non inviato: BLE non autenticato.");
    screenNeedsUpdate = true;
    return;
  }

  if (nativeStopPending) return;

  targetStopRequested = targetReached;
  manualStopRequested = !targetReached;
  nativeStopPending = true;
  nativeStopAck = false;
  zeroCurrentFrames = 0;
  nativeStopRequestedAt = millis();
  currentState = STATE_STOPPING;
  sessionMessage = targetReached
    ? "Target energetico stimato raggiunto; verifico corrente auto a zero"
    : "STOP richiesto; verifico corrente auto a zero";

  sendBLECommand("$CMD,CHARGE,STOP,0");
  lastReply = sessionMessage;
  screenNeedsUpdate = true;
  Serial.println("[CHARGE] STOP inviato; servono due frame AD consecutivi a corrente zero.");
}

void markChargingInterrupted(int stateVal) {
  currentState = STATE_INTERRUPTED;
  if (stateVal == 3) {
    sessionMessage = "AUTO SCOLLEGATA prima del target";
  } else {
    sessionMessage = "RICARICA INTERROTTA prima del target";
  }
  lastReply = sessionMessage + "; auto a 0 A confermata in due letture";
  Serial.printf("[CHARGE] Interruzione rilevata: stato=%d, corrente=%.1f A, energia=%.1f/%.1f kWh\n",
                stateVal, liveCurrentA, deliveredKwh, targetGrossKwh);
  screenNeedsUpdate = true;
}

void confirmStopFromTelemetry(int stateVal) {
  nativeStopPending = false;
  currentState = STATE_COMPLETED;
  if (targetStopRequested) {
    sessionMessage = "TARGET RAGGIUNTO: auto a 0 A confermata";
    lastReply = nativeStopAck
      ? "Target raggiunto; ACK STOP e corrente auto a zero confermati"
      : "Target raggiunto; corrente auto a zero confermata da due frame AD";
  } else {
    sessionMessage = "RICARICA FERMATA: auto a 0 A confermata";
    lastReply = nativeStopAck
      ? "ACK STOP e corrente auto a zero confermati"
      : "Corrente auto a zero confermata da due frame AD";
  }
  Serial.printf("[CHARGE] STOP CONFERMATO: stato=%d, corrente=%.1f A, frames=%u\n",
                stateVal, liveCurrentA, zeroCurrentFrames);
  screenNeedsUpdate = true;
}

void processWallboxLine(String resp) {
  resp.trim();
  if (resp.length() == 0) return;
  Serial.println("[EWB RX] " + resp);
  lastReply = resp;

  if (resp.indexOf("$CMD,CHARGE,STOP,OK") >= 0) {
    nativeStopAck = true;
    Serial.println("[CHARGE] ACK STOP ricevuto dalla wallbox; attendo telemetria.");
  }

  if (resp.indexOf("$BLE,AUTH,OK") >= 0) {
    bleAuthenticated = true;
    lastAdPollMillis = millis() - AD_POLL_INTERVAL_MS;
    Serial.println("[BLE] Autenticazione applicativa confermata.");
    screenNeedsUpdate = true;
  }

  if (resp.indexOf("$BLE,AUTH,FAIL") >= 0 || resp.indexOf("$ERR,") >= 0) {
    bleAuthenticated = false;
    Serial.println("[BLE] La wallbox ha segnalato un errore.");
    screenNeedsUpdate = true;
  }

  if (!resp.startsWith("$DATA,READ,AD,")) return;

  // Absolute CSV indexes: 7 state, 10 current in dA, 11 power W,
  // 14 DPM current in dA, 22 session energy in 0.1 kWh.
  String stateField = getCsvField(resp, 7);
  String currentField = getCsvField(resp, 10);
  String powerField = getCsvField(resp, 11);
  String dpmField = getCsvField(resp, 14);
  String energyField = getCsvField(resp, 22);

  if (!isUnsignedInteger(stateField) || !isUnsignedInteger(currentField)) {
    lastAdFrameValid = false;
    Serial.println("[AD] Frame incompleto/non valido; non aggiorno stato carica.");
    return;
  }

  const int stateVal = stateField.toInt();
  liveCurrentA = currentField.toFloat() / 10.0f;
  lastAdFrameValid = true;

  if (isUnsignedInteger(powerField)) {
    wallboxPowerKw = powerField.toFloat() / 1000.0f;
  } else {
    wallboxPowerKw = liveCurrentA * NOMINAL_VOLTAGE_V / 1000.0f;
  }

  float sessionKwh = -1.0f;
  if (isUnsignedInteger(energyField)) sessionKwh = energyField.toFloat() / 10.0f;

  if (stateVal == 8) {
    chargeWasObserved = true;
    zeroCurrentFrames = 0;

    if (currentState == STATE_WAITING_START || currentState == STATE_INTERRUPTED) {
      currentState = STATE_CHARGING;
      sessionStartKwh = sessionKwh;
      deliveredKwh = 0.0f;
      hasLiveEnergyData = sessionKwh >= 0.0f;
      elapsedChargingSec = 0;
      lastTickMillis = millis();
      sessionMessage = "RICARICA IN CORSO";
      Serial.println("[CHARGE] Stato di ricarica rilevato (AD state 8).");
    } else if (currentState == STATE_CHARGING && sessionKwh >= 0.0f && sessionStartKwh >= 0.0f) {
      deliveredKwh = sessionKwh - sessionStartKwh;
      if (deliveredKwh < 0.0f) deliveredKwh = sessionKwh;
      hasLiveEnergyData = true;
    }
  } else {
    if (currentState == STATE_CHARGING && liveCurrentA < 0.5f) wallboxPowerKw = 0.0f;

    // Confirm an end/interruption only from valid AD frames; ignore transient single frames.
    if (chargeWasObserved && liveCurrentA < 0.5f &&
        currentState != STATE_STOPPING && currentState != STATE_COMPLETED) {
      if (zeroCurrentFrames < ZERO_CURRENT_CONFIRM_FRAMES) zeroCurrentFrames++;
      if (zeroCurrentFrames >= ZERO_CURRENT_CONFIRM_FRAMES) {
        markChargingInterrupted(stateVal);
      }
    } else if (liveCurrentA >= 0.5f) {
      zeroCurrentFrames = 0;
    }

    if (nativeStopPending && liveCurrentA < 0.5f) {
      if (zeroCurrentFrames < ZERO_CURRENT_CONFIRM_FRAMES) zeroCurrentFrames++;
      if (zeroCurrentFrames >= ZERO_CURRENT_CONFIRM_FRAMES) {
        confirmStopFromTelemetry(stateVal);
      }
    } else if (nativeStopPending && liveCurrentA >= 0.5f) {
      zeroCurrentFrames = 0;
    }
  }

  if (isUnsignedInteger(dpmField)) {
    float dpmCurrentDa = dpmField.toFloat();
    dpmTotalPowerKw = dpmCurrentDa * DPM_NOMINAL_VOLTAGE_V / 10000.0f;
    float carPowerKw = (stateVal == 8) ? wallboxPowerKw : 0.0f;
    homePowerKw = dpmTotalPowerKw - carPowerKw;
    if (homePowerKw < 0.0f) homePowerKw = 0.0f;
    hasHomePowerData = true;
    Serial.printf("[DPM AD] dpm=%s dA, total=%.2f kW, car=%.2f kW, house-estimate=%.2f kW\n",
                  dpmField.c_str(), dpmTotalPowerKw, wallboxPowerKw, homePowerKw);
  }

  if (stateVal == 8 && currentState == STATE_CHARGING &&
      hasLiveEnergyData && deliveredKwh >= targetGrossKwh) {
    Serial.println("[CHARGE] Target energetico stimato raggiunto.");
    requestStop(true);
  }

  screenNeedsUpdate = true;
}

void notifyCallback(NimBLERemoteCharacteristic* characteristic, uint8_t* data,
                    size_t length, bool isNotify) {
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
      Serial.println("[BLE] Buffer notifiche pieno; azzerato.");
      bleRxBuffer = "";
    }
  }
}

bool connectToWallbox() {
  Serial.println("[BLE] Scansione EasyWallbox...");
  NimBLEScan* scan = NimBLEDevice::getScan();
  scan->setActiveScan(true);
  scan->setInterval(100);
  scan->setWindow(99);
  NimBLEScanResults results = scan->getResults(2000, false);
  const NimBLEAdvertisedDevice* target = nullptr;

  for (int i = 0; i < results.getCount(); i++) {
    const NimBLEAdvertisedDevice* dev = results.getDevice(i);
    String address = String(dev->getAddress().toString().c_str());
    address.toLowerCase();
    if (address == EWB_MAC_ADDR) {
      target = dev;
      break;
    }
  }

  if (target == nullptr) {
    scan->clearResults();
    Serial.println("[BLE] Wallbox non trovata.");
    return false;
  }

  if (pClient == nullptr) pClient = NimBLEDevice::createClient();
  pClient->setClientCallbacks(&wallboxClientCallbacks, false);
  if (!pClient->connect(target)) {
    scan->clearResults();
    bleConnected = false;
    bleAuthenticated = false;
    Serial.println("[BLE] Connessione fallita.");
    return false;
  }
  scan->clearResults();

  if (!pClient->secureConnection()) {
    Serial.printf("[BLE] secureConnection fallita, errore=%d\n", pClient->getLastError());
    pClient->disconnect();
    bleConnected = false;
    bleAuthenticated = false;
    return false;
  }

  NimBLERemoteService* service = pClient->getService(serviceUUID);
  if (service == nullptr) {
    Serial.println("[BLE] Servizio EasyWallbox non trovato.");
    pClient->disconnect();
    bleConnected = false;
    return false;
  }

  pCharWrite = service->getCharacteristic(charWriteUUID);
  pCharNotify = service->getCharacteristic(charNotifyUUID);
  pCharStatus = service->getCharacteristic(charStatusUUID);
  if (pCharWrite == nullptr) {
    Serial.println("[BLE] Caratteristica scrittura non trovata.");
    pClient->disconnect();
    bleConnected = false;
    return false;
  }

  if (pCharNotify != nullptr && pCharNotify->canNotify()) {
    Serial.printf("[BLE] Sottoscrizione TX: %s\n",
      pCharNotify->subscribe(true, notifyCallback, true) ? "OK" : "ERRORE");
  }
  if (pCharStatus != nullptr && pCharStatus->canNotify()) {
    Serial.printf("[BLE] Sottoscrizione ST: %s\n",
      pCharStatus->subscribe(true, notifyCallback, true) ? "OK" : "ERRORE");
  }

  bleConnected = true;
  bleAuthenticated = false;
  bleRxBuffer = "";

  Serial.println("[BLE] Attendo 8 secondi prima dell'AUTH...");
  unsigned long waitStart = millis();
  while (millis() - waitStart < AUTH_DELAY_MS) {
    server.handleClient();
    delay(5);
  }

  if (!pClient->isConnected()) {
    bleConnected = false;
    Serial.println("[BLE] Connessione persa prima dell'AUTH.");
    return false;
  }
  sendBLECommand(String("$BLE,AUTH,") + EWB_BLE_PIN);
  return true;
}

void startSession(int soc) {
  if (soc < 5 || soc > 79 || currentState != STATE_SELECT_SOC) return;
  startSoC = soc;
  float netKwh = ((TARGET_SOC - startSoC) / 100.0f) * NET_BATTERY_KWH;
  targetGrossKwh = netKwh * AC_LOSS_FACTOR;
  fallbackDurationSec = (unsigned long)((netKwh / FALLBACK_POWER_KW) * 3600.0f);

  elapsedChargingSec = 0;
  deliveredKwh = 0.0f;
  sessionStartKwh = -1.0f;
  hasLiveEnergyData = false;
  chargeWasObserved = false;
  targetStopRequested = false;
  manualStopRequested = false;
  nativeStopPending = false;
  nativeStopAck = false;
  zeroCurrentFrames = 0;
  currentState = STATE_WAITING_START;
  lastTickMillis = millis();
  sessionMessage = "SoC " + String(startSoC) + "% ricevuto; sessione armata, attendo l'auto";
  lastReply = sessionMessage;
  Serial.printf("[SESSION] SoC iniziale accettato: %d%%; target energetico stimato %.1f kWh\n",
                startSoC, targetGrossKwh);
  screenNeedsUpdate = true;
}

void handleRoot() {
  static const char page[] PROGMEM = R"HTML(
<!doctype html><html lang="it"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>EasyWallbox</title>
<style>
:root{color-scheme:dark}body{font-family:system-ui,Arial;background:#111827;color:#e5e7eb;margin:0;padding:16px;max-width:760px;margin:auto}.card{background:#1f2937;border:1px solid #374151;border-radius:12px;padding:16px;margin:12px 0}.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(135px,1fr));gap:10px}.stat{background:#111827;border-radius:9px;padding:12px}.label{font-size:13px;color:#9ca3af}.value{font-size:20px;font-weight:700;margin-top:5px;overflow-wrap:anywhere}button{border:0;border-radius:8px;padding:12px 14px;color:#fff;font-size:15px;background:#2563eb;margin:4px}button:disabled{opacity:.55}.stop{background:#dc2626}.warn{background:#b45309}.secondary{background:#4b5563}input[type=range]{width:100%}.status{padding:12px;border-radius:8px;background:#374151;font-weight:700;margin:10px 0}.pending{background:#92400e}.ok{background:#166534}.alert{background:#991b1b}.note{font-size:13px;color:#fbbf24}
</style></head><body>
<h2>EasyWallbox · monitor locale</h2>
<div class="card"><div class="label">SoC iniziale: <b id="socLabel">45%</b> · target firmware: 80%</div><input id="soc" type="range" min="5" max="79" value="45"><div id="socStatus" class="status">Seleziona il SoC iniziale</div><button id="start">Imposta SoC / arma sessione</button><button id="stop" class="stop">Richiedi STOP</button><button id="dpmOn" class="secondary">DPM ON</button><button id="dpmOff" class="warn">DPM OFF 15 min</button><p class="note">Nessuna password web: solo rete locale fidata, non esporre a Internet. Il target SoC è stimato dall'energia; verifica sempre la ricarica.</p></div>
<div class="card grid"><div class="stat"><div class="label">Stato</div><div class="value" id="state">--</div></div><div class="stat"><div class="label">BLE</div><div class="value" id="ble">--</div></div><div class="stat"><div class="label">Wi-Fi / IP</div><div class="value" id="wifi">--</div></div><div class="stat"><div class="label">DPM</div><div class="value" id="dpm">--</div></div></div>
<div class="card"><h3>Ricarica</h3><div id="sessionMessage" class="status">--</div><div class="grid"><div class="stat"><div class="label">Corrente auto</div><div class="value" id="amps">-- A</div></div><div class="stat"><div class="label">Potenza auto</div><div class="value" id="car">-- kW</div></div><div class="stat"><div class="label">Energia erogata</div><div class="value" id="energy">-- kWh</div></div><div class="stat"><div class="label">Target energia stimato</div><div class="value" id="target">-- kWh</div></div><div class="stat"><div class="label">Energia rimanente</div><div class="value" id="remaining">-- kWh</div></div><div class="stat"><div class="label">Casa stimata</div><div class="value" id="home">-- kW</div></div><div class="stat"><div class="label">Totale DPM stimato</div><div class="value" id="total">-- kW</div></div></div></div>
<div class="card"><div class="label">Ultimo comando / esito scrittura</div><div id="command">--</div><div class="label" style="margin-top:10px">Ultima risposta wallbox</div><div id="reply">--</div></div>
<script>
const slider=document.getElementById('soc'),socStatus=document.getElementById('socStatus'),startButton=document.getElementById('start');let socDirty=false;
slider.addEventListener('input',()=>{socDirty=true;document.getElementById('socLabel').textContent=slider.value+'%';socStatus.className='status';socStatus.textContent='SoC selezionato: '+slider.value+'%. Premi “Imposta SoC” per confermare.'});
async function control(action){const selected=slider.value,body=new URLSearchParams({action,soc:selected});if(action==='start'){socStatus.className='status pending';socStatus.textContent='Invio SoC '+selected+'%…';startButton.disabled=true}try{const r=await fetch('/api/control',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body,cache:'no-store'});let d={};try{d=await r.json()}catch(e){}if(!r.ok||!d.ok){if(action==='start'){socStatus.className='status alert';socStatus.textContent=d.error||'SoC non accettato'}else alert(d.error||'Comando non accettato');return}if(action==='start'){socDirty=false;slider.value=d.soc;document.getElementById('socLabel').textContent=d.soc+'%';socStatus.className='status ok';socStatus.textContent='✓ SoC '+d.soc+'% ricevuto: sessione armata, in attesa dell’auto.'}await refresh()}catch(e){if(action==='start'){socStatus.className='status alert';socStatus.textContent='Errore rete: SoC non confermato'}else alert('Errore rete: '+e)}finally{startButton.disabled=false}}
document.getElementById('start').onclick=()=>control('start');document.getElementById('stop').onclick=()=>control('stop');document.getElementById('dpmOn').onclick=()=>control('dpmOn');document.getElementById('dpmOff').onclick=()=>control('dpmOff');
async function refresh(){try{const r=await fetch('/api/status',{cache:'no-store'}),d=await r.json();for(const [id,key] of [['state','state'],['ble','ble'],['wifi','wifi'],['dpm','dpm'],['amps','currentA'],['car','carKw'],['energy','delivered'],['target','target'],['remaining','remaining'],['command','command'],['reply','reply'],['sessionMessage','sessionMessage']])document.getElementById(id).textContent=d[key]+(['amps'].includes(id)?' A':['car','home','total'].includes(id)?' kW':(['energy','target','remaining'].includes(id)?' kWh':''));document.getElementById('home').textContent=d.homeValid?d.homeKw+' kW':'N/D';document.getElementById('total').textContent=d.homeValid?d.totalKw+' kW':'N/D';const m=document.getElementById('sessionMessage');m.className='status'+(d.stateCode==='interrupted'?' alert':d.stateCode==='completed'?' ok':'');if(!socDirty){slider.value=d.soc;document.getElementById('socLabel').textContent=d.soc+'%'}}catch(e){}}
setInterval(refresh,3000);refresh();
</script></body></html>
)HTML";
  server.send_P(200, "text/html; charset=utf-8", page);
}

void handleStatus() {
  float remaining = targetGrossKwh - deliveredKwh;
  if (remaining < 0.0f) remaining = 0.0f;
  String code = "idle";
  if (currentState == STATE_WAITING_START) code = "waiting";
  else if (currentState == STATE_CHARGING) code = "charging";
  else if (currentState == STATE_INTERRUPTED) code = "interrupted";
  else if (currentState == STATE_STOPPING) code = "stopping";
  else if (currentState == STATE_COMPLETED) code = "completed";

  String json = "{";
  json += "\"state\":\"" + String(stateName()) + "\",";
  json += "\"stateCode\":\"" + code + "\",";
  json += "\"ble\":\"" + String(bleConnected ? (bleAuthenticated ? "Connesso" : "Connesso, non autenticato") : "Disconnesso") + "\",";
  json += "\"wifi\":\"" + String(WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString() : "Disconnesso") + "\",";
  json += "\"soc\":" + String(startSoC) + ",";
  json += "\"dpm\":\"" + String(dpmTemporarilyOff ? "OFF temporaneo" : "ON") + "\",";
  json += "\"currentA\":" + String(liveCurrentA, 1) + ",";
  json += "\"carKw\":" + String(wallboxPowerKw, 2) + ",";
  json += "\"homeKw\":" + String(homePowerKw, 2) + ",";
  json += "\"totalKw\":" + String(dpmTotalPowerKw, 2) + ",";
  json += "\"homeValid\":" + String(hasHomePowerData ? "true" : "false") + ",";
  json += "\"delivered\":" + String(deliveredKwh, 1) + ",";
  json += "\"target\":" + String(targetGrossKwh, 1) + ",";
  json += "\"remaining\":" + String(remaining, 1) + ",";
  json += "\"stopPending\":" + String(nativeStopPending ? "true" : "false") + ",";
  json += "\"stopAck\":" + String(nativeStopAck ? "true" : "false") + ",";
  json += "\"sessionMessage\":\"" + jsonEscape(sessionMessage) + "\",";
  json += "\"command\":\"" + jsonEscape(lastCommand) + "\",";
  json += "\"tx\":\"" + jsonEscape(lastTxResult) + "\",";
  json += "\"reply\":\"" + jsonEscape(lastReply) + "\"}";
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", json);
}

void handleControl() {
  // Deliberately no HTTP authentication: keep this device on a trusted LAN only.
  String action = server.arg("action");

  if (action == "start") {
    if (currentState != STATE_SELECT_SOC) {
      server.send(409, "application/json", "{\"ok\":false,\"error\":\"Sessione già avviata\"}");
      return;
    }
    int soc = server.arg("soc").toInt();
    if (soc < 5 || soc > 79) {
      server.send(400, "application/json", "{\"ok\":false,\"error\":\"SoC fuori intervallo\"}");
      return;
    }
    startSession(soc);
    String response = "{\"ok\":true,\"soc\":" + String(startSoC) + ",\"message\":\"Sessione armata\"}";
    server.send(200, "application/json", response);
    return;
  }

  if (!bleConnected || !bleAuthenticated) {
    server.send(503, "application/json", "{\"ok\":false,\"error\":\"Wallbox BLE non autenticata\"}");
    return;
  }

  if (action == "stop") {
    requestStop(false);
  } else if (action == "dpmOff") {
    dpmTemporarilyOff = true;
    dpmOffStartMillis = millis();
    setDpmEnabled(false);
    lastReply = "Richiesto DPM OFF temporaneo (DPM disabilitato)";
  } else if (action == "dpmOn") {
    dpmTemporarilyOff = false;
    setDpmEnabled(true);
    lastReply = "Richiesto DPM ON";
  } else {
    server.send(400, "application/json", "{\"ok\":false,\"error\":\"Azione sconosciuta\"}");
    return;
  }

  screenNeedsUpdate = true;
  server.send(200, "application/json", "{\"ok\":true}");
}

void drawPage1() {
  display.setCursor(0, 0);
  display.print(bleConnected ? "BLE:OK " : "BLE:-- ");
  display.print(dpmTemporarilyOff ? "DPM:OFF" : "DPM:ON");
  display.drawLine(0, 9, 127, 9, SSD1306_WHITE);

  display.setTextSize(1);
  if (currentState == STATE_SELECT_SOC) {
    display.setCursor(0, 14);
    display.print("SoC iniziale:");
    display.setTextSize(3);
    display.setCursor(8, 28);
    display.printf("%d%%", startSoC);
  } else if (currentState == STATE_WAITING_START) {
    display.setCursor(0, 16);
    display.print("ATTENDO AUTO");
    display.setCursor(0, 30);
    display.printf("Target %.1f kWh", targetGrossKwh);
  } else if (currentState == STATE_CHARGING) {
    display.setCursor(0, 13);
    display.printf("RICARICA %.1f A", liveCurrentA);
    display.setCursor(0, 27);
    display.printf("%.1f/%.1f kWh", deliveredKwh, targetGrossKwh);
    display.setCursor(0, 40);
    display.print("Target SoC stimato 80%");
  } else if (currentState == STATE_INTERRUPTED) {
    display.setCursor(0, 14);
    display.print("RICARICA FERMA");
    display.setCursor(0, 28);
    display.print("Prima del target");
    display.setCursor(0, 42);
    display.print(liveCurrentA < 0.5f ? "Auto: 0 A confermato" : "Controllare wallbox");
  } else if (currentState == STATE_STOPPING) {
    display.setCursor(0, 14);
    display.print("STOP RICHIESTO");
    display.setCursor(0, 29);
    display.print("Verifico auto = 0 A");
    display.setCursor(0, 43);
    display.printf("Conferme: %u/%u", zeroCurrentFrames, ZERO_CURRENT_CONFIRM_FRAMES);
  } else {
    display.setCursor(0, 13);
    display.print(targetStopRequested ? "TARGET RAGGIUNTO" : "RICARICA FERMATA");
    display.setCursor(0, 28);
    display.print("Auto: 0 A confermato");
    display.setCursor(0, 43);
    display.print("Click per nuova sessione");
  }
}

void drawPage2() {
  display.setCursor(0, 0);
  display.print("TELEMETRIA DPM");
  display.drawLine(0, 9, 127, 9, SSD1306_WHITE);
  display.setCursor(0, 13);
  if (hasHomePowerData) display.printf("Casa est.: %.2f kW", homePowerKw);
  else display.print("Casa est.: N/D");
  display.setCursor(0, 25);
  display.printf("Auto: %.2f kW %.1fA", wallboxPowerKw, liveCurrentA);
  display.setCursor(0, 37);
  if (hasHomePowerData) display.printf("DPM totale: %.2f kW", dpmTotalPowerKw);
  else display.print("DPM totale: N/D");
  display.setCursor(0, 51);
  display.printf("Energia %.1f/%.1f kWh", deliveredKwh, targetGrossKwh);
}

void updateDisplay() {
  if (!oledIsOn) return;
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setTextWrap(false);
  if (activeDisplayPage == 1) drawPage1();
  else drawPage2();
  display.display();
  screenNeedsUpdate = false;
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("Avvio controller Fiat 500e + EasyWallbox - " FW_REVISION);

  pinMode(ENC_CLK, INPUT_PULLUP);
  pinMode(ENC_DT, INPUT_PULLUP);
  pinMode(ENC_SW, INPUT_PULLUP);
  lastCLKState = digitalRead(ENC_CLK);

  Wire.begin(21, 22);
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println("[OLED] Errore inizializzazione.");
  }
  display.clearDisplay();
  display.display();

  NimBLEDevice::init("ESP32_500e_Controller");
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);
  NimBLEDevice::setSecurityAuth(true, true, true);
  NimBLEDevice::setSecurityIOCap(BLE_HS_IO_KEYBOARD_ONLY);

  lastUserActivityMs = millis();
  lastPageSwitchMs = lastUserActivityMs;
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  lastWifiAttempt = millis();

  unsigned long wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < 15000UL) delay(250);
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("[WiFi] IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("[WiFi] Non connesso; verrà ritentato.");
  }

  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/status", HTTP_GET, handleStatus);
  server.on("/api/control", HTTP_POST, handleControl);
  server.begin();
  Serial.println("[WEB] Server avviato sulla porta 80, senza autenticazione HTTP.");
  updateDisplay();
}

void loop() {
  unsigned long now = millis();
  server.handleClient();

  if (nativeStopPending && now - nativeStopRequestedAt > STOP_CONFIRM_TIMEOUT_MS) {
    nativeStopPending = false;
    currentState = STATE_INTERRUPTED;
    sessionMessage = "STOP non confermato: corrente auto non a zero; controlla wallbox";
    lastReply = sessionMessage;
    Serial.println("[CHARGE] STOP timeout: corrente zero non confermata.");
    screenNeedsUpdate = true;
  }

  if (WiFi.status() != WL_CONNECTED && now - lastWifiAttempt >= 10000UL) {
    lastWifiAttempt = now;
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASS);
  }

  if (oledIsOn && now - lastUserActivityMs > OLED_TIMEOUT_MS) {
    display.ssd1306_command(SSD1306_DISPLAYOFF);
    oledIsOn = false;
  }

  if (oledIsOn && now - lastPageSwitchMs >= DISPLAY_PAGE1_MS) {
    unsigned long phase = (now - lastPageSwitchMs - DISPLAY_PAGE1_MS) %
                          (DISPLAY_PAGE1_MS + DISPLAY_PAGE2_MS);
    int desiredPage = phase < DISPLAY_PAGE2_MS ? 2 : 1;
    if (desiredPage != activeDisplayPage) {
      activeDisplayPage = desiredPage;
      screenNeedsUpdate = true;
    }
  }

  int currentCLK = digitalRead(ENC_CLK);
  if (currentCLK != lastCLKState && currentCLK == LOW) {
    wakeUpOled();
    if (currentState == STATE_SELECT_SOC) {
      startSoC += (digitalRead(ENC_DT) != currentCLK) ? 1 : -1;
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
      wakeUpOled();
    }
  } else if (btnReading && btnIsDown) {
    if (!longPressHandled && now - btnDownTime >= LONG_PRESS_MS) {
      longPressHandled = true;
      if (bleConnected && bleAuthenticated) toggleDpmOff15Min();
    }
  } else if (!btnReading && btnIsDown) {
    btnIsDown = false;
    if (!longPressHandled && now - btnDownTime >= 80UL) {
      if (currentState == STATE_SELECT_SOC) {
        startSession(startSoC);
      } else if (currentState == STATE_COMPLETED || currentState == STATE_INTERRUPTED) {
        currentState = STATE_SELECT_SOC;
        nativeStopPending = false;
        targetStopRequested = false;
        manualStopRequested = false;
        chargeWasObserved = false;
        zeroCurrentFrames = 0;
        sessionMessage = "Seleziona il SoC iniziale";
        screenNeedsUpdate = true;
      } else if (currentState != STATE_STOPPING) {
        requestStop(false);
      }
    }
  }

  if (dpmTemporarilyOff && getDpmRemainingSec() == 0) {
    dpmTemporarilyOff = false;
    if (bleConnected && bleAuthenticated) setDpmEnabled(true);
    lastReply = "Timer DPM scaduto; ripristino DPM richiesto";
    screenNeedsUpdate = true;
  }

  if (pClient == nullptr || !pClient->isConnected()) {
    if (bleConnected) {
      bleConnected = false;
      bleAuthenticated = false;
      screenNeedsUpdate = true;
      Serial.println("[BLE] Connessione persa.");
    }
    if (now - lastReconnectAttempt > 7000UL) {
      lastReconnectAttempt = now;
      connectToWallbox();
    }
  }

  if (bleConnected && bleAuthenticated && now - lastAdPollMillis >= AD_POLL_INTERVAL_MS) {
    lastAdPollMillis = now;
    sendBLECommand("$DATA,READ,AD");
  }

  if (now - lastTickMillis >= 1000UL) {
    lastTickMillis += 1000UL;
    if (currentState == STATE_CHARGING) {
      elapsedChargingSec++;
      if (!hasLiveEnergyData && fallbackDurationSec > 0) {
        deliveredKwh = ((float)elapsedChargingSec / (float)fallbackDurationSec) * targetGrossKwh;
      }
      bool timeReached = !hasLiveEnergyData && fallbackDurationSec > 0 &&
                         elapsedChargingSec >= fallbackDurationSec;
      if (timeReached && !nativeStopPending) {
        Serial.println("[CHARGE] Stima temporale raggiunta; invio richiesta STOP.");
        requestStop(true);
      }
      if (oledIsOn) screenNeedsUpdate = true;
    }
  }

  if (screenNeedsUpdate) updateDisplay();
}
