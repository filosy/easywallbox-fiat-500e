/*
 * CONTROLLER FIAT 500e + EASYWALLBOX — REV 3.2 WEB
 * - BLE pairing and application authentication
 * - Wi-Fi web server without HTTP authentication
 * - AD telemetry and house/DPM power estimation
 * - Graphical confirmation of SoC after it is accepted by ESP32
 * - Native stop command: $CMD,CHARGE,STOP,0
 * - DPM enable/disable via EEPROM index 178
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <NimBLEDevice.h>
#include <WiFi.h>
#include <WebServer.h>

// Replace before uploading. Use a 2.4 GHz Wi-Fi network.
const char* WIFI_SSID = "NOME_WIFI";
const char* WIFI_PASS = "PASSWORD_WIFI";

#define FW_REVISION "REV 3.2 WEB"
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
const unsigned long STOP_CONFIRM_TIMEOUT_MS = 20000UL;

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
unsigned long lastAdPollMillis = 0;
unsigned long lastReconnectAttempt = 0;
unsigned long lastWifiAttempt = 0;

bool dpmTemporarilyOff = false;
unsigned long dpmOffStartMillis = 0;

bool nativeStopPending = false;
bool nativeStopAck = false;
unsigned long nativeStopRequestedAt = 0;

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

bool screenNeedsUpdate = true;
bool oledIsOn = true;
unsigned long lastUserActivityMs = 0;

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
    case STATE_COMPLETED: return "Stop richiesto";
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

  lastTxResult = ok
    ? "Scrittura BLE OK; esecuzione non confermata"
    : "Scrittura BLE fallita";
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
  sendBLECommand(enabled ? "$EEP,WRITE,IDX,178,1" : "$EEP,WRITE,IDX,178,0");
}

void toggleDpmOff15Min() {
  if (!dpmTemporarilyOff) {
    dpmTemporarilyOff = true;
    dpmOffStartMillis = millis();
    setDpmEnabled(false);
    lastReply = "Richiesto DPM OFF per 15 minuti";
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

void requestStop() {
  if (!bleConnected || !bleAuthenticated) {
    lastReply = "STOP non inviato: BLE non autenticato";
    lastTxResult = "Nessuna scrittura";
    Serial.println("[CHARGE] STOP non inviato: BLE non autenticato.");
    return;
  }

  if (dpmTemporarilyOff) {
    dpmTemporarilyOff = false;
    setDpmEnabled(true);
  }

  nativeStopPending = true;
  nativeStopAck = false;
  nativeStopRequestedAt = millis();

  sendBLECommand("$CMD,CHARGE,STOP,0");
  currentState = STATE_COMPLETED;
  lastReply = "STOP inviato; attendo conferma BLE e telemetria";
  screenNeedsUpdate = true;
}

void processWallboxLine(String resp) {
  resp.trim();
  if (resp.length() == 0) return;

  Serial.println("[EWB RX] " + resp);
  lastReply = resp;

  if (resp.indexOf("$CMD,CHARGE,STOP,OK") >= 0) {
    nativeStopAck = true;
    lastReply = "ACK STOP ricevuto; attendo conferma telemetria";
    Serial.println("[CHARGE] ACK STOP ricevuto dalla wallbox.");
  }

  if (resp.indexOf("$BLE,AUTH,OK") >= 0) {
    bleAuthenticated = true;
    lastAdPollMillis = millis() - AD_POLL_INTERVAL_MS;
    Serial.println("[BLE] Autenticazione applicativa confermata.");
  }

  if (resp.indexOf("$BLE,AUTH,FAIL") >= 0 || resp.indexOf("$ERR,") >= 0) {
    bleAuthenticated = false;
    Serial.println("[BLE] La wallbox ha segnalato un errore.");
  }

  if (!resp.startsWith("$DATA,READ,AD,")) return;

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
    wallboxPowerKw = carPowerField.toFloat() / 1000.0f;
  } else {
    wallboxPowerKw = liveCurrentA * NOMINAL_VOLTAGE_V / 1000.0f;
  }

  if (stateVal != 8 && currentState == STATE_CHARGING && liveCurrentA < 0.5f) {
    wallboxPowerKw = 0.0f;
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
    } else if (currentState == STATE_CHARGING && sessionKwh >= 0.0f && sessionStartKwh >= 0.0f) {
      deliveredKwh = sessionKwh - sessionStartKwh;
      if (deliveredKwh < 0.0f) deliveredKwh = sessionKwh;
      hasLiveEnergyData = true;
    }
  }

  if (nativeStopPending && stateVal != 8 && liveCurrentA < 0.5f) {
    nativeStopPending = false;
    lastReply = nativeStopAck
      ? "STOP confermato: ACK ricevuto e corrente a zero"
      : "STOP confermato dalla telemetria: corrente a zero";
    Serial.printf("[CHARGE] STOP CONFERMATO da AD: stato=%d, corrente=%.1f A\n",
                  stateVal, liveCurrentA);
  }

  if (isUnsignedInteger(dpmCurrentField)) {
    float dpmCurrentDa = dpmCurrentField.toFloat();
    dpmTotalPowerKw = dpmCurrentDa * DPM_NOMINAL_VOLTAGE_V / 10000.0f;

    float carPowerKw = (stateVal == 8) ? wallboxPowerKw : 0.0f;
    homePowerKw = dpmTotalPowerKw - carPowerKw;
    if (homePowerKw < 0.0f) homePowerKw = 0.0f;
    hasHomePowerData = true;

    Serial.printf("[DPM AD] dpm=%s dA, total=%.2f kW, car=%.2f kW, house-estimate=%.2f kW\n",
                  dpmCurrentField.c_str(), dpmTotalPowerKw, wallboxPowerKw, homePowerKw);
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

  NimBLEScan* pScan = NimBLEDevice::getScan();
  pScan->setActiveScan(true);
  pScan->setInterval(100);
  pScan->setWindow(99);

  NimBLEScanResults results = pScan->getResults(2000, false);
  const NimBLEAdvertisedDevice* targetDevice = nullptr;

  for (int i = 0; i < results.getCount(); i++) {
    const NimBLEAdvertisedDevice* dev = results.getDevice(i);
    String address = String(dev->getAddress().toString().c_str());
    address.toLowerCase();
    if (address == EWB_MAC_ADDR) {
      targetDevice = dev;
      break;
    }
  }

  if (targetDevice == nullptr) {
    pScan->clearResults();
    Serial.println("[BLE] Wallbox non trovata.");
    return false;
  }

  if (pClient == nullptr) pClient = NimBLEDevice::createClient();
  pClient->setClientCallbacks(&wallboxClientCallbacks, false);

  if (!pClient->connect(targetDevice)) {
    pScan->clearResults();
    bleConnected = false;
    bleAuthenticated = false;
    Serial.println("[BLE] Connessione fallita.");
    return false;
  }

  pScan->clearResults();
  Serial.println("[BLE] Connesso; pairing sicuro...");

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
    pCharNotify->subscribe(true, notifyCallback, true);
  }
  if (pCharStatus != nullptr && pCharStatus->canNotify()) {
    pCharStatus->subscribe(true, notifyCallback, true);
  }

  bleConnected = true;
  bleAuthenticated = false;
  bleRxBuffer = "";

  Serial.println("[BLE] Attendo 8 secondi prima dell'AUTH...");
  unsigned long authWaitStart = millis();
  while (millis() - authWaitStart < AUTH_DELAY_MS) {
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
  lastTickMillis = millis();
  nativeStopPending = false;
  nativeStopAck = false;
  currentState = STATE_WAITING_START;

  lastReply = "SoC ricevuto; sessione armata, in attesa dell'auto";
  Serial.printf("[SESSION] SoC iniziale accettato: %d%%\n", startSoC);
  screenNeedsUpdate = true;
}

void handleRoot() {
  static const char page[] PROGMEM = R"HTML(
<!doctype html>
<html lang="it"><head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>EasyWallbox</title>
<style>
:root{color-scheme:dark}body{font-family:system-ui,Arial;background:#111827;color:#e5e7eb;margin:0;padding:16px;max-width:760px;margin:auto}.card{background:#1f2937;border:1px solid #374151;border-radius:12px;padding:16px;margin:12px 0}.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(135px,1fr));gap:10px}.stat{background:#111827;border-radius:9px;padding:12px}.label{font-size:13px;color:#9ca3af}.value{font-size:21px;font-weight:700;margin-top:5px}button{border:0;border-radius:8px;padding:12px 14px;color:white;font-size:15px;background:#2563eb;margin:4px}button:disabled{opacity:.55}.stop{background:#dc2626}.warn{background:#b45309}.secondary{background:#4b5563}input[type=range]{width:100%}.soc-status{margin:10px 4px;padding:10px 12px;border-radius:8px;background:#374151;font-weight:600}.soc-status.pending{background:#92400e}.soc-status.ok{background:#166534}.soc-status.error{background:#991b1b}.note{font-size:13px;color:#fbbf24}
</style></head><body>
<h2>EasyWallbox · monitor locale</h2>
<div class="card">
<div class="label">SoC iniziale selezionato: <b id="socLabel">45%</b> · target: 80%</div>
<input id="soc" type="range" min="5" max="79" value="45">
<div id="socStatus" class="soc-status">Seleziona il SoC iniziale</div>
<button id="start">Imposta SoC / arma sessione</button>
<button id="stop" class="stop">Richiedi STOP</button>
<button id="dpmOn" class="secondary">DPM ON</button>
<button id="dpmOff" class="warn">DPM OFF 15 min</button>
<p class="note">La conferma SoC significa che l'ESP32 ha armato la sessione. Verifica sempre l'arresto reale sulla wallbox.</p>
</div>
<div class="card grid">
<div class="stat"><div class="label">Stato sessione</div><div class="value" id="state">--</div></div>
<div class="stat"><div class="label">BLE</div><div class="value" id="ble">--</div></div>
<div class="stat"><div class="label">Wi-Fi / IP</div><div class="value" id="wifi">--</div></div>
<div class="stat"><div class="label">DPM</div><div class="value" id="dpm">--</div></div>
</div>
<div class="card"><h3>Telemetria</h3><div class="grid">
<div class="stat"><div class="label">Corrente auto</div><div class="value" id="amps">-- A</div></div>
<div class="stat"><div class="label">Potenza auto</div><div class="value" id="car">-- kW</div></div>
<div class="stat"><div class="label">Casa stimata</div><div class="value" id="home">-- kW</div></div>
<div class="stat"><div class="label">Totale DPM</div><div class="value" id="total">-- kW</div></div>
<div class="stat"><div class="label">Energia erogata</div><div class="value" id="energy">-- kWh</div></div>
<div class="stat"><div class="label">Target</div><div class="value" id="target">-- kWh</div></div>
<div class="stat"><div class="label">Rimanente</div><div class="value" id="remaining">-- kWh</div></div>
</div></div>
<div class="card"><div class="label">Ultimo comando / esito scrittura</div><div id="command">--</div><div class="label" style="margin-top:10px">Ultima risposta wallbox</div><div id="reply">--</div></div>
<script>
const slider=document.getElementById('soc');const socStatus=document.getElementById('socStatus');const startButton=document.getElementById('start');let socModificato=false;
slider.addEventListener('input',()=>{socModificato=true;document.getElementById('socLabel').textContent=slider.value+'%';socStatus.className='soc-status';socStatus.textContent='SoC selezionato: '+slider.value+'%. Premi “Imposta SoC” per confermare.'});
async function control(action){const selectedSoc=slider.value;const body=new URLSearchParams({action,soc:selectedSoc});if(action==='start'){socStatus.className='soc-status pending';socStatus.textContent='Invio SoC '+selectedSoc+'%…';startButton.disabled=true}try{const r=await fetch('/api/control',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body,cache:'no-store'});let d={};try{d=await r.json()}catch(e){}if(!r.ok||!d.ok){if(action==='start'){socStatus.className='soc-status error';socStatus.textContent=d.error||'ESP32 non ha accettato il SoC'}else alert(d.error||('Comando non accettato: '+r.status));return}if(action==='start'){socModificato=false;slider.value=d.soc;document.getElementById('socLabel').textContent=d.soc+'%';socStatus.className='soc-status ok';socStatus.textContent='✓ SoC '+d.soc+'% ricevuto. Sessione armata, in attesa dell’auto.'}await refresh()}catch(e){if(action==='start'){socStatus.className='soc-status error';socStatus.textContent='Errore di comunicazione: il SoC non è stato confermato.'}else alert('Errore di rete: '+e)}finally{startButton.disabled=false}}
document.getElementById('start').onclick=()=>control('start');document.getElementById('stop').onclick=()=>control('stop');document.getElementById('dpmOn').onclick=()=>control('dpmOn');document.getElementById('dpmOff').onclick=()=>control('dpmOff');
async function refresh(){try{const r=await fetch('/api/status',{cache:'no-store'});const d=await r.json();document.getElementById('state').textContent=d.state;document.getElementById('ble').textContent=d.ble;document.getElementById('wifi').textContent=d.wifi;document.getElementById('dpm').textContent=d.dpm;document.getElementById('amps').textContent=d.currentA+' A';document.getElementById('car').textContent=d.carKw+' kW';document.getElementById('home').textContent=d.homeValid?d.homeKw+' kW':'N/D';document.getElementById('total').textContent=d.homeValid?d.totalKw+' kW':'N/D';document.getElementById('energy').textContent=d.delivered+' kWh';document.getElementById('target').textContent=d.target+' kWh';document.getElementById('remaining').textContent=d.remaining+' kWh';document.getElementById('command').textContent=d.command+' — '+d.tx;document.getElementById('reply').textContent=d.reply;if(!socModificato){slider.value=d.soc;document.getElementById('socLabel').textContent=d.soc+'%'}}catch(e){}}
setInterval(refresh,3000);refresh();
</script></body></html>
)HTML";
  server.send_P(200, "text/html; charset=utf-8", page);
}

void handleStatus() {
  float remaining = targetGrossKwh - deliveredKwh;
  if (remaining < 0.0f) remaining = 0.0f;

  String json = "{";
  json += "\"state\":\"" + String(stateName()) + "\",";
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
  json += "\"command\":\"" + jsonEscape(lastCommand) + "\",";
  json += "\"tx\":\"" + jsonEscape(lastTxResult) + "\",";
  json += "\"reply\":\"" + jsonEscape(lastReply) + "\"";
  json += "}";

  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", json);
}

void handleControl() {
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
    String response = "{\"ok\":true,\"soc\":" + String(startSoC) + ",\"message\":\"SoC ricevuto; sessione armata\"}";
    server.send(200, "application/json", response);
    return;
  }

  if (!bleConnected || !bleAuthenticated) {
    server.send(503, "application/json", "{\"ok\":false,\"error\":\"Wallbox BLE non autenticata\"}");
    return;
  }

  if (action == "stop") {
    requestStop();
  } else if (action == "dpmOff") {
    dpmTemporarilyOff = true;
    dpmOffStartMillis = millis();
    setDpmEnabled(false);
    lastReply = "Richiesto DPM OFF per 15 minuti";
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
  display.print(bleConnected ? "EWB:OK " : "EWB:-- ");
  display.print(dpmTemporarilyOff ? "DPM:OFF" : "DPM:ON");
  display.drawLine(0, 9, 127, 9, SSD1306_WHITE);

  if (currentState == STATE_SELECT_SOC) {
    display.setCursor(0, 14);
    display.print("SoC iniziale:");
    display.setTextSize(3);
    display.setCursor(8, 26);
    display.printf("%d%%", startSoC);
  } else if (currentState == STATE_WAITING_START) {
    display.setCursor(0, 14);
    display.print("IN ATTESA AUTO");
    display.setTextSize(2);
    display.setCursor(2, 28);
    display.printf("%.1f/%.1fkWh", deliveredKwh, targetGrossKwh);
  } else if (currentState == STATE_CHARGING) {
    display.setCursor(0, 14);
    display.printf("RICARICA %.1fA", liveCurrentA);
    display.setTextSize(2);
    display.setCursor(2, 28);
    display.printf("%.1f/%.1fkWh", deliveredKwh, targetGrossKwh);
  } else {
    display.setTextSize(2);
    display.setCursor(5, 24);
    display.print("STOP RICHIESTO");
  }

  display.setTextSize(1);
  display.setCursor(0, 56);
  display.print(nativeStopPending ? "Verifica arresto..." : "Web / BLE attivo");
}

void drawPage2() {
  display.setCursor(0, 0);
  display.printf("TELEMETRIA | DPM:%s", dpmTemporarilyOff ? "OFF" : "ON");
  display.drawLine(0, 9, 127, 9, SSD1306_WHITE);

  display.setCursor(0, 13);
  if (hasHomePowerData) display.printf("Casa stim.: %.2f kW", homePowerKw);
  else display.print("Casa stim.: N/D");

  display.setCursor(0, 24);
  display.printf("Auto: %.2f kW", wallboxPowerKw);

  display.setCursor(0, 35);
  if (hasHomePowerData) display.printf("Totale: %.2f kW", dpmTotalPowerKw);
  else display.print("Totale: N/D");

  display.setCursor(0, 48);
  display.printf("Energia: %.1f/%.1f kWh", deliveredKwh, targetGrossKwh);
}

void updateDisplay() {
  if (!oledIsOn) return;
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setTextWrap(false);

  if (currentState == STATE_SELECT_SOC) drawPage1();
  else drawPage1();
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

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  lastWifiAttempt = millis();

  unsigned long wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < 15000UL) {
    delay(250);
  }

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
  Serial.println("[WEB] Server avviato sulla porta 80.");

  updateDisplay();
}

void loop() {
  unsigned long now = millis();
  server.handleClient();

  if (nativeStopPending && now - nativeStopRequestedAt > STOP_CONFIRM_TIMEOUT_MS) {
    nativeStopPending = false;
    lastReply = nativeStopAck
      ? "ACK STOP ricevuto, ma manca conferma AD: controlla la wallbox"
      : "STOP non confermato: controlla wallbox e log BLE";
    Serial.println("[CHARGE] Timeout: stop non confermato dalla telemetria.");
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
      } else if (currentState == STATE_COMPLETED) {
        currentState = STATE_SELECT_SOC;
        nativeStopPending = false;
        nativeStopAck = false;
        screenNeedsUpdate = true;
      } else {
        requestStop();
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

      bool kwhReached = hasLiveEnergyData && deliveredKwh >= targetGrossKwh;
      bool timeReached = !hasLiveEnergyData && fallbackDurationSec > 0 && elapsedChargingSec >= fallbackDurationSec;

      if ((kwhReached || timeReached) && !nativeStopPending) {
        Serial.println("[CHARGE] Soglia stimata raggiunta; invio richiesta STOP.");
        requestStop();
        wakeUpOled();
      }
    }
  }

  if (screenNeedsUpdate) updateDisplay();
}
