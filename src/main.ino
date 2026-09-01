#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <FastAccelStepper.h>

/**
 * ESP32 web-controlled STEP/DIR linear actuator controller.
 *
 * Architecture:
 * - FastAccelStepper queues acceleration profiles and generates STEP pulses with
 *   ESP32 MCPWM hardware. PCNT tracks the commanded pulse position.
 * - WebServer handles a small REST-style API and an embedded mobile webpage.
 * - Wi-Fi runs in AP+station mode. The direct AP remains usable when the
 *   configured infrastructure network is unavailable.
 * - Motion is open-loop: position is the commanded pulse count, not encoder
 *   feedback. A stalled motor will therefore make reported and physical
 *   positions disagree.
 *
 * Safety invariants:
 * - ENABLE is active LOW and remains HIGH until setup completes.
 * - No movement is issued during boot.
 * - Zeroing is rejected while the hardware queue is moving.
 * - STOP requests normal acceleration-limited deceleration.
 */

// Keep real credentials out of source control. A clean clone still compiles
// with obvious placeholders, but a deployed device should use src/secrets.h.
#if __has_include("secrets.h")
#include "secrets.h"
#else
#include "secrets.example.h"
#warning "Using placeholder network credentials from secrets.example.h"
#endif

// -------------------- Hardware pins --------------------
#define STEP_PIN 25
#define DIR_PIN 26
#define ENABLE_PIN 27
#define HOME_SWITCH_PIN 33  // Reserved for a future origin switch.

// -------------------- Motor and web server state --------------------
// The engine owns the ESP32 pulse peripherals. The pointer is null only if the
// requested STEP pin/peripheral could not be allocated during setup.
FastAccelStepperEngine stepperEngine;
FastAccelStepper *stepper = nullptr;
WebServer server(80);

constexpr long DEFAULT_MOVE_STEPS = 200;
constexpr float DEFAULT_MAX_SPEED = 1000.0F;
constexpr float DEFAULT_ACCELERATION = 500.0F;

// Settings are shared between HTTP handlers. They are changed only by the
// Arduino application task, so no mutex is required.
long requestedSteps = DEFAULT_MOVE_STEPS;
float configuredMaxSpeed = DEFAULT_MAX_SPEED;
float configuredAcceleration = DEFAULT_ACCELERATION;

// The entire UI lives in flash, avoiding a filesystem dependency. Keep named
// JavaScript functions expressed as arrow functions: Arduino's .ino preprocessor
// can otherwise mistake JavaScript `function` declarations for C++ and inject
// invalid prototypes into this raw string.
const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1">
  <title>ESP32 Stepper</title>
  <style>
    :root { color-scheme: dark; font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif; }
    body { margin: 0; background: #111827; color: #f9fafb; }
    main { max-width: 560px; margin: auto; padding: 18px; }
    h1 { font-size: 1.55rem; margin: 6px 0 18px; }
    .card { background: #1f2937; border-radius: 16px; padding: 16px; margin-bottom: 14px; box-shadow: 0 5px 20px #0005; }
    .position { text-align: center; font-size: 3rem; font-variant-numeric: tabular-nums; overflow-wrap: anywhere; }
    .label { color: #9ca3af; font-size: .85rem; text-align: center; }
    .buttons { display: grid; grid-template-columns: 1fr 1fr; gap: 12px; }
    button { min-height: 64px; border: 0; border-radius: 13px; color: white; font-size: 1.15rem; font-weight: 700; touch-action: manipulation; }
    button:active { transform: scale(.98); }
    .move { background: #2563eb; }
    .stop { background: #dc2626; grid-column: 1 / -1; }
    .zero { background: #4b5563; width: 100%; margin-top: 12px; }
    .fields { display: grid; gap: 12px; }
    label { color: #d1d5db; font-size: .9rem; }
    input { box-sizing: border-box; width: 100%; min-height: 50px; margin-top: 5px; padding: 10px 12px; border: 1px solid #4b5563; border-radius: 10px; background: #111827; color: white; font-size: 1.05rem; }
    .save { width: 100%; margin-top: 14px; background: #059669; }
    #state, #message { text-align: center; margin-top: 10px; color: #9ca3af; min-height: 1.2em; }
    #message.error { color: #fca5a5; }
  </style>
</head>
<body><main>
  <h1>ESP32 Stepper Control</h1>
  <section class="card">
    <div class="label">CURRENT POSITION (STEPS)</div>
    <div id="position" class="position">--</div>
    <div id="state">Connecting...</div>
  </section>
  <section class="card">
    <div class="buttons">
      <button class="move" id="left">&#9664; LEFT</button>
      <button class="move" id="right">RIGHT &#9654;</button>
      <button class="stop" id="stop">STOP</button>
    </div>
    <button class="zero" id="zero">Zero Position</button>
    <div id="message"></div>
  </section>
  <section class="card fields">
    <label>Move distance (steps)<input id="steps" type="number" min="1" step="1" value="200"></label>
    <label>Max speed (steps/sec)<input id="speed" type="number" min="1" step="1" value="1000"></label>
    <label>Acceleration (steps/sec&sup2;)<input id="accel" type="number" min="1" step="1" value="500"></label>
    <button class="save" id="save">Apply Settings</button>
  </section>
</main>
<script>
const $ = id => document.getElementById(id);
let firstStatus = true;

// POST form data to one API endpoint and surface firmware errors in the page.
const post = async (path, values = {}) => {
  const body = new URLSearchParams(values);
  try {
    const response = await fetch(path, {method: 'POST', headers: {'Content-Type': 'application/x-www-form-urlencoded'}, body});
    const result = await response.json();
    if (!response.ok) throw new Error(result.error || ('HTTP ' + response.status));
    $('message').textContent = result.message || '';
    $('message').className = '';
    updateStatus();
  } catch (error) {
    $('message').textContent = error.message;
    $('message').className = 'error';
  }
};

const settings = () => {
  return {steps: $('steps').value, maxSpeed: $('speed').value, acceleration: $('accel').value};
};

// These discrete click handlers can later become pointerdown/pointerup handlers
// backed by /jog/start and /jog/stop without changing the existing move API.
$('left').addEventListener('click', () => post('/move', {...settings(), direction: 'left'}));
$('right').addEventListener('click', () => post('/move', {...settings(), direction: 'right'}));
$('stop').addEventListener('click', () => post('/stop'));
$('zero').addEventListener('click', () => post('/zero'));
$('save').addEventListener('click', () => post('/settings', settings()));

// Polling is intentionally independent of motion generation. MCPWM continues
// producing stable pulses even when a network request is delayed.
const updateStatus = async () => {
  try {
    const response = await fetch('/status', {cache: 'no-store'});
    if (!response.ok) throw new Error('Status unavailable');
    const data = await response.json();
    $('position').textContent = data.currentPosition;
    $('state').textContent = data.moving ? `Moving to ${data.targetPosition} (${data.distanceToGo} remaining)` : 'Stopped';
    if (firstStatus) {
      $('speed').value = data.maxSpeed;
      $('accel').value = data.acceleration;
      $('steps').value = data.requestedSteps;
      firstStatus = false;
    }
  } catch (error) {
    $('state').textContent = error.message;
  }
};
updateStatus();
setInterval(updateStatus, 300);
</script></body></html>
)HTML";

// -------------------- HTTP helpers and API handlers --------------------
/** Send a JSON response with caching disabled so position is never stale. */
void sendJson(int statusCode, const String &json) {
  server.sendHeader("Cache-Control", "no-store");
  server.send(statusCode, "application/json", json);
}

/** Parse a required positive base-10 integer form argument. */
bool parsePositiveLong(const String &name, long &value) {
  if (!server.hasArg(name) || server.arg(name).isEmpty()) return false;
  char *end = nullptr;
  const String text = server.arg(name);
  const long parsed = strtol(text.c_str(), &end, 10);
  if (*end != '\0' || parsed <= 0) return false;
  value = parsed;
  return true;
}

/** Parse a required positive finite floating-point form argument. */
bool parsePositiveFloat(const String &name, float &value) {
  if (!server.hasArg(name) || server.arg(name).isEmpty()) return false;
  char *end = nullptr;
  const String text = server.arg(name);
  const float parsed = strtof(text.c_str(), &end);
  if (*end != '\0' || !isfinite(parsed) || parsed <= 0.0F) return false;
  value = parsed;
  return true;
}

/**
 * Validate and apply optional motion settings from the current HTTP request.
 *
 * FastAccelStepper accepts integer steps/s and steps/s^2. Web values are
 * rounded to those native units. When already moving, applySpeedAcceleration()
 * updates the queued ramp without coupling pulse timing to this HTTP handler.
 * Returns false after sending an HTTP error response.
 */
bool applySettingsFromRequest(bool requireSteps) {
  long newSteps = requestedSteps;
  float newSpeed = configuredMaxSpeed;
  float newAcceleration = configuredAcceleration;

  if ((requireSteps || server.hasArg("steps")) && !parsePositiveLong("steps", newSteps)) {
    Serial.println("ERROR: invalid move distance");
    sendJson(400, "{\"error\":\"Move distance must be a positive integer\"}");
    return false;
  }
  if (server.hasArg("maxSpeed") && !parsePositiveFloat("maxSpeed", newSpeed)) {
    Serial.println("ERROR: invalid max speed");
    sendJson(400, "{\"error\":\"Max speed must be positive\"}");
    return false;
  }
  if (server.hasArg("acceleration") && !parsePositiveFloat("acceleration", newAcceleration)) {
    Serial.println("ERROR: invalid acceleration");
    sendJson(400, "{\"error\":\"Acceleration must be positive\"}");
    return false;
  }

  // Validate before float-to-integer conversion; out-of-range conversion has
  // undefined behavior in C++ and should never reach the motion library.
  if (newSpeed < 1.0F || newSpeed > static_cast<float>(INT32_MAX) ||
      newAcceleration < 1.0F || newAcceleration > static_cast<float>(INT32_MAX)) {
    Serial.println("ERROR: speed or acceleration is outside integer range");
    sendJson(400, "{\"error\":\"Speed or acceleration is outside the supported range\"}");
    return false;
  }
  const uint32_t speedHz = static_cast<uint32_t>(lroundf(newSpeed));
  const int32_t acceleration = static_cast<int32_t>(lroundf(newAcceleration));

  if (stepper != nullptr &&
      (stepper->setSpeedInHz(speedHz) != 0 || stepper->setAcceleration(acceleration) != 0)) {
    Serial.println("ERROR: speed or acceleration is outside FastAccelStepper limits");
    sendJson(400, "{\"error\":\"Speed or acceleration is outside motor-controller limits\"}");
    return false;
  }

  const bool motionSettingsChanged =
      newSpeed != configuredMaxSpeed || newAcceleration != configuredAcceleration;
  if (newSpeed != configuredMaxSpeed) {
    configuredMaxSpeed = static_cast<float>(speedHz);
    Serial.printf("Max speed changed to %.0f steps/sec\n", configuredMaxSpeed);
  }
  if (newAcceleration != configuredAcceleration) {
    configuredAcceleration = static_cast<float>(acceleration);
    Serial.printf("Acceleration changed to %.0f steps/sec^2\n", configuredAcceleration);
  }

  if (stepper != nullptr) {
    if (motionSettingsChanged && stepper->isRunning() && !stepper->isStopping()) {
      stepper->applySpeedAcceleration();
    }
  }
  requestedSteps = newSteps;
  return true;
}

/** GET /status: return the hardware engine's current motion state as JSON. */
void handleStatus() {
  if (stepper == nullptr) {
    sendJson(503, "{\"error\":\"Motor pulse engine is unavailable\"}");
    return;
  }
  const int32_t currentPosition = stepper->getCurrentPosition();
  const int32_t targetPosition = stepper->targetPos();
  const int32_t distance = targetPosition - currentPosition;
  const bool moving = stepper->isRunning();
  String json;
  json.reserve(220);
  json += "{\"currentPosition\":" + String(currentPosition);
  json += ",\"targetPosition\":" + String(targetPosition);
  json += ",\"distanceToGo\":" + String(distance);
  json += ",\"moving\":" + String(moving ? "true" : "false");
  json += ",\"maxSpeed\":" + String(configuredMaxSpeed, 2);
  json += ",\"acceleration\":" + String(configuredAcceleration, 2);
  json += ",\"requestedSteps\":" + String(requestedSteps) + "}";
  sendJson(200, json);
}

/**
 * POST /move: queue a signed relative move.
 * `right` is positive and `left` is negative. FastAccelStepper interprets move()
 * relative to the existing target, matching the original UI behavior.
 */
void handleMove() {
  if (stepper == nullptr) {
    sendJson(503, "{\"error\":\"Motor pulse engine is unavailable\"}");
    return;
  }
  if (!applySettingsFromRequest(true)) return;
  if (!server.hasArg("direction")) {
    Serial.println("ERROR: move direction missing");
    sendJson(400, "{\"error\":\"Direction must be left or right\"}");
    return;
  }

  const String direction = server.arg("direction");
  long signedSteps;
  if (direction == "right") signedSteps = requestedSteps;
  else if (direction == "left") signedSteps = -requestedSteps;
  else {
    Serial.println("ERROR: invalid move direction");
    sendJson(400, "{\"error\":\"Direction must be left or right\"}");
    return;
  }

  const int8_t result = stepper->move(static_cast<int32_t>(signedSteps));
  if (result != MOVE_OK) {
    Serial.printf("ERROR: FastAccelStepper rejected move with code %d\n", static_cast<int>(result));
    sendJson(409, "{\"error\":\"Motor controller rejected the move\"}");
    return;
  }
  Serial.printf("Move command: %s, requested steps: %ld, target: %ld\n",
                direction.c_str(), requestedSteps, static_cast<long>(stepper->targetPos()));
  sendJson(200, "{\"message\":\"Move accepted\"}");
}

/** POST /stop: request a non-blocking, acceleration-limited controlled stop. */
void handleStop() {
  if (stepper == nullptr) {
    sendJson(503, "{\"error\":\"Motor pulse engine is unavailable\"}");
    return;
  }
  // FastAccelStepper's hardware-backed queue performs a controlled deceleration.
  stepper->stopMove();
  Serial.println("STOP command received (controlled deceleration)");
  sendJson(200, "{\"message\":\"Stopping\"}");
}

/** POST /zero: redefine commanded position as zero, but only at standstill. */
void handleZero() {
  // Re-zeroing during motion would redefine coordinates while moving, so reject
  // it. STOP first and wait for the status to report stopped.
  if (stepper == nullptr) {
    sendJson(503, "{\"error\":\"Motor pulse engine is unavailable\"}");
    return;
  }
  if (stepper->isRunning()) {
    Serial.println("ERROR: zero requested while motor is moving");
    sendJson(409, "{\"error\":\"Stop the motor before zeroing\"}");
    return;
  }
  stepper->setCurrentPosition(0);
  Serial.println("Zero position command: current position set to 0");
  sendJson(200, "{\"message\":\"Position zeroed\"}");
}

/** POST /settings: validate/store distance, speed, and acceleration values. */
void handleSettings() {
  if (!applySettingsFromRequest(true)) return;
  Serial.printf("Settings applied; move distance: %ld steps\n", requestedSteps);
  sendJson(200, "{\"message\":\"Settings applied\"}");
}

/** Return a consistent JSON error for routes not registered in setup(). */
void handleNotFound() {
  Serial.printf("ERROR: HTTP 404 for %s\n", server.uri().c_str());
  sendJson(404, "{\"error\":\"Not found\"}");
}

// -------------------- Future homing placeholder --------------------
/**
 * Reserved non-blocking homing workflow.
 *
 * Implement this as a loop-driven state machine rather than waiting in this
 * function. That preserves STOP and HTTP responsiveness during homing.
 */
void homeStepper() {
  // Future sequence (do not call automatically):
  // 1. Start a non-blocking move toward HOME_SWITCH_PIN.
  // 2. Detect the switch and stop with FastAccelStepper.
  // 3. Back away until the switch releases.
  // 4. Approach the switch again slowly for repeatability.
  // 5. Stop and call stepper->setCurrentPosition(0).
  // Implement this later as a non-blocking state machine so HTTP stays responsive.
}

/** Log station disconnect reason codes for Wi-Fi troubleshooting. */
void handleWiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info) {
  if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) {
    Serial.printf("\nWi-Fi disconnected; reason code: %u\n",
                  info.wifi_sta_disconnected.reason);
  }
}

/**
 * Initialize safe GPIO state, hardware motion engine, both Wi-Fi interfaces,
 * API routes, and finally the motor driver. No move is queued here.
 */
void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("ESP32 stepper controller starting");

  // Start safely: hold the active-LOW enable HIGH before configuring motion.
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, HIGH);

  // FastAccelStepper uses ESP32 hardware peripherals and a queued ramp engine,
  // so pulse timing does not depend on web-server or Wi-Fi work in loop().
  stepperEngine.init();
  // Explicitly use ESP32 MCPWM for pulse timing and PCNT for position tracking.
  stepper = stepperEngine.stepperConnectToPin(STEP_PIN, DRIVER_MCPWM_PCNT);
  if (stepper != nullptr) {
    stepper->setDirectionPin(DIR_PIN);
    stepper->setEnablePin(ENABLE_PIN, true);  // true = active LOW
    stepper->setAutoEnable(false);            // Keep holding torque when stopped.
    stepper->setSpeedInHz(static_cast<uint32_t>(configuredMaxSpeed));
    stepper->setAcceleration(static_cast<int32_t>(configuredAcceleration));
    Serial.println("FastAccelStepper MCPWM/PCNT pulse engine initialized");
  } else {
    Serial.println("ERROR: FastAccelStepper could not allocate STEP_PIN");
  }
  // No movement is commanded at boot.

  // Wi-Fi connection.
  WiFi.mode(WIFI_AP_STA);
  WiFi.onEvent(handleWiFiEvent);

  // Keep a direct phone-to-ESP32 network available even if the home Wi-Fi is
  // weak or unavailable. The AP and station interfaces share this web server.
  if (WiFi.softAP(AP_SSID, AP_PASSWORD)) {
    Serial.printf("Direct access point started: %s\n", AP_SSID);
    Serial.print("Direct access URL: http://");
    Serial.println(WiFi.softAPIP());
  } else {
    Serial.println("ERROR: could not start direct access point");
  }

  Serial.println("Scanning for nearby Wi-Fi networks...");
  const int networkCount = WiFi.scanNetworks();
  bool configuredNetworkSeen = false;
  bool preferredAccessPointFound = false;
  int32_t preferredChannel = 0;
  uint8_t preferredBssid[6] = {};
  for (int i = 0; i < networkCount; ++i) {
    if (WiFi.SSID(i) == WIFI_SSID) {
      configuredNetworkSeen = true;
      Serial.printf("Found configured SSID '%s': BSSID %s, RSSI %d dBm, channel %d, encryption %d\n",
                    WIFI_SSID, WiFi.BSSIDstr(i).c_str(), WiFi.RSSI(i),
                    WiFi.channel(i), WiFi.encryptionType(i));

      // This network has both WPA2-only and WPA2/WPA3 radios with the same SSID.
      // Prefer WPA2-only because this ESP32 repeatedly fails authentication on
      // the mixed-security radio even though the password is correct.
      if (!preferredAccessPointFound && WiFi.encryptionType(i) == WIFI_AUTH_WPA2_PSK) {
        preferredAccessPointFound = true;
        preferredChannel = WiFi.channel(i);
        memcpy(preferredBssid, WiFi.BSSID(i), sizeof(preferredBssid));
      }
    }
  }
  if (!configuredNetworkSeen) {
    Serial.printf("ERROR: configured SSID '%s' was not found during scan\n", WIFI_SSID);
  }
  WiFi.scanDelete();

  if (preferredAccessPointFound) {
    Serial.printf("Connecting through WPA2 access point %02X:%02X:%02X:%02X:%02X:%02X on channel %ld\n",
                  preferredBssid[0], preferredBssid[1], preferredBssid[2],
                  preferredBssid[3], preferredBssid[4], preferredBssid[5],
                  static_cast<long>(preferredChannel));
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD, preferredChannel, preferredBssid, true);
  } else {
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  }
  Serial.printf("Connecting to Wi-Fi SSID: %s", WIFI_SSID);
  const unsigned long wifiStartTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifiStartTime < 20000UL) {
    delay(500);  // Startup-only wait; no motor command exists yet.
    Serial.print('.');
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("ESP32 station IP address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("ERROR: station connection timed out; direct access point remains available");
  }

  // HTTP routes.
  server.on("/", HTTP_GET, []() { server.send_P(200, "text/html", INDEX_HTML); });
  server.on("/status", HTTP_GET, handleStatus);
  server.on("/move", HTTP_POST, handleMove);
  server.on("/stop", HTTP_POST, handleStop);
  server.on("/zero", HTTP_POST, handleZero);
  server.on("/settings", HTTP_POST, handleSettings);
  server.onNotFound(handleNotFound);
  server.begin();
  Serial.println("HTTP server started");

  // Enable the active-LOW motor driver only after initialization is complete.
  if (stepper != nullptr && stepper->enableOutputs()) {
    Serial.println("Motor driver enabled; no movement commanded");
  } else {
    digitalWrite(ENABLE_PIN, HIGH);
    Serial.println("ERROR: motor driver remains disabled");
  }
}

/**
 * Service HTTP clients. Motor timing is absent by design: FastAccelStepper's
 * MCPWM/PCNT engine and background queue task continue independently.
 */
void loop() {
  // FastAccelStepper generates queued pulses independently using ESP32 hardware.
  // The application loop only needs to keep the web server responsive.
  server.handleClient();
}
