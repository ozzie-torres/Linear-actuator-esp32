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
// X retains the original wiring. Y uses ordinary output-capable GPIOs that are
// neither boot-strapping pins nor input-only pins on this ESP32-WROOM-32 board.
// GPIO 18/19/23/5 remain free for a future VSPI SD-card interface. GPIO 21
// remains available if a future I2C bus is needed; its clock can be reassigned
// because GPIO 22 is used by Y ENABLE.
#define X_STEP_PIN 25
#define X_DIR_PIN 26
#define X_ENABLE_PIN 27
#define Y_STEP_PIN 16
#define Y_DIR_PIN 17
#define Y_ENABLE_PIN 22

// One minimum/home switch is assigned to each axis. Wire each switch's normally
// open (NO) contact between its GPIO and GND. Both pins have internal pull-ups,
// so no external resistors or 3.3 V switch wiring are required. An unpressed
// switch reads HIGH and a pressed switch reads LOW. Unlike NC wiring, a broken
// wire cannot be detected and will look like an unpressed switch.
#define X_LIMIT_PIN 32
#define Y_LIMIT_PIN 13
constexpr uint8_t LIMIT_TRIGGERED_LEVEL = LOW;

// Bench calibration confirmed that LOW on each DIR pin must count upward so
// negative coordinate motion travels toward the minimum/home switch. Change
// these values only if the driver/motor wiring changes; never rewire energized
// motor coils to correct coordinate direction.
constexpr bool X_DIRECTION_HIGH_COUNTS_UP = false;
constexpr bool Y_DIRECTION_HIGH_COUNTS_UP = false;

// -------------------- Motor and web server state --------------------
// The engine owns the ESP32 pulse peripherals. The pointer is null only if the
// requested STEP pin/peripheral could not be allocated during setup.
FastAccelStepperEngine stepperEngine;
WebServer server(80);

constexpr long DEFAULT_MOVE_STEPS = 200;
constexpr float DEFAULT_MAX_SPEED = 1000.0F;
constexpr float DEFAULT_ACCELERATION = 500.0F;

/** Runtime state and hardware assignment for one independently movable axis. */
struct AxisState {
  const char *name;
  uint8_t stepPin;
  uint8_t directionPin;
  uint8_t enablePin;
  uint8_t limitPin;
  bool directionHighCountsUp;
  FastAccelStepper *stepper;
  long requestedSteps;
  float configuredMaxSpeed;
  float configuredAcceleration;
  volatile bool limitEventPending;
};

// Settings are changed only by HTTP handlers on the Arduino application task.
// Only limitEventPending is shared with an ISR and is therefore volatile.
AxisState axisX = {"X", X_STEP_PIN, X_DIR_PIN, X_ENABLE_PIN, X_LIMIT_PIN,
                   X_DIRECTION_HIGH_COUNTS_UP,
                   nullptr, DEFAULT_MOVE_STEPS, DEFAULT_MAX_SPEED,
                   DEFAULT_ACCELERATION, false};
AxisState axisY = {"Y", Y_STEP_PIN, Y_DIR_PIN, Y_ENABLE_PIN, Y_LIMIT_PIN,
                   Y_DIRECTION_HIGH_COUNTS_UP,
                   nullptr, DEFAULT_MOVE_STEPS, DEFAULT_MAX_SPEED,
                   DEFAULT_ACCELERATION, false};

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
    .axes { display: grid; gap: 14px; }
    .axis-title { display: flex; align-items: center; justify-content: space-between; margin-bottom: 10px; }
    .axis-title h2 { margin: 0; font-size: 1.3rem; }
    .position { text-align: center; font-size: 2.5rem; font-variant-numeric: tabular-nums; overflow-wrap: anywhere; }
    .label { color: #9ca3af; font-size: .85rem; text-align: center; }
    .buttons { display: grid; grid-template-columns: 1fr 1fr; gap: 12px; }
    button { min-height: 64px; border: 0; border-radius: 13px; color: white; font-size: 1.15rem; font-weight: 700; touch-action: manipulation; }
    button:active { transform: scale(.98); }
    .move { background: #2563eb; }
    .stop { background: #dc2626; grid-column: 1 / -1; }
    .zero { background: #4b5563; width: 100%; margin-top: 12px; }
    .all-stop { width: 100%; margin-bottom: 14px; background: #991b1b; }
    .limit { border-radius: 999px; padding: 5px 9px; background: #065f46; font-size: .75rem; font-weight: 700; }
    .limit.hit { background: #b91c1c; }
    .fields { display: grid; gap: 12px; }
    label { color: #d1d5db; font-size: .9rem; }
    input { box-sizing: border-box; width: 100%; min-height: 50px; margin-top: 5px; padding: 10px 12px; border: 1px solid #4b5563; border-radius: 10px; background: #111827; color: white; font-size: 1.05rem; }
    .save { width: 100%; margin-top: 14px; background: #059669; }
    [id$="State"], #message { text-align: center; margin-top: 10px; color: #9ca3af; min-height: 1.2em; }
    #message.error { color: #fca5a5; }
  </style>
</head>
<body><main>
  <h1>ESP32 Two-Axis Control</h1>
  <button class="all-stop" id="stopAll">STOP BOTH AXES</button>
  <div class="axes">
  <section class="card" data-axis="x">
    <div class="axis-title"><h2>X Axis</h2><span id="xLimit" class="limit">LIMIT --</span></div>
    <div class="label">CURRENT POSITION (STEPS)</div>
    <div id="xPosition" class="position">--</div>
    <div id="xState">Connecting...</div>
    <div class="buttons">
      <button class="move" id="xNegative">X-</button>
      <button class="move" id="xPositive">X+</button>
      <button class="stop" id="xStop">STOP X</button>
    </div>
    <button class="zero" id="xZero">Zero X Position</button>
    <div class="fields">
      <label>Move distance (steps)<input id="xSteps" type="number" min="1" step="1" value="200"></label>
      <label>Max speed (steps/sec)<input id="xSpeed" type="number" min="1" step="1" value="1000"></label>
      <label>Acceleration (steps/sec&sup2;)<input id="xAccel" type="number" min="1" step="1" value="500"></label>
      <button class="save" id="xSave">Apply X Settings</button>
    </div>
  </section>
  <section class="card" data-axis="y">
    <div class="axis-title"><h2>Y Axis</h2><span id="yLimit" class="limit">LIMIT --</span></div>
    <div class="label">CURRENT POSITION (STEPS)</div>
    <div id="yPosition" class="position">--</div>
    <div id="yState">Connecting...</div>
    <div class="buttons">
      <button class="move" id="yNegative">Y-</button>
      <button class="move" id="yPositive">Y+</button>
      <button class="stop" id="yStop">STOP Y</button>
    </div>
    <button class="zero" id="yZero">Zero Y Position</button>
    <div class="fields">
      <label>Move distance (steps)<input id="ySteps" type="number" min="1" step="1" value="200"></label>
      <label>Max speed (steps/sec)<input id="ySpeed" type="number" min="1" step="1" value="1000"></label>
      <label>Acceleration (steps/sec&sup2;)<input id="yAccel" type="number" min="1" step="1" value="500"></label>
      <button class="save" id="ySave">Apply Y Settings</button>
    </div>
  </section>
  </div>
  <div id="message"></div>
</main>
<script>
const $ = id => document.getElementById(id);
const initialized = {x: false, y: false};
let commandInFlight = false;
let statusRequestInFlight = false;

// POST form data to one API endpoint and surface firmware errors in the page.
const post = async (path, values = {}) => {
  // Status polling is paused while a command is in flight. This keeps the
  // synchronous ESP32 WebServer queue clear for motion and STOP requests.
  commandInFlight = true;
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
  } finally {
    commandInFlight = false;
  }
};

const settings = axis => {
  return {axis, steps: $(axis + 'Steps').value, maxSpeed: $(axis + 'Speed').value, acceleration: $(axis + 'Accel').value};
};

// These discrete click handlers can later become pointerdown/pointerup handlers
// backed by /jog/start and /jog/stop without changing the existing move API.
for (const axis of ['x', 'y']) {
  $(axis + 'Negative').addEventListener('click', () => post('/move', {...settings(axis), direction: 'negative'}));
  $(axis + 'Positive').addEventListener('click', () => post('/move', {...settings(axis), direction: 'positive'}));
  $(axis + 'Stop').addEventListener('click', () => post('/stop', {axis}));
  $(axis + 'Zero').addEventListener('click', () => post('/zero', {axis}));
  $(axis + 'Save').addEventListener('click', () => post('/settings', settings(axis)));
}
$('stopAll').addEventListener('click', () => post('/stop', {axis: 'all'}));

// Polling is intentionally independent of motion generation. MCPWM continues
// producing stable pulses even when a network request is delayed.
const updateStatus = async () => {
  // setInterval can fire again before a slow fetch completes. Without this
  // guard those requests accumulate and make button commands appear laggy.
  if (commandInFlight || statusRequestInFlight) return;
  statusRequestInFlight = true;
  try {
    const response = await fetch('/status', {cache: 'no-store'});
    if (!response.ok) throw new Error('Status unavailable');
    const data = await response.json();
    for (const axis of ['x', 'y']) {
      const state = data[axis];
      $(axis + 'Position').textContent = state.currentPosition;
      $(axis + 'State').textContent = state.moving ? `Moving to ${state.targetPosition} (${state.distanceToGo} remaining)` : 'Stopped';
      const limit = $(axis + 'Limit');
      limit.textContent = state.limitTriggered ? 'LIMIT PRESSED' : 'LIMIT OK';
      limit.className = state.limitTriggered ? 'limit hit' : 'limit';
      if (!initialized[axis]) {
        $(axis + 'Speed').value = state.maxSpeed;
        $(axis + 'Accel').value = state.acceleration;
        $(axis + 'Steps').value = state.requestedSteps;
        initialized[axis] = true;
      }
    }
  } catch (error) {
    $('xState').textContent = error.message;
    $('yState').textContent = error.message;
  } finally {
    statusRequestInFlight = false;
  }
};
updateStatus();
setInterval(updateStatus, 500);
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

/** Resolve the required `axis=x|y` form argument to its runtime object. */
AxisState *axisFromRequest() {
  if (!server.hasArg("axis")) {
    Serial.println("ERROR: axis argument missing");
    sendJson(400, "{\"error\":\"Axis must be x or y\"}");
    return nullptr;
  }
  const String requestedAxis = server.arg("axis");
  if (requestedAxis == "x") return &axisX;
  if (requestedAxis == "y") return &axisY;
  Serial.printf("ERROR: unsupported axis '%s'\n", requestedAxis.c_str());
  sendJson(400, "{\"error\":\"Axis must be x or y\"}");
  return nullptr;
}

/** Validate and apply request settings to one hardware ramp generator. */
bool applySettingsFromRequest(AxisState &axis, bool requireSteps) {
  long newSteps = axis.requestedSteps;
  float newSpeed = axis.configuredMaxSpeed;
  float newAcceleration = axis.configuredAcceleration;

  if ((requireSteps || server.hasArg("steps")) && !parsePositiveLong("steps", newSteps)) {
    Serial.printf("ERROR: invalid %s move distance\n", axis.name);
    sendJson(400, "{\"error\":\"Move distance must be a positive integer\"}");
    return false;
  }
  if (server.hasArg("maxSpeed") && !parsePositiveFloat("maxSpeed", newSpeed)) {
    sendJson(400, "{\"error\":\"Max speed must be positive\"}");
    return false;
  }
  if (server.hasArg("acceleration") && !parsePositiveFloat("acceleration", newAcceleration)) {
    sendJson(400, "{\"error\":\"Acceleration must be positive\"}");
    return false;
  }
  if (newSpeed < 1.0F || newSpeed > static_cast<float>(INT32_MAX) ||
      newAcceleration < 1.0F || newAcceleration > static_cast<float>(INT32_MAX)) {
    sendJson(400, "{\"error\":\"Speed or acceleration is outside the supported range\"}");
    return false;
  }

  const uint32_t speedHz = static_cast<uint32_t>(lroundf(newSpeed));
  const int32_t acceleration = static_cast<int32_t>(lroundf(newAcceleration));
  if (axis.stepper != nullptr &&
      (axis.stepper->setSpeedInHz(speedHz) != 0 ||
       axis.stepper->setAcceleration(acceleration) != 0)) {
    sendJson(400, "{\"error\":\"Speed or acceleration is outside motor-controller limits\"}");
    return false;
  }

  const bool changed = newSpeed != axis.configuredMaxSpeed ||
                       newAcceleration != axis.configuredAcceleration;
  axis.requestedSteps = newSteps;
  axis.configuredMaxSpeed = static_cast<float>(speedHz);
  axis.configuredAcceleration = static_cast<float>(acceleration);
  if (changed && axis.stepper != nullptr && axis.stepper->isRunning() &&
      !axis.stepper->isStopping()) {
    axis.stepper->applySpeedAcceleration();
  }
  return true;
}

/** Serialize one axis using hardware position and its fail-safe limit input. */
String axisStatusJson(const AxisState &axis) {
  const int32_t current = axis.stepper->getCurrentPosition();
  const int32_t target = axis.stepper->targetPos();
  const bool limitTriggered = digitalRead(axis.limitPin) == LIMIT_TRIGGERED_LEVEL;
  String json;
  json.reserve(230);
  json += "{\"currentPosition\":" + String(current);
  json += ",\"targetPosition\":" + String(target);
  json += ",\"distanceToGo\":" + String(target - current);
  json += ",\"moving\":" + String(axis.stepper->isRunning() ? "true" : "false");
  json += ",\"limitTriggered\":" + String(limitTriggered ? "true" : "false");
  json += ",\"maxSpeed\":" + String(axis.configuredMaxSpeed, 2);
  json += ",\"acceleration\":" + String(axis.configuredAcceleration, 2);
  json += ",\"requestedSteps\":" + String(axis.requestedSteps) + "}";
  return json;
}

/** GET /status: return both axes' motion and limit state as JSON. */
void handleStatus() {
  if (axisX.stepper == nullptr || axisY.stepper == nullptr) {
    sendJson(503, "{\"error\":\"One or more motor pulse engines are unavailable\"}");
    return;
  }
  String json = "{\"x\":" + axisStatusJson(axisX);
  json += ",\"y\":" + axisStatusJson(axisY) + "}";
  sendJson(200, json);
}

/** POST /move: queue a positive or negative relative move on one axis. */
void handleMove() {
  AxisState *axis = axisFromRequest();
  if (axis == nullptr) return;
  if (axis->stepper == nullptr) {
    sendJson(503, "{\"error\":\"Motor pulse engine is unavailable\"}");
    return;
  }
  if (!applySettingsFromRequest(*axis, true)) return;
  if (!server.hasArg("direction")) {
    sendJson(400, "{\"error\":\"Direction must be positive or negative\"}");
    return;
  }

  const String direction = server.arg("direction");
  long signedSteps;
  if (direction == "positive" || direction == "right") signedSteps = axis->requestedSteps;
  else if (direction == "negative" || direction == "left") signedSteps = -axis->requestedSteps;
  else {
    sendJson(400, "{\"error\":\"Direction must be positive or negative\"}");
    return;
  }

  // Each switch protects the negative/minimum end. Positive motion remains
  // available to back off and close an opened limit-switch circuit.
  if (signedSteps < 0 && digitalRead(axis->limitPin) == LIMIT_TRIGGERED_LEVEL) {
    Serial.printf("ERROR: %s negative move blocked by active limit\n", axis->name);
    sendJson(409, "{\"error\":\"Negative move blocked by active limit switch\"}");
    return;
  }

  const int8_t result = axis->stepper->move(static_cast<int32_t>(signedSteps));
  if (result != MOVE_OK) {
    Serial.printf("ERROR: %s move rejected with code %d\n", axis->name, result);
    sendJson(409, "{\"error\":\"Motor controller rejected the move\"}");
    return;
  }
  Serial.printf("%s move: %s, steps: %ld, target: %ld\n", axis->name,
                direction.c_str(), axis->requestedSteps,
                static_cast<long>(axis->stepper->targetPos()));
  sendJson(200, "{\"message\":\"Move accepted\"}");
}

/** POST /stop: controlled-stop one selected axis or both axes. */
void handleStop() {
  const String requestedAxis = server.hasArg("axis") ? server.arg("axis") : "all";
  if (requestedAxis == "all") {
    if (axisX.stepper != nullptr) axisX.stepper->stopMove();
    if (axisY.stepper != nullptr) axisY.stepper->stopMove();
    Serial.println("STOP ALL command received");
  } else {
    AxisState *axis = axisFromRequest();
    if (axis == nullptr) return;
    if (axis->stepper == nullptr) {
      sendJson(503, "{\"error\":\"Motor pulse engine is unavailable\"}");
      return;
    }
    axis->stepper->stopMove();
    Serial.printf("STOP %s command received\n", axis->name);
  }
  sendJson(200, "{\"message\":\"Stopping\"}");
}

/** POST /zero: redefine one axis position as zero only at standstill. */
void handleZero() {
  AxisState *axis = axisFromRequest();
  if (axis == nullptr) return;
  if (axis->stepper == nullptr) {
    sendJson(503, "{\"error\":\"Motor pulse engine is unavailable\"}");
    return;
  }
  if (axis->stepper->isRunning()) {
    sendJson(409, "{\"error\":\"Stop the motor before zeroing\"}");
    return;
  }
  axis->stepper->setCurrentPosition(0);
  Serial.printf("Zero %s position command\n", axis->name);
  sendJson(200, "{\"message\":\"Position zeroed\"}");
}

/** POST /settings: validate and store settings for one selected axis. */
void handleSettings() {
  AxisState *axis = axisFromRequest();
  if (axis == nullptr) return;
  if (!applySettingsFromRequest(*axis, true)) return;
  Serial.printf("%s settings applied; distance: %ld steps\n",
                axis->name, axis->requestedSteps);
  sendJson(200, "{\"message\":\"Settings applied\"}");
}

/** Return a consistent JSON error for routes not registered in setup(). */
void handleNotFound() {
  Serial.printf("ERROR: HTTP 404 for %s\n", server.uri().c_str());
  sendJson(404, "{\"error\":\"Not found\"}");
}

/**
 * Hard-limit ISRs stop only the affected manual-test axis. forceStop() is
 * documented by FastAccelStepper as interrupt-safe and prevents a web request
 * from delaying limit response. Future coordinated CNC motion should stop all
 * axes and invalidate machine position; FluidNC supplies that behavior.
 */
void IRAM_ATTR handleXLimitInterrupt() {
  axisX.limitEventPending = true;
  if (axisX.stepper != nullptr) axisX.stepper->forceStop();
}

void IRAM_ATTR handleYLimitInterrupt() {
  axisY.limitEventPending = true;
  if (axisY.stepper != nullptr) axisY.stepper->forceStop();
}

/** Allocate and configure one MCPWM/PCNT-backed FastAccelStepper axis. */
bool configureAxis(AxisState &axis) {
  axis.stepper = stepperEngine.stepperConnectToPin(axis.stepPin, DRIVER_MCPWM_PCNT);
  if (axis.stepper == nullptr) {
    Serial.printf("ERROR: could not allocate %s STEP pin GPIO %u\n",
                  axis.name, axis.stepPin);
    return false;
  }
  axis.stepper->setDirectionPin(axis.directionPin, axis.directionHighCountsUp);
  axis.stepper->setEnablePin(axis.enablePin, true);  // true = active LOW
  axis.stepper->setAutoEnable(false);                // Retain holding torque.
  if (axis.stepper->setSpeedInHz(static_cast<uint32_t>(axis.configuredMaxSpeed)) != 0 ||
      axis.stepper->setAcceleration(static_cast<int32_t>(axis.configuredAcceleration)) != 0) {
    Serial.printf("ERROR: invalid default motion settings for %s\n", axis.name);
    return false;
  }
  Serial.printf("%s axis ready: STEP %u, DIR %u, ENABLE %u, LIMIT %u\n",
                axis.name, axis.stepPin, axis.directionPin,
                axis.enablePin, axis.limitPin);
  return true;
}

// -------------------- Future homing placeholder --------------------
/**
 * Reserved non-blocking homing workflow.
 *
 * Implement this as a loop-driven state machine rather than waiting in this
 * function. That preserves STOP and HTTP responsiveness during homing.
 */
void homeStepper() {
  // Future sequence for each X_LIMIT_PIN/Y_LIMIT_PIN (do not call automatically):
  // 1. Start a non-blocking negative move toward the axis switch.
  // 2. Detect the switch and stop that FastAccelStepper axis.
  // 3. Back away in the positive direction until the switch closes.
  // 4. Approach slowly for a repeatable second touch.
  // 5. Stop and call that axis's setCurrentPosition(0).
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

  // Start safely: hold both active-LOW enable outputs HIGH before motion setup.
  pinMode(X_ENABLE_PIN, OUTPUT);
  pinMode(Y_ENABLE_PIN, OUTPUT);
  digitalWrite(X_ENABLE_PIN, HIGH);
  digitalWrite(Y_ENABLE_PIN, HIGH);

  // NO-to-GND switches read HIGH while released and LOW while pressed. GPIO 32
  // and GPIO 13 both provide the internal pull-ups required by this wiring.
  pinMode(X_LIMIT_PIN, INPUT_PULLUP);
  pinMode(Y_LIMIT_PIN, INPUT_PULLUP);

  // FastAccelStepper uses ESP32 hardware peripherals and a queued ramp engine,
  // so pulse timing does not depend on web-server or Wi-Fi work in loop().
  stepperEngine.init();
  const bool xReady = configureAxis(axisX);
  const bool yReady = configureAxis(axisY);
  // NO switches transition HIGH -> LOW when pressed.
  attachInterrupt(digitalPinToInterrupt(X_LIMIT_PIN), handleXLimitInterrupt, FALLING);
  attachInterrupt(digitalPinToInterrupt(Y_LIMIT_PIN), handleYLimitInterrupt, FALLING);
  Serial.println("FastAccelStepper MCPWM/PCNT engines initialized");
  // No movement is commanded at boot.

  // Wi-Fi connection.
  WiFi.mode(WIFI_AP_STA);
  // This controller has continuous external power. Disabling station power
  // saving trades a little consumption for substantially lower HTTP latency.
  WiFi.setSleep(false);
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
  bool strongestAccessPointFound = false;
  int strongestRssi = INT_MIN;
  int32_t strongestChannel = 0;
  uint8_t strongestBssid[6] = {};
  bool wpa2FallbackFound = false;
  int wpa2FallbackRssi = INT_MIN;
  int32_t wpa2FallbackChannel = 0;
  uint8_t wpa2FallbackBssid[6] = {};
  for (int i = 0; i < networkCount; ++i) {
    if (WiFi.SSID(i) == WIFI_SSID) {
      configuredNetworkSeen = true;
      Serial.printf("Found configured SSID '%s': BSSID %s, RSSI %d dBm, channel %d, encryption %d\n",
                    WIFI_SSID, WiFi.BSSIDstr(i).c_str(), WiFi.RSSI(i),
                    WiFi.channel(i), WiFi.encryptionType(i));

      // Prefer the strongest radio regardless of whether it advertises WPA2 or
      // WPA2/WPA3 transition mode. Retain the strongest WPA2-only radio as a
      // compatibility fallback in case the mixed-security association fails.
      if (!strongestAccessPointFound || WiFi.RSSI(i) > strongestRssi) {
        strongestAccessPointFound = true;
        strongestRssi = WiFi.RSSI(i);
        strongestChannel = WiFi.channel(i);
        memcpy(strongestBssid, WiFi.BSSID(i), sizeof(strongestBssid));
      }
      if (WiFi.encryptionType(i) == WIFI_AUTH_WPA2_PSK &&
          (!wpa2FallbackFound || WiFi.RSSI(i) > wpa2FallbackRssi)) {
        wpa2FallbackFound = true;
        wpa2FallbackRssi = WiFi.RSSI(i);
        wpa2FallbackChannel = WiFi.channel(i);
        memcpy(wpa2FallbackBssid, WiFi.BSSID(i), sizeof(wpa2FallbackBssid));
      }
    }
  }
  if (!configuredNetworkSeen) {
    Serial.printf("ERROR: configured SSID '%s' was not found during scan\n", WIFI_SSID);
  }
  WiFi.scanDelete();

  if (strongestAccessPointFound) {
    Serial.printf("Connecting through strongest access point %02X:%02X:%02X:%02X:%02X:%02X at %d dBm on channel %ld\n",
                  strongestBssid[0], strongestBssid[1], strongestBssid[2],
                  strongestBssid[3], strongestBssid[4], strongestBssid[5],
                  strongestRssi, static_cast<long>(strongestChannel));
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD, strongestChannel, strongestBssid, true);
  } else {
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  }
  Serial.printf("Connecting to Wi-Fi SSID: %s", WIFI_SSID);
  const unsigned long wifiStartTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifiStartTime < 12000UL) {
    delay(500);  // Startup-only wait; no motor command exists yet.
    Serial.print('.');
  }

  // Some older ESP32 Wi-Fi stacks cannot associate with certain WPA2/WPA3
  // transition-mode radios. Retry the strongest WPA2-only BSSID when it is a
  // different radio; the direct ESP32 AP remains available throughout.
  if (WiFi.status() != WL_CONNECTED && wpa2FallbackFound &&
      memcmp(strongestBssid, wpa2FallbackBssid, sizeof(strongestBssid)) != 0) {
    Serial.printf("\nStrongest radio failed; retrying WPA2 fallback %02X:%02X:%02X:%02X:%02X:%02X at %d dBm\n",
                  wpa2FallbackBssid[0], wpa2FallbackBssid[1],
                  wpa2FallbackBssid[2], wpa2FallbackBssid[3],
                  wpa2FallbackBssid[4], wpa2FallbackBssid[5], wpa2FallbackRssi);
    WiFi.disconnect(false, false);
    delay(250);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD, wpa2FallbackChannel,
               wpa2FallbackBssid, true);
    const unsigned long fallbackStartTime = millis();
    while (WiFi.status() != WL_CONNECTED &&
           millis() - fallbackStartTime < 12000UL) {
      delay(500);  // Startup-only retry; motion is still disabled.
      Serial.print('.');
    }
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
  server.on("/favicon.ico", HTTP_GET, []() { server.send(204); });
  server.on("/status", HTTP_GET, handleStatus);
  server.on("/move", HTTP_POST, handleMove);
  server.on("/stop", HTTP_POST, handleStop);
  server.on("/zero", HTTP_POST, handleZero);
  server.on("/settings", HTTP_POST, handleSettings);
  server.onNotFound(handleNotFound);
  server.begin();
  Serial.println("HTTP server started");

  // Enable each active-LOW driver only if its complete axis setup succeeded.
  if (xReady && axisX.stepper->enableOutputs()) Serial.println("X driver enabled");
  else digitalWrite(X_ENABLE_PIN, HIGH);
  if (yReady && axisY.stepper->enableOutputs()) Serial.println("Y driver enabled");
  else digitalWrite(Y_ENABLE_PIN, HIGH);
  Serial.println("No movement commanded at boot");
}

/**
 * Service HTTP clients. Motor timing is absent by design: FastAccelStepper's
 * MCPWM/PCNT engine and background queue task continue independently.
 */
void loop() {
  // ISRs perform the time-critical stop; logging is deferred here because
  // Serial is not safe inside an interrupt handler.
  if (axisX.limitEventPending) {
    axisX.limitEventPending = false;
    Serial.println("X LIMIT PRESSED: X motion force-stopped");
  }
  if (axisY.limitEventPending) {
    axisY.limitEventPending = false;
    Serial.println("Y LIMIT PRESSED: Y motion force-stopped");
  }

  // FastAccelStepper generates queued pulses independently using ESP32 hardware.
  // The application loop only needs to keep the web server responsive.
  server.handleClient();
}
