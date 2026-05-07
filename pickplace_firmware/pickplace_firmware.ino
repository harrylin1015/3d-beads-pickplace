// pickplace_firmware.ino
// X / Y / Z-axis controller for a pick-and-place (modified 3D printer).
// Z axis uses two synced motors (Z1 + Z2).
// Runs on a Raspberry Pi Pico W via the Arduino framework.
// Communicates over USB CDC serial (Serial object = USB on the Pico W).

// ── Pin constants ──────────────────────────────────────────────────────────
const int PIN_X_STEP  = 16;
const int PIN_X_DIR   = 17;

const int PIN_Y_STEP  = 15;
const int PIN_Y_DIR   = 14;

const int PIN_Z1_STEP = 18;   // first Z motor
const int PIN_Z1_DIR  = 19;
const int PIN_Z2_STEP = 13;   // second Z motor (synced with Z1)
const int PIN_Z2_DIR  = 12;

// ── Timing constants ───────────────────────────────────────────────────────
const unsigned long DEFAULT_DELAY = 5000;  // µs between steps (lower = faster)
const long BAUD_RATE              = 9600;
const int  STEP_PULSE_US          = 4;     // step pulse HIGH duration
const int  DIR_SETTLE_US          = 5;     // settle time after changing DIR pin

// ── Direction polarity ─────────────────────────────────────────────────────
// LOW = positive direction (+X/+Y/+Z), HIGH = negative.
// Swap a pair if a motor runs the wrong way.
const int DIR_X_FWD = LOW;
const int DIR_X_BWD = HIGH;

const int DIR_Y_FWD = LOW;
const int DIR_Y_BWD = HIGH;

const int DIR_Z_FWD = LOW;
const int DIR_Z_BWD = HIGH;

// Set Z2_INVERT = true if the two Z motors are mounted on opposite ends of the
// same leadscrew/belt and face each other (mechanically mirrored).
// When true, Z2's DIR pin is always opposite to Z1's.
const bool Z2_INVERT = true;

// ── Mutable state ──────────────────────────────────────────────────────────
unsigned long stepDelayUs = DEFAULT_DELAY;

// Non-blocking serial accumulator — avoids readStringUntil() timeout lag.
String serialBuf = "";

// ── Helpers ────────────────────────────────────────────────────────────────
void initAxis(int stepPin, int dirPin, int dirFwd) {
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin,  OUTPUT);
  digitalWrite(stepPin, LOW);
  digitalWrite(dirPin, dirFwd);
}

bool allDigits(const String& s) {
  if (s.length() == 0) return false;
  for (unsigned int i = 0; i < s.length(); i++) {
    if (!isDigit(s[i])) return false;
  }
  return true;
}

// Parse the signed-integer argument of a move command (e.g. "+100" or "-50").
// Returns true and sets 'steps' on success; false on malformed input.
bool parseMoveArg(const String& arg, long& steps) {
  if (arg.length() < 2) return false;
  char sign = arg.charAt(0);
  if (sign != '+' && sign != '-') return false;
  if (!allDigits(arg.substring(1))) return false;
  steps = arg.toInt();  // String::toInt() honours the leading sign
  return true;
}

void sendErr(const String& cmd) {
  Serial.print("ERR ");
  Serial.println(cmd);
}

// ── Setup ──────────────────────────────────────────────────────────────────
void setup() {
  initAxis(PIN_X_STEP,  PIN_X_DIR,  DIR_X_FWD);
  initAxis(PIN_Y_STEP,  PIN_Y_DIR,  DIR_Y_FWD);
  initAxis(PIN_Z1_STEP, PIN_Z1_DIR, DIR_Z_FWD);
  initAxis(PIN_Z2_STEP, PIN_Z2_DIR, DIR_Z_FWD);

  Serial.begin(BAUD_RATE);
  Serial.println("READY");
}

// ── Stepper motion ─────────────────────────────────────────────────────────
void stepX(long steps) {
  if (steps == 0) return;
  digitalWrite(PIN_X_DIR, steps > 0 ? DIR_X_FWD : DIR_X_BWD);
  if (steps < 0) steps = -steps;
  delayMicroseconds(DIR_SETTLE_US);
  for (long i = 0; i < steps; i++) {
    digitalWrite(PIN_X_STEP, HIGH); delayMicroseconds(STEP_PULSE_US);
    digitalWrite(PIN_X_STEP, LOW);  delayMicroseconds(stepDelayUs);
  }
}

void stepY(long steps) {
  if (steps == 0) return;
  digitalWrite(PIN_Y_DIR, steps > 0 ? DIR_Y_FWD : DIR_Y_BWD);
  if (steps < 0) steps = -steps;
  delayMicroseconds(DIR_SETTLE_US);
  for (long i = 0; i < steps; i++) {
    digitalWrite(PIN_Y_STEP, HIGH); delayMicroseconds(STEP_PULSE_US);
    digitalWrite(PIN_Y_STEP, LOW);  delayMicroseconds(stepDelayUs);
  }
}

void stepZ(long steps) {
  if (steps == 0) return;
  bool fwd = (steps > 0);
  if (steps < 0) steps = -steps;

  digitalWrite(PIN_Z1_DIR, fwd ? DIR_Z_FWD : DIR_Z_BWD);
  // Z2_INVERT: second motor faces opposite direction on the same shaft,
  // so its DIR pin must be flipped relative to Z1.
  bool z2fwd = Z2_INVERT ? !fwd : fwd;
  digitalWrite(PIN_Z2_DIR, z2fwd ? DIR_Z_FWD : DIR_Z_BWD);

  delayMicroseconds(DIR_SETTLE_US);

  for (long i = 0; i < steps; i++) {
    // Pulse both Z step pins in the same iteration to keep them in sync.
    digitalWrite(PIN_Z1_STEP, HIGH);
    digitalWrite(PIN_Z2_STEP, HIGH);
    delayMicroseconds(STEP_PULSE_US);
    digitalWrite(PIN_Z1_STEP, LOW);
    digitalWrite(PIN_Z2_STEP, LOW);
    delayMicroseconds(stepDelayUs);
  }
}

// ── Coordinated multi-axis motion (Bresenham) ──────────────────────────────
// Interleaves step pulses so all axes move simultaneously.
// The dominant axis (most steps) sets the pace; lesser axes step at the
// proportionally correct sub-intervals — no hardware changes required.
void stepMulti(long xSteps, long ySteps, long zSteps) {
  // Set directions
  digitalWrite(PIN_X_DIR, xSteps >= 0 ? DIR_X_FWD : DIR_X_BWD);
  digitalWrite(PIN_Y_DIR, ySteps >= 0 ? DIR_Y_FWD : DIR_Y_BWD);
  bool zFwd = (zSteps >= 0);
  digitalWrite(PIN_Z1_DIR, zFwd ? DIR_Z_FWD : DIR_Z_BWD);
  digitalWrite(PIN_Z2_DIR, (Z2_INVERT ? !zFwd : zFwd) ? DIR_Z_FWD : DIR_Z_BWD);
  delayMicroseconds(DIR_SETTLE_US);

  long ax = abs(xSteps);
  long ay = abs(ySteps);
  long az = abs(zSteps);
  long dom = max(ax, max(ay, az));
  if (dom == 0) return;

  // Initialise error accumulators at dom/2 for symmetric rounding.
  long errX = dom / 2;
  long errY = dom / 2;
  long errZ = dom / 2;

  for (long i = 0; i < dom; i++) {
    bool doX = false, doY = false, doZ = false;

    errX += ax; if (errX >= dom) { errX -= dom; doX = true; }
    errY += ay; if (errY >= dom) { errY -= dom; doY = true; }
    errZ += az; if (errZ >= dom) { errZ -= dom; doZ = true; }

    if (doX) digitalWrite(PIN_X_STEP,  HIGH);
    if (doY) digitalWrite(PIN_Y_STEP,  HIGH);
    if (doZ) { digitalWrite(PIN_Z1_STEP, HIGH); digitalWrite(PIN_Z2_STEP, HIGH); }

    delayMicroseconds(STEP_PULSE_US);

    if (doX) digitalWrite(PIN_X_STEP,  LOW);
    if (doY) digitalWrite(PIN_Y_STEP,  LOW);
    if (doZ) { digitalWrite(PIN_Z1_STEP, LOW);  digitalWrite(PIN_Z2_STEP, LOW);  }

    delayMicroseconds(stepDelayUs);
  }
}

// ── Command parser ─────────────────────────────────────────────────────────
void processCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  // P — ping
  if (cmd == "P") { Serial.println("PONG"); return; }

  // S<µs> — set inter-step delay
  if (cmd.startsWith("S")) {
    String arg = cmd.substring(1);
    if (!allDigits(arg)) { sendErr(cmd); return; }
    stepDelayUs = (unsigned long)arg.toInt();
    Serial.println("DONE");
    return;
  }

  // X/Y/Z ± <n> — move single axis
  long steps;
  if (cmd.startsWith("X") && parseMoveArg(cmd.substring(1), steps)) {
    stepX(steps); Serial.println("DONE"); return;
  }
  if (cmd.startsWith("Y") && parseMoveArg(cmd.substring(1), steps)) {
    stepY(steps); Serial.println("DONE"); return;
  }
  if (cmd.startsWith("Z") && parseMoveArg(cmd.substring(1), steps)) {
    stepZ(steps); Serial.println("DONE"); return;
  }

  // M [X±n] [Y±n] [Z±n] — move multiple axes simultaneously (Bresenham)
  if (cmd == "M" || cmd.startsWith("M ")) {
    long xS = 0, yS = 0, zS = 0;
    String rest = cmd.substring(1);
    rest.trim();
    while (rest.length() > 0) {
      int sp = rest.indexOf(' ');
      String token = (sp < 0) ? rest : rest.substring(0, sp);
      rest = (sp < 0) ? "" : rest.substring(sp + 1);
      rest.trim();
      if (token.length() < 2) continue;
      char axis = token.charAt(0);
      long s;
      if (!parseMoveArg(token.substring(1), s)) continue;
      if      (axis == 'X') xS = s;
      else if (axis == 'Y') yS = s;
      else if (axis == 'Z') zS = s;
    }
    stepMulti(xS, yS, zS);
    Serial.println("DONE");
    return;
  }

  sendErr(cmd);
}

// ── Main loop ──────────────────────────────────────────────────────────────
void loop() {
  // Non-blocking accumulator — dispatch on newline, never stall on timeout.
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      processCommand(serialBuf);
      serialBuf = "";
    } else if (c != '\r') {
      serialBuf += c;
    }
  }
}
