/*
  ============================================================================
  Centrifuge Controller – v1.0 (STABLE RELEASE)

  Release status: validated in states 0..4 with load variations and lid-on test at 4000 RPM.
  ============================================================================
  Hardware:
    - Arduino Uno
    - Motor driver: BTS7960
    - Motor: 775 DC motor
    - RPM sensor: Hall sensor + 1 magnet (1 pulse per revolution)
    - Display: TM1637 4-digit

  What this firmware does (high level):
    - BTN1 toggles motor RUN/STOP.
    - BTN2 short press shows current state (---N) for 3 seconds.
      If pressed again while showing state, it advances state 1→2→3→4→1.
    - BTN2 long press (3 seconds) switches to safe state 0 (---0).
    - While running:
        * Measures RPM from hall pulses with filtering + EMA smoothing.
        * Governor increases PWM slowly until close to target, then trims gently.
        * When stable and close enough, it "locks" PWM to keep things calm.
    - Safety cutoff stops the motor if RPM exceeds RPM_LIMIT for RPM_TRIP_COUNT checks.
    - Display shows live RPM, unless showing ---N or STOP.
      Optional: when locked, display can show the exact target RPM (UX mode).

  How to operate:
    - BTN1: Start/Stop motor.
    - BTN2 short press:
        * If not showing state: show ---N for 3 seconds (N = current state).
        * If already showing state: advance state 1→2→3→4→1 and keep showing ---N.
    - BTN2 long press (hold 3 seconds): switch to safe state 0 (---0).
    - Display:
        * Normally shows RPM (measured).
        * While showing ---N or STOP, RPM display is paused.
        * Optional UX: SHOW_TARGET_WHEN_LOCKED can show the target RPM when locked.
    - Serial (only if SERIAL_ENABLED = true):
        * r = run/stop, 0..4 = select state, s = status, h = help.

    v1.0 Contract (what must NOT change without intent):
    - BTN1/BTN2 user interaction semantics must remain identical.
    - Safety cutoff behavior must remain identical (RPM_LIMIT + RPM_TRIP_COUNT).
    - PWM must always be applied via ramping (no sudden jumps).
    - Hall/RPM filtering must stay robust (no raw RPM directly controlling PWM).
    - SHOW_TARGET_WHEN_LOCKED only affects display output, never control.
    - SERIAL interface (if enabled) must mirror button behavior, not replace it.

  Notes for beginners:
    - "PWM" is the motor power command (0..255).
    - We never jump PWM suddenly; we ramp by small steps for safety.
    - RPM measurement from a hall sensor can be noisy; we reject bad pulses.
*/

/*
  ----------------------------------------------------------------------------
  Glossary (quick, beginner-friendly)
  ----------------------------------------------------------------------------
  RPM
    Revolutions Per Minute. How fast the rotor spins.

  PWM
    Pulse Width Modulation. A number (0..255) that controls motor power.
    Higher PWM = more power = higher RPM (usually).

  Hall sensor pulse / dt
    The hall sensor gives one pulse per revolution (with one magnet).
    dt (delta time) is the time between two pulses (in microseconds).
    Smaller dt = faster spin.

  PPR (Pulses Per Revolution)
    How many pulses we get per full revolution. Here: 1 magnet => PPR = 1.

  Timeout (RPM timeout)
    If no hall pulse is seen for a while, we assume the rotor has stopped
    (RPM becomes 0). We use a longer timeout while running, shorter while stopped.

  Filtering (spike / missed-pulse reject)
    Hall readings can be noisy. We reject impossible dt values that are
    too big/small compared to the last good dt.

  EMA (Exponential Moving Average)
    A smoothing method that blends new RPM readings with older ones.
    Makes the RPM value calmer and more stable (less jitter).

  Governor (control loop)
    The logic that adjusts PWM to reach and maintain the target RPM.

  Target RPM
    The desired RPM for the current state (0..4), e.g. 1000 RPM for state 1.

  Error (err)
    err = targetRPM - measuredRPM.
    Positive err means we are below target (need more power).
    Negative err means we are above target (need less power / hold steady).

  Deadband (DB)
    A small “okay zone” around the target where we avoid constant tiny corrections.

  Stability check
    Uses the last few raw RPM samples and only trusts them when the spread is small.
    Prevents the controller from reacting to noise/vibration.

  Modes: CLIMB / HOLD / LOCK
    CLIMB: slowly increase/decrease PWM in small steps to approach the target.
    HOLD : gently trim PWM (small corrections) to stay near target.
    LOCK : freeze PWM when stable, to avoid hunting. It can unlock if drift is large.

  Ramp (PWM ramping)
    Even if the governor requests a new PWM instantly, we apply it gradually
    (small steps every RAMP_DT_MS) to keep motion smooth and safe.

  Display option (SHOW_TARGET_WHEN_LOCKED)
    If true: when locked, show the target RPM (calm commercial UX).
    If false: always show the measured RPM (honest mode).
*/


#include <Arduino.h>
#include <TM1637Display.h>

// ============================================================================
// Pin configuration
// ============================================================================
static const uint8_t PIN_RPWM = 5;
static const uint8_t PIN_LPWM = 6;
static const uint8_t PIN_REN  = 7;
static const uint8_t PIN_LEN  = 8;

static const uint8_t PIN_BTN1 = 9;
static const uint8_t PIN_BTN2 = 10;

static const uint8_t PIN_CLK  = 3;
static const uint8_t PIN_DIO  = 4;

static const uint8_t PIN_HALL = 2;

TM1637Display display(PIN_CLK, PIN_DIO);


// ============================================================================
// Build options (safe to toggle; do not change control behavior)
// ============================================================================
// Display option:
// true  = when locked, show the target RPM (commercial-style UX)
// false = always show measured RPM (honest mode)
static const bool SHOW_TARGET_WHEN_LOCKED = false;

// Optional serial interface (debug / fallback control)
static const bool SERIAL_ENABLED = false;


// ============================================================================
// Fixed configuration / tuning constants
// ============================================================================

// Motor / safety PWM limits
static const uint8_t PWM_MAX_SAFE   = 160;   // maximum PWM we allow (safety)
static const uint8_t PWM_START_SAFE = 6;     // minimum start PWM when running

// RPM targets per state: 0..4
static const uint16_t RPM_TARGETS[5] = { 300, 1000, 2000, 3000, 4000 };

// Soft PWM ramping (how quickly we approach the target PWM)
static const uint8_t  RAMP_STEP  = 1;
static const uint16_t RAMP_DT_MS = 120;

// Button debounce & long press
static const uint16_t DEBOUNCE_MS  = 30;
static const uint16_t LONGPRESS_MS = 3000;

// RPM measurement constants
static const uint8_t  PULSES_PER_REV    = 1;
static const uint16_t RPM_DISPLAY_CLAMP = 9999;

// Safety cutoff
static const uint16_t RPM_LIMIT      = 4200;
static const uint8_t  RPM_TRIP_COUNT = 3;

// Hall pulse timeouts
static const uint32_t RPM_TIMEOUT_US_RUN  = 2000000UL; // when running: forgiving
static const uint32_t RPM_TIMEOUT_US_STOP = 800000UL;  // when stopped: show 0 fast (~0.8s)
static const uint32_t HALL_MIN_DT_US      = 8500UL;    // reject noise faster than ~7000 RPM

// Reject "missed pulse" / "spike" dt values compared to the last good dt
static const float DT_TOO_BIG_FACTOR   = 1.55f;
static const float DT_TOO_SMALL_FACTOR = 0.25f;

// EMA smoothing for RPM used by control and running display
static const float RPM_EMA_ALPHA = 0.25f;

// Governor behavior
static const int16_t  DB_RPM        = 50;    // deadband (normal)
static const int16_t  DB_S1         = 60;    // deadband near 1000 RPM
static const int16_t  UNLOCK_RPM    = 80;    // unlock PWM lock if drift is large
static const uint16_t LOCK_TIME_MS  = 900;   // must stay stable+in DB long enough
static const uint16_t CLIMB_DT_MS   = 400;   // interval between climb steps
static const uint16_t HOLD_DT_MS    = 180;   // trim interval
static const uint16_t HOLD_DT_S1    = 220;   // trim interval near 1000 RPM
static const uint8_t  CLIMB_STEP_PWM = 1;    // PWM step during climb

// HOLD PI trim (small, gentle corrections)
static const float HOLD_KP    = 0.0016f;
static const float HOLD_KI    = 0.00010f;
static const float HOLD_I_MAX = 14.0f;

// State 0 gentle cap (to avoid overshoot at ~300 RPM)
static const uint8_t PWM_MAX_S0 = 8;         // gentle cap for state 0 to reduce overshoot


// ============================================================================
// Runtime state variables
// ============================================================================

// Running state + PWM
static bool    g_running     = false;
static uint8_t g_pwm_current = 0;    // actually applied PWM
static uint8_t g_pwm_target  = 0;    // governor request PWM (ramp approaches it)

// RPM state selection
static uint8_t       g_rpm_state  = 1;   // 0..4
static unsigned long g_lastRampMs = 0;

// Button debounce tracking (BTN1)
static bool          g_btn1Stable = HIGH;
static bool          g_btn1Last   = HIGH;
static unsigned long g_btn1LastChangeMs = 0;

// Button debounce + long press tracking (BTN2)
static bool          g_btn2Stable = HIGH;
static bool          g_btn2Last   = HIGH;
static unsigned long g_btn2LastChangeMs = 0;

static bool          g_btn2Pressing = false;
static unsigned long g_btn2PressStartMs = 0;
static bool          g_btn2LongHandled = false;

// Display mode: showing ---N or STOP for 3 seconds
static bool          g_showingState = false;
static unsigned long g_stateShowUntilMs = 0;

// Display value (what we actually show when not showing state/STOP)
static uint16_t g_rpmDisplayValue = 0;

// Measured RPM used by control (EMA smoothed)
static uint16_t g_rpmValue = 0;

// Hall ISR shared values (volatile because ISR writes them)
static volatile uint32_t g_isrLastEdgeUs = 0;
static volatile uint32_t g_isrDtUs       = 0;
static volatile bool     g_isrHasNewDt   = false;

// Measurement filter state
static uint32_t g_lastGoodDtUs = 0;
static float    g_rpmEma       = 0.0f;

// Stability tracking (last 4 raw RPM samples)
static uint16_t g_rpmHist[4] = {0, 0, 0, 0};
static uint8_t  g_rpmHistCount = 0;
static uint8_t  g_rpmHistIdx   = 0;

// Safety counter
static uint8_t g_overLimitCount = 0;

// Governor state machine
enum GovMode : uint8_t { GOV_CLIMB = 0, GOV_HOLD = 1, GOV_LOCK = 2 };
static GovMode        g_govMode      = GOV_CLIMB;
static unsigned long  g_lastClimbMs  = 0;
static unsigned long  g_lastHoldMs   = 0;
static unsigned long  g_inDbSinceMs  = 0;
static float          g_holdI        = 0.0f;


// ============================================================================
// Serial interface (optional control + diagnostics)
// - Does NOT replace buttons or display
// - Safe fallback if buttons/display are missing
// ============================================================================
static unsigned long g_lastSerialPrintMs = 0;

static void serialInit() {
  Serial.begin(115200);
  Serial.println(F("\nCentrifuge Controller Serial Interface"));
  Serial.println(F("Commands: r=run/stop, 0..4=state, s=status, h=help"));
}

static void serialPrintStatus() {
  if (!SERIAL_ENABLED) return;

  Serial.print(F("RPM="));
  Serial.print(g_rpmValue);
  Serial.print(F("  STATE="));
  Serial.print(g_rpm_state);
  Serial.print(F("  PWM="));
  Serial.print(g_pwm_current);
  Serial.print(F("  MODE="));

  switch (g_govMode) {
    case GOV_CLIMB: Serial.println(F("CLIMB")); break;
    case GOV_HOLD:  Serial.println(F("HOLD"));  break;
    case GOV_LOCK:  Serial.println(F("LOCK"));  break;
    default:        Serial.println(F("?"));     break;
  }
}

static void handleSerial(unsigned long nowMs) {
  if (!SERIAL_ENABLED) return;

  // Periodic status print (1 Hz while running)
  if (g_running && (unsigned long)(nowMs - g_lastSerialPrintMs) >= 1000UL) {
    g_lastSerialPrintMs = nowMs;
    serialPrintStatus();
  }

  if (!Serial.available()) return;

  char c = Serial.read();

  if (c == 'r' || c == 'R') {
    toggleRun();
    serialPrintStatus();
  }
  else if (c >= '0' && c <= '4') {
    g_rpm_state = (uint8_t)(c - '0');
    if (g_running) applyRPMPreset();
    resetRPMControl();
    serialPrintStatus();
  }
  else if (c == 's' || c == 'S') {
    serialPrintStatus();
  }
  else if (c == 'h' || c == 'H') {
    Serial.println(F("Commands:"));
    Serial.println(F("  r  = run / stop"));
    Serial.println(F("  0-4= select state"));
    Serial.println(F("  s  = show status"));
    Serial.println(F("  h  = help"));
  }
}

// ============================================================================
// Interrupt Service Routine: hall sensor pulse timing
// ============================================================================
void isrHallFalling() {
  uint32_t nowUs = micros();

  // First edge after start: store timestamp and ignore dt (avoids huge dt)
  if (g_isrLastEdgeUs == 0) {
    g_isrLastEdgeUs = nowUs;
    return;
  }

  uint32_t dt = nowUs - g_isrLastEdgeUs;
  g_isrLastEdgeUs = nowUs;

  // Reject unreasonably small dt (noise / bounce)
  if (dt >= HALL_MIN_DT_US) {
    g_isrDtUs = dt;
    g_isrHasNewDt = true;
  }
}

// ============================================================================
// Motor driver helpers
// ============================================================================
static void driverEnable(bool en) {
  digitalWrite(PIN_REN, en ? HIGH : LOW);
  digitalWrite(PIN_LEN, en ? HIGH : LOW);
}

// Apply PWM to the motor (one direction only)
static void motorApplyPWM(uint8_t pwm) {
  analogWrite(PIN_RPWM, 0);
  analogWrite(PIN_LPWM, pwm);
}

// ============================================================================
// Control helpers (start/stop, preset start PWM, and reset of controller state)
// ============================================================================
static void resetRPMControl() {
  // Reset governor state
  g_govMode = GOV_CLIMB;
  g_lastClimbMs = 0;
  g_lastHoldMs  = 0;
  g_inDbSinceMs = 0;
  g_holdI = 0.0f;

  // Reset measurement helpers
  g_rpmEma = 0.0f;
  g_lastGoodDtUs = 0;

  // Reset stability window
  g_rpmHist[0] = 0; g_rpmHist[1] = 0; g_rpmHist[2] = 0; g_rpmHist[3] = 0;
  g_rpmHistCount = 0;
  g_rpmHistIdx = 0;
}

// Choose a starting PWM when running.
// We do NOT map states to fixed PWM levels. RPM targets define behavior.
static void applyRPMPreset() {
  if (!g_running) {
    g_pwm_target = 0;
    return;
  }

  // If already spinning, keep current PWM to avoid sudden drops.
  uint8_t start = g_pwm_current;
  if (start == 0) start = PWM_START_SAFE;

  if (start > PWM_MAX_SAFE) start = PWM_MAX_SAFE;
  if (start > 0 && start < PWM_START_SAFE) start = PWM_START_SAFE;

  g_pwm_target = start;
}

// BTN1 toggles this state
static void toggleRun() {
  if (g_running) {
    g_pwm_target = 0;
    g_running = false;

    g_overLimitCount = 0;
    resetRPMControl();
  } else {
    // Prepare the hall ISR state so the first pulse doesn't poison dt logic
    noInterrupts();
    g_isrLastEdgeUs = 0;
    g_isrHasNewDt   = false;
    interrupts();

    g_rpmValue = 0;
    g_rpmDisplayValue = 0;

    g_running = true;
    applyRPMPreset();

    g_overLimitCount = 0;
    resetRPMControl();
  }
}

// ============================================================================
// RPM stability helper
// ============================================================================
static bool rpmIsStable() {
  if (g_rpmHistCount < 4) return false;

  uint16_t lo = 65535;
  uint16_t hi = 0;

  for (uint8_t i = 0; i < 4; i++) {
    uint16_t v = g_rpmHist[i];
    if (v < lo) lo = v;
    if (v > hi) hi = v;
  }

  uint16_t spread = (hi >= lo) ? (uint16_t)(hi - lo) : 65535;
  return (spread <= 80);
}

// ============================================================================
// RPM measurement (hall dt → rpm, with filtering + EMA smoothing)
// ============================================================================
static void updateHallAndRPM() {
  uint32_t nowUs = micros();

  // Check for timeout (no pulses => RPM is 0)
  uint32_t lastEdgeUs;
  noInterrupts();
  lastEdgeUs = g_isrLastEdgeUs;
  interrupts();

  uint32_t timeoutUs = g_running ? RPM_TIMEOUT_US_RUN : RPM_TIMEOUT_US_STOP;

  if ((uint32_t)(nowUs - lastEdgeUs) > timeoutUs) {
    g_rpmValue = 0;
    return;
  }

  // No new dt => no update this loop
  if (!g_isrHasNewDt) return;

  // Read dt atomically and clear the flag
  uint32_t dt;
  noInterrupts();
  dt = g_isrDtUs;
  g_isrHasNewDt = false;
  interrupts();

  // Reject spikes / missed pulses using last good dt
  if (g_lastGoodDtUs > 0) {
    float fdt = (float)dt;
    float flg = (float)g_lastGoodDtUs;

    if (fdt > flg * DT_TOO_BIG_FACTOR) return;
    if (fdt < flg * DT_TOO_SMALL_FACTOR) return;
  }

  g_lastGoodDtUs = dt;

  // Convert dt to RPM: rpm = 60e6 / (dtUs * pulsesPerRev)
  uint32_t denom = (uint32_t)PULSES_PER_REV * dt;
  uint32_t rpmCalc = (denom > 0UL) ? (60000000UL / denom) : 0UL;
  if (rpmCalc > (uint32_t)RPM_DISPLAY_CLAMP) rpmCalc = (uint32_t)RPM_DISPLAY_CLAMP;

  // Store raw sample for stability check
  g_rpmHist[g_rpmHistIdx] = (uint16_t)rpmCalc;
  g_rpmHistIdx++;
  if (g_rpmHistIdx >= 4) g_rpmHistIdx = 0;
  if (g_rpmHistCount < 4) g_rpmHistCount++;

  // EMA smoothing for stable display/control
  if (g_rpmEma <= 0.0f) g_rpmEma = (float)rpmCalc;
  else g_rpmEma = g_rpmEma + RPM_EMA_ALPHA * ((float)rpmCalc - g_rpmEma);

  g_rpmValue = (uint16_t)(g_rpmEma + 0.5f);
}

// ============================================================================
// Governor (closed-loop controller)
// ============================================================================
static void updateGovernor(unsigned long nowMs) {
  if (!g_running) return;
  if (g_rpm_state > 4) return; // allow 0..4

  uint16_t target = RPM_TARGETS[g_rpm_state];

  // If we have no pulses yet (rpm==0), climb gently until hall starts reporting.
  if (g_rpmValue == 0) {
    // State 0 climbs slower and caps PWM to reduce overshoot before the first pulse.
    uint16_t spinDt = (g_rpm_state == 0) ? 1200 : ((target <= 400) ? 800 : CLIMB_DT_MS);
    if ((uint16_t)(nowMs - g_lastClimbMs) < spinDt) return;
    g_lastClimbMs = nowMs;

    int16_t newP = (int16_t)g_pwm_target + 1;

    int16_t maxP = (g_rpm_state == 0) ? (int16_t)PWM_MAX_S0 : (int16_t)PWM_MAX_SAFE;
    if (newP > maxP) newP = maxP;
    if (newP > 0 && newP < (int16_t)PWM_START_SAFE) newP = (int16_t)PWM_START_SAFE;

    g_pwm_target = (uint8_t)newP;
    return;
  }

  // Small UX bias: aim a touch above target so it settles at/just above the displayed target.
  if (g_rpm_state == 1) target += 10;
  if (g_rpm_state == 2) target += 30;
  if (g_rpm_state == 3) target += 40;
  // Note: State 4 uses the exact target by default (no bias found).

  int16_t err = (int16_t)target - (int16_t)g_rpmValue;
  int16_t db  = (target <= 1200) ? DB_S1 : DB_RPM;

  // LOCK mode: freeze PWM unless drift becomes large enough to justify unlocking.
  if (g_govMode == GOV_LOCK) {
    int16_t eabs = err;
    if (eabs < 0) eabs = (int16_t)(-eabs);

    // Unlock faster if we drop below target, keep normal threshold for overshoot.
    int16_t unlockNow = (err > 0) ? 40 : UNLOCK_RPM;
    if (eabs > unlockNow) {
      g_govMode = GOV_HOLD;
      g_lastHoldMs = 0;
      g_holdI *= 0.5f;
      g_inDbSinceMs = 0;
    }
    return;
  }

  // Stability requirement for smart decisions
  bool stable = rpmIsStable();
  // During early ramp-up (before we have 4 samples), allow CLIMB decisions.
  if (g_rpmHistCount < 4) stable = true;

  // Enter LOCK if stable and within deadband for long enough,
  // and only when at/above target (err <= 0).
  {
    int16_t eabs = err;
    if (eabs < 0) eabs = (int16_t)(-eabs);

    if (stable && (err <= 0) && eabs <= db) {
      if (g_inDbSinceMs == 0) g_inDbSinceMs = nowMs;
      if ((unsigned long)(nowMs - g_inDbSinceMs) >= LOCK_TIME_MS) {
        g_govMode = GOV_LOCK;
        g_holdI = 0.0f;
        return;
      }
    } else {
      g_inDbSinceMs = 0;
    }
  }

  // CLIMB mode: move PWM slowly toward the target RPM.
  if (g_govMode == GOV_CLIMB) {
    if ((uint16_t)(nowMs - g_lastClimbMs) < CLIMB_DT_MS) return;
    g_lastClimbMs = nowMs;

    // When near target, switch to HOLD for gentle trimming.
    int16_t holdUp;
    int16_t holdDown;

    if (g_rpm_state == 0) {
      // Much earlier HOLD for 300 RPM target to avoid overshoot
      holdUp = 25;
      holdDown = 80;
    } else {
      holdUp   = (target <= 1200) ? 70  : 260;
      holdDown = (target <= 1200) ? 180 : 180;
    }

    if ((err >= 0 && err <= holdUp) || (err < 0 && err >= (int16_t)(-holdDown))) {
      g_govMode = GOV_HOLD;
      g_lastHoldMs = 0;
      g_holdI = 0.0f;
      return;
    }

    if (!stable) return;

    if (err > 0) {
      // Below target: increase PWM by +1 (or capped for state 0)
      int16_t newP = (int16_t)g_pwm_target + (int16_t)CLIMB_STEP_PWM;

      int16_t maxP = (g_rpm_state == 0) ? (int16_t)PWM_MAX_S0 : (int16_t)PWM_MAX_SAFE;
      if (newP > maxP) newP = maxP;

      if (newP > 0 && newP < (int16_t)PWM_START_SAFE) newP = (int16_t)PWM_START_SAFE;
      g_pwm_target = (uint8_t)newP;
    } else {
      // Above target:
      // For low RPM targets, do not back off here; switch to HOLD instead.
      if (target <= 1200) {
        g_govMode = GOV_HOLD;
        g_lastHoldMs = 0;
        g_holdI = 0.0f;
        return;
      }

      // For higher targets, allow slow back-off by -1 PWM
      int16_t newP = (int16_t)g_pwm_target - (int16_t)CLIMB_STEP_PWM;
      if (newP < 0) newP = 0;
      if (newP > 0 && newP < (int16_t)PWM_START_SAFE) newP = (int16_t)PWM_START_SAFE;
      g_pwm_target = (uint8_t)newP;
    }

    return;
  }

  // HOLD mode: small trimming around target (PI-like behavior).
  if (g_govMode == GOV_HOLD) {
    uint16_t holdDt = (target <= 1200) ? HOLD_DT_S1 : HOLD_DT_MS;
    if ((uint16_t)(nowMs - g_lastHoldMs) < holdDt) return;
    g_lastHoldMs = nowMs;

    if (!stable) return;

    // If we are just slightly above target, relax the integrator and do nothing else.
    if (err <= 0 && err > -db) {
      g_holdI *= 0.85f;
      return;
    }

    // Integrator
    g_holdI += (float)err * HOLD_KI;
    if (g_holdI >  HOLD_I_MAX) g_holdI =  HOLD_I_MAX;
    if (g_holdI < -HOLD_I_MAX) g_holdI = -HOLD_I_MAX;

    // Proportional + integral
    float dF = (float)err * HOLD_KP + g_holdI;

    // Limit trim steps
    float upLim = (target <= 1200) ? 2.0f : 1.0f;
    float dnLim = 1.0f;

    if (dF >  upLim) dF =  upLim;
    if (dF < -dnLim) dF = -dnLim;

    // Convert float correction to integer PWM delta
    int16_t dP = (dF >= 0.0f) ? (int16_t)(dF + 0.5f) : (int16_t)(dF - 0.5f);

    int16_t newP = (int16_t)g_pwm_target + dP;
    if (newP < 0) newP = 0;
    if (newP > (int16_t)PWM_MAX_SAFE) newP = (int16_t)PWM_MAX_SAFE;
    if (newP > 0 && newP < (int16_t)PWM_START_SAFE) newP = (int16_t)PWM_START_SAFE;

    g_pwm_target = (uint8_t)newP;
    return;
  }
}

// ============================================================================
// Display helpers
// ============================================================================
static void displayRPM(uint16_t rpm) {
  if (rpm > RPM_DISPLAY_CLAMP) rpm = RPM_DISPLAY_CLAMP;
  display.showNumberDec((int)rpm, false);
}

static void displayState(uint8_t state) {
  const uint8_t hyphen = SEG_G;
  uint8_t segs[4] = { hyphen, hyphen, hyphen, display.encodeDigit(state % 10) };
  display.setSegments(segs);
}

static void displayStop() {
  const uint8_t S = SEG_A | SEG_F | SEG_G | SEG_C | SEG_D;
  const uint8_t t = SEG_F | SEG_E | SEG_G | SEG_D;
  const uint8_t O = SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F;
  const uint8_t P = SEG_A | SEG_B | SEG_E | SEG_F | SEG_G;
  uint8_t segs[4] = { S, t, O, P };
  display.setSegments(segs);
}

static void showStateFor3s(unsigned long nowMs) {
  g_showingState = true;
  g_stateShowUntilMs = nowMs + 3000UL;
  displayState(g_rpm_state);
}

// Display smoothing (UX only; does not affect control).
// - Running: show measured RPM directly.
// - Stopped: estimate RPM from time since last pulse, but never allow the display to increase.
static void updateRpmDisplay(unsigned long nowMs) {
  (void)nowMs;

  // If showing ---N or STOP, do not interfere
  if (g_showingState) return;

  // Running: display follows measured RPM
  if (g_running) {
    g_rpmDisplayValue = g_rpmValue;
    return;
  }

  // Stopped: estimate from time since last hall pulse
  uint32_t lastEdgeUs;
  noInterrupts();
  lastEdgeUs = g_isrLastEdgeUs;
  interrupts();

  // Boot case: no hall pulse yet
  if (lastEdgeUs == 0) {
    g_rpmDisplayValue = 0;
    return;
  }

  uint32_t nowUs = micros();
  uint32_t sinceUs = nowUs - lastEdgeUs;

  // If no pulse for long enough, call it 0
  const uint32_t OFF_ZERO_US = 500000UL;
  if (sinceUs > OFF_ZERO_US) {
    g_rpmDisplayValue = 0;
    return;
  }

  // rpm_est = 60e6 / (sinceUs * PPR)
  uint32_t denom = (uint32_t)PULSES_PER_REV * sinceUs;
  uint32_t rpmEst = (denom > 0UL) ? (60000000UL / denom) : 0UL;
  if (rpmEst > (uint32_t)RPM_DISPLAY_CLAMP) rpmEst = (uint32_t)RPM_DISPLAY_CLAMP;

  uint16_t est16 = (uint16_t)rpmEst;

  // Monotonic smoothing: never allow displayed RPM to go up while stopped
  if (g_rpmDisplayValue == 0) {
    g_rpmDisplayValue = est16;
    return;
  }

  if (est16 < g_rpmDisplayValue) {
    const uint16_t MAX_DROP_PER_CALL = 60;
    uint16_t drop = (uint16_t)(g_rpmDisplayValue - est16);
    if (drop > MAX_DROP_PER_CALL) drop = MAX_DROP_PER_CALL;
    g_rpmDisplayValue = (uint16_t)(g_rpmDisplayValue - drop);
  }
}

// ============================================================================
// PWM ramping (softly applies g_pwm_target)
// ============================================================================
static void updateRamp(unsigned long nowMs) {
  if ((uint16_t)(nowMs - g_lastRampMs) < RAMP_DT_MS) return;
  g_lastRampMs = nowMs;

  if (g_pwm_current == g_pwm_target) return;

  if (g_pwm_current < g_pwm_target) {
    uint8_t next = (uint8_t)(g_pwm_current + RAMP_STEP);
    g_pwm_current = (next > g_pwm_target) ? g_pwm_target : next;
  } else {
    int16_t next = (int16_t)g_pwm_current - (int16_t)RAMP_STEP;
    g_pwm_current = (next < (int16_t)g_pwm_target) ? g_pwm_target : (uint8_t)next;
  }

  motorApplyPWM(g_pwm_current);
}

// ============================================================================
// Button handling
// ============================================================================
static bool button1Pressed(unsigned long nowMs) {
  bool raw = digitalRead(PIN_BTN1);

  if (raw != g_btn1Last) {
    g_btn1Last = raw;
    g_btn1LastChangeMs = nowMs;
  }

  if ((uint16_t)(nowMs - g_btn1LastChangeMs) >= DEBOUNCE_MS && raw != g_btn1Stable) {
    g_btn1Stable = raw;
    if (g_btn1Stable == LOW) return true; // one event per press
  }
  return false;
}

static void stepNormalStateForward() {
  if (g_rpm_state < 1 || g_rpm_state > 4) {
    g_rpm_state = 1;
  } else {
    g_rpm_state++;
    if (g_rpm_state > 4) g_rpm_state = 1;
  }

  if (g_running) applyRPMPreset();
  resetRPMControl();
}

static void setSafeState0() {
  g_rpm_state = 0;
  if (g_running) applyRPMPreset();
  resetRPMControl();
}

static void handleButton2(unsigned long nowMs) {
  bool raw = digitalRead(PIN_BTN2);

  if (raw != g_btn2Last) {
    g_btn2Last = raw;
    g_btn2LastChangeMs = nowMs;
  }

  if ((uint16_t)(nowMs - g_btn2LastChangeMs) >= DEBOUNCE_MS && raw != g_btn2Stable) {
    g_btn2Stable = raw;

    if (g_btn2Stable == LOW) {
      g_btn2Pressing = true;
      g_btn2PressStartMs = nowMs;
      g_btn2LongHandled = false;
    }

    if (g_btn2Stable == HIGH) {
      if (g_btn2Pressing) {
        g_btn2Pressing = false;

        if (g_btn2LongHandled) return;

        if (!g_showingState) {
          showStateFor3s(nowMs);
        } else {
          stepNormalStateForward();
          showStateFor3s(nowMs);
        }
      }
    }
  }

  if (g_btn2Pressing && !g_btn2LongHandled) {
    if ((uint16_t)(nowMs - g_btn2PressStartMs) >= LONGPRESS_MS) {
      g_btn2LongHandled = true;
      setSafeState0();
      showStateFor3s(nowMs);
    }
  }
}

// ============================================================================
// setup / loop
// ============================================================================
void setup() {
  pinMode(PIN_RPWM, OUTPUT);
  pinMode(PIN_LPWM, OUTPUT);
  pinMode(PIN_REN,  OUTPUT);
  pinMode(PIN_LEN,  OUTPUT);

  pinMode(PIN_BTN1, INPUT_PULLUP);
  pinMode(PIN_BTN2, INPUT_PULLUP);
  pinMode(PIN_HALL, INPUT_PULLUP);

  // Prepare ISR shared state
  g_isrLastEdgeUs = 0;
  g_isrHasNewDt  = false;
  attachInterrupt(digitalPinToInterrupt(PIN_HALL), isrHallFalling, FALLING);

  driverEnable(true);
  motorApplyPWM(0);

  display.setBrightness(0x0f);
  displayRPM(0);
  g_rpmDisplayValue = 0;

  if (SERIAL_ENABLED) {
    serialInit();
  }
}

void loop() {
  unsigned long nowMs = millis();

  if (SERIAL_ENABLED) {
    handleSerial(nowMs);
  }

  // Buttons
  if (button1Pressed(nowMs)) {
    toggleRun();
  }
  handleButton2(nowMs);

  // Measure -> control -> apply
  updateHallAndRPM();
  updateGovernor(nowMs);

  // Safety cutoff: stop if over RPM_LIMIT for several checks
  if (RPM_LIMIT > 0 && g_running) {
    if (g_rpmValue > RPM_LIMIT) {
      if (g_overLimitCount < RPM_TRIP_COUNT) g_overLimitCount++;
    } else {
      g_overLimitCount = 0;
    }

    if (g_overLimitCount >= RPM_TRIP_COUNT) {
      g_pwm_target = 0;
      g_running = false;
      g_overLimitCount = 0;
      resetRPMControl();

      g_showingState = true;
      g_stateShowUntilMs = nowMs + 3000UL;
      displayStop();
    }
  }

  // Apply PWM smoothly
  updateRamp(nowMs);

  // End the ---N/STOP screen after 3 seconds
  if (g_showingState && (long)(nowMs - g_stateShowUntilMs) >= 0) {
    g_showingState = false;
  }

  // Update and draw the display
  updateRpmDisplay(nowMs);

  if (!g_showingState) {
    if (SHOW_TARGET_WHEN_LOCKED && g_running && g_govMode == GOV_LOCK && g_rpm_state <= 4) {
      // Show the exact target value (not the internally biased target)
      displayRPM(RPM_TARGETS[g_rpm_state]);
    } else {
      // Honest mode: show the measured/smoothed value
      displayRPM(g_rpmDisplayValue);
    }
  }
}
