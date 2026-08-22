/*
  LOCOMOTIVE TEST CONTROLLER
  Arduino Nano Every + Dual Sabertooth 2x60 (Analog Voltage, Independent Mode)

  No Dimension Engineering library required -- drives the Sabertooths with a
  real 0-5V analog voltage on S1/S2 (jumpered together at each board's
  terminal block), generated from a filtered + biased PWM output.

  Sabertooth DIP switches (SAME on both boards):
    1 = ON, 2 = ON   -> Analog voltage control mode
    3 = ON           -> Cutoff disabled (power-supply mode). Recommended for
                         two 12V lithium batteries in series with their own
                         internal BMS -- Sabertooth's own lithium cutoff
                         assumes bare series cells and may misjudge a
                         two-12V-pack-in-series configuration. Confirm your
                         batteries have BMS-level low-voltage protection.
    4 = OFF          -> Independent mode (S1 -> Motor 1, S2 -> Motor 2;
                         since S1/S2 are jumpered together at the terminal
                         block, both motors always get the same command)
    5 = ON           -> Linear response
    6 = ON           -> Full 0-5V input range (not 4x-sensitivity mode)

  0V = full reverse, 2.5V = stop, 5V = full forward on S1/S2.

  ---- Failsafes added in this version ----

    1) Graceful stop on direction-change events.
      When direction transitions from Forward/Reverse into Center, or
      rapidly flips Forward<->Reverse, the commanded speed ramps down
      smoothly to zero. Ramp time scales with current speed: full speed
      takes 2.5 seconds, lower speeds take proportionally less time.

  2) Re-arming interlock: whenever the direction switch moves OUT of
     center into Forward or Reverse, the throttle output stays locked at
     zero until the throttle pot is confirmed back at (or very near) its
     zero position, regardless of what direction is selected or what the
     pot is currently reading. Only once the pot has been seen at zero
     does the controller "arm" and start responding to the throttle
     normally. This also applies at power-on: if the direction switch
     happens to already be in Forward/Reverse and the throttle pot is
     not at zero when the board boots, the outputs stay locked at stop
     until the pot is brought to zero -- so an unexpected switch/pot
     position at startup can't cause unexpected motion either.
*/

// ---------- Pin assignments ----------
const uint8_t THROTTLE_PIN = A0;  // 10k throttle pot wiper
const uint8_t DIRECTION_PIN = A1; // direction switch w/ 10k/10k divider
const uint8_t PWM_OUT_PIN = 9;    // -> R1/C1 filter -> jumpered S1/S2
const uint8_t STATUS_LED_PIN = 13;
const uint8_t FORWARD_LED_PIN = 5; // lit = forward, blinking = forward but throttle-locked
const uint8_t REVERSE_LED_PIN = 6; // lit = reverse, blinking = reverse but throttle-locked

// ---------- Direction switch decoding ----------
enum Direction
{
  DIR_REVERSE,
  DIR_NEUTRAL,
  DIR_FORWARD
};

const int DIR_LOW_THRESHOLD = 340;  // below this = switch grounded = reverse
const int DIR_HIGH_THRESHOLD = 680; // above this = switch to +5V = forward

// ---------- Throttle shaping ----------
const int THROTTLE_DEADBAND = 15;     // ignore small pot noise near zero
                                      // when computing running speed
const int THROTTLE_ZERO_RAW_MAX = 20; // raw ADC value (0-1023) at/below
                                      // which the pot counts as "fully
                                      // counterclockwise" for re-arming

// ---------- PWM output ----------
// analogWrite() duty 0-255 maps linearly to the filtered 0-5V output.
const uint8_t PWM_STOP = 128; // ~2.5V = stop

// ---------- Interlock state ----------
// Starts locked so an unexpected switch/pot position at power-on can't
// cause immediate motion -- see failsafe #2 above.
bool throttleLocked = true;

// ---------- Direction-change deceleration ----------
const unsigned long MAX_CENTER_STOP_MS = 2500; // full-speed stop time
Direction previousDir = DIR_NEUTRAL;
bool centerStopRampActive = false;
int centerStopInitialSpeed = 0; // signed speed at ramp start
unsigned long centerStopStartMs = 0;
unsigned long centerStopDurationMs = 0;

// Tracks the most recent commanded speed so a direction change can ramp
// down from the actual current command rather than jumping to zero.
int lastCommandedSpeed = 0;

// ---------- Heartbeat LED ----------
unsigned long lastBlinkMs = 0;
bool ledState = false;
const unsigned long BLINK_INTERVAL_MS = 500;

// ---------- Direction indicator LEDs ----------
unsigned long lastDirBlinkMs = 0;
bool dirLedBlinkState = false;
const unsigned long DIR_BLINK_INTERVAL_MS = 250;

void setup()
{
  pinMode(PWM_OUT_PIN, OUTPUT);
  analogWrite(PWM_OUT_PIN, PWM_STOP); // defined "stop" as early as possible

  pinMode(STATUS_LED_PIN, OUTPUT);

  pinMode(FORWARD_LED_PIN, OUTPUT);
  pinMode(REVERSE_LED_PIN, OUTPUT);
}

void loop()
{
  unsigned long now = millis();
  Direction dir = readDirection();

  // Start a proportional ramp when entering center or when reversing
  // direction quickly (FWD<->REV).
  bool enteredCenter = (dir == DIR_NEUTRAL && previousDir != DIR_NEUTRAL);
  bool flippedDirection = (dir != DIR_NEUTRAL && previousDir != DIR_NEUTRAL && dir != previousDir);
  if (enteredCenter || flippedDirection)
  {
    int startSpeed = lastCommandedSpeed;
    throttleLocked = true;

    if (startSpeed != 0)
    {
      centerStopRampActive = true;
      centerStopInitialSpeed = startSpeed;
      centerStopStartMs = now;
      centerStopDurationMs = (unsigned long)(((unsigned long)abs(startSpeed) * MAX_CENTER_STOP_MS + 63UL) / 127UL);
      if (centerStopDurationMs == 0)
      {
        centerStopDurationMs = 1;
      }
    }
    else
    {
      centerStopRampActive = false;
    }
  }

  // ---- Failsafe #2: re-arm only once the pot is confirmed at zero ----
  if (dir != DIR_NEUTRAL && throttleLocked && throttleAtZero())
  {
    throttleLocked = false;
  }

  int signedSpeed = 0;
  if (centerStopRampActive)
  {
    unsigned long elapsed = now - centerStopStartMs;
    if (elapsed >= centerStopDurationMs)
    {
      signedSpeed = 0;
      centerStopRampActive = false;
    }
    else
    {
      float progress = (float)elapsed / (float)centerStopDurationMs;
      signedSpeed = (int)(centerStopInitialSpeed * (1.0f - progress));
    }
  }
  else if (!throttleLocked)
  {
    signedSpeed = readThrottleSpeed(dir); // -127..127
  }
  // else: stays 0 -- either ramp completed, center-off, or waiting for
  // pot to zero out after a direction change.

  lastCommandedSpeed = signedSpeed;

  uint8_t duty = (uint8_t)constrain(PWM_STOP + signedSpeed, 0, 255);
  analogWrite(PWM_OUT_PIN, duty);

  previousDir = dir;

  updateHeartbeat();
  updateDirectionLeds(dir, throttleLocked, now);

  delay(20); // simple loop pacing
}

// Reads the direction switch via the A1 resistor-divider input and
// returns a clean three-state result.
Direction readDirection()
{
  int v = analogRead(DIRECTION_PIN);
  if (v < DIR_LOW_THRESHOLD)
    return DIR_REVERSE;
  if (v > DIR_HIGH_THRESHOLD)
    return DIR_FORWARD;
  return DIR_NEUTRAL;
}

// True when the throttle pot is at (or very near) its zero/counterclockwise
// end -- used only to decide when it's safe to re-arm after a direction
// change, not for normal speed calculation.
bool throttleAtZero()
{
  return analogRead(THROTTLE_PIN) <= THROTTLE_ZERO_RAW_MAX;
}

// Reads the throttle pot and returns a signed value in -127..127,
// applying the requested direction. Only called once the interlock has
// allowed the throttle to be "live."
int readThrottleSpeed(Direction dir)
{
  int raw = analogRead(THROTTLE_PIN);        // 0-1023
  int magnitude = map(raw, 0, 1023, 0, 127); // 0-127

  if (magnitude < map(THROTTLE_DEADBAND, 0, 1023, 0, 127))
  {
    magnitude = 0;
  }

  switch (dir)
  {
  case DIR_FORWARD:
    return magnitude;
  case DIR_REVERSE:
    return -magnitude;
  case DIR_NEUTRAL:
  default:
    return 0;
  }
}

// Blinks the status LED at a fixed rate to show the controller is alive
// and the main loop hasn't stalled.
void updateHeartbeat()
{
  unsigned long now = millis();
  if (now - lastBlinkMs >= BLINK_INTERVAL_MS)
  {
    lastBlinkMs = now;
    ledState = !ledState;
    digitalWrite(STATUS_LED_PIN, ledState ? HIGH : LOW);
  }
}

// Lights the LED matching the selected direction: solid while that
// direction is actually driving, blinking while locked out by the
// re-arming interlock, dark when the switch is centered.
void updateDirectionLeds(Direction dir, bool locked, unsigned long now)
{
  if (dir == DIR_NEUTRAL)
  {
    digitalWrite(FORWARD_LED_PIN, LOW);
    digitalWrite(REVERSE_LED_PIN, LOW);
    return;
  }

  uint8_t activePin = (dir == DIR_FORWARD) ? FORWARD_LED_PIN : REVERSE_LED_PIN;
  uint8_t inactivePin = (dir == DIR_FORWARD) ? REVERSE_LED_PIN : FORWARD_LED_PIN;
  digitalWrite(inactivePin, LOW);

  if (locked)
  {
    if (now - lastDirBlinkMs >= DIR_BLINK_INTERVAL_MS)
    {
      lastDirBlinkMs = now;
      dirLedBlinkState = !dirLedBlinkState;
    }
    digitalWrite(activePin, dirLedBlinkState ? HIGH : LOW);
  }
  else
  {
    digitalWrite(activePin, HIGH);
  }
}

/*
  Note on stop speed: this sketch now applies a software direction-change
  ramp.
  The Sabertooth's own internal ramping still contributes to the final
  physical deceleration profile. If you need a harder/faster cutoff than
  that (e.g. a true emergency-stop that removes drive power immediately
  rather than ramping down), that requires a hardware addition -- for
  example a relay or MOSFET in the battery line to the Sabertooths,
  driven by an Arduino output whenever throttleLocked is true. Happy to
  design that in if you want it.
*/
