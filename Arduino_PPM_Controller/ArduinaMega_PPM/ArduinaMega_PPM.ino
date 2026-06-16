/*
 * Arduino Mega PPM/CPPM Signal Generator for FT722 Flight Controller
 *
 * This sketch generates a stable 8-channel PPM/CPPM signal using Arduino Mega 2560.
 * The signal is generated on D44 as a 5V logic output.
 *
 * Main purpose:
 * - Send 8 RC channels to a flight controller using one PPM signal wire.
 * - Control Roll, Pitch, Throttle, and Yaw using push buttons.
 * - Keep AUX channels at safe default values.
 *
 * Hardware:
 * - Arduino Mega 2560
 * - FT722 flight controller or any flight controller with PPM/CPPM input
 * - PPM output pin: D44
 *
 * Safety:
 * - Remove propellers during testing.
 * - Throttle is limited to THROTTLE_TEST_MAX during bench testing.
 * - Emergency reset button restores safe channel values.
 * - Opposite buttons on the same axis cancel each other.
 *
 * PPM signal structure:
 * - Each channel is represented by:
 *     fixed HIGH pulse + variable LOW gap
 *
 * Example:
 * - Channel value = 1500us
 * - Pulse width   = 300us
 * - LOW gap       = 1500 - 300 = 1200us
 *
 * Full frame:
 * - CH1 pulse + gap
 * - CH2 pulse + gap
 * - ...
 * - CH8 pulse + gap
 * - Long LOW sync gap until total frame length reaches 22500us
 */

#include <Arduino.h>
#include <avr/interrupt.h>

// ============================================================================
// CONFIGURATION CONSTANTS
// ============================================================================

// PPM Output Pin
// D44 outputs native 5V logic on Arduino Mega.
#define PPM_OUTPUT_PIN 44

// Button Input Pins
// Buttons should connect the input pin to GND when pressed.
// INPUT_PULLUP is used, so pressed = LOW.
#define RESET_BUTTON_PIN  2    // Emergency safe reset
#define CH4_DOWN_PIN      4    // Yaw decrease
#define CH4_UP_PIN        5    // Yaw increase
#define CH2_DOWN_PIN      6    // Pitch decrease
#define CH2_UP_PIN        7    // Pitch increase
#define CH1_DOWN_PIN      8    // Roll decrease
#define CH1_UP_PIN        9    // Roll increase
#define CH3_DOWN_PIN     10    // Throttle decrease
#define CH3_UP_PIN       11    // Throttle increase

// PPM Frame Configuration
#define PPM_FRAME_LENGTH        22500   // Total frame length in microseconds
#define PPM_PULSE_WIDTH           300   // Fixed HIGH pulse width in microseconds
#define PPM_CHANNEL_MIN          1000   // Minimum channel value in microseconds
#define PPM_CHANNEL_CENTER       1500   // Center / neutral channel value
#define PPM_CHANNEL_MAX          2000   // Maximum channel value
#define PPM_NUM_CHANNELS            8   // Number of PPM channels

// Safety Limits for Testing
#define THROTTLE_TEST_MAX        1300   // Maximum throttle during bench testing
#define THROTTLE_MIN             1000   // Minimum throttle / motor off
#define CONTROL_RATE_US_PER_SEC    50   // Gradual movement rate: 50us per second

// ============================================================================
// TIMER5 CONFIGURATION
// ============================================================================
//
// Arduino Mega 2560 clock: 16MHz
// Timer5 prescaler: 8
// Timer clock: 16MHz / 8 = 2MHz
//
// Therefore:
// - 1 timer count = 0.5us
// - 1us = 2 timer counts
//
// Timer5 is used in CTC mode.
// OCR5A is updated dynamically every interrupt.
// This means the timer waits exactly for the next PPM segment length.
//
// Example:
// - 300us pulse  -> OCR5A = 600 counts
// - 1200us gap   -> OCR5A = 2400 counts
// - 1700us gap   -> OCR5A = 3400 counts
//

#define TIMER5_PRESCALER        8
#define TIMER_COUNTS_PER_US     2

static inline uint16_t usToTimerCounts(uint16_t us)
{
  return us * TIMER_COUNTS_PER_US;
}

// ============================================================================
// PPM SIGNAL STATE MACHINE
// ============================================================================
//
// STATE_PULSE:
//   The output is HIGH for the fixed pulse width, usually 300us.
//
// STATE_GAP:
//   The output is LOW for the rest of the channel time.
//   The LOW gap length depends on the channel value.
//
// STATE_SYNC:
//   The output remains LOW for the long sync gap at the end of the frame.
//

enum PPMState {
  STATE_PULSE = 0,
  STATE_GAP   = 1,
  STATE_SYNC  = 2
};

volatile struct {
  uint8_t state;          // Current PPM state: pulse, gap, or sync
  uint8_t channel;        // Current channel index: 0 to 7
  bool output_state;      // Current output state: HIGH or LOW
} ppm_state = {STATE_PULSE, 0, true};

// ============================================================================
// CHANNEL VALUES
// ============================================================================
//
// Channel order used here:
//
// CH1 = Roll
// CH2 = Pitch
// CH3 = Throttle
// CH4 = Yaw
// CH5 = AUX1
// CH6 = AUX2
// CH7 = AUX3
// CH8 = AUX4
//
// Betaflight common channel map:
// AETR1234
//
// This means:
// CH1 Roll
// CH2 Pitch
// CH3 Throttle
// CH4 Yaw
// CH5 AUX1
// CH6 AUX2
// CH7 AUX3
// CH8 AUX4
//

volatile struct {
  uint16_t ch1;   // Roll / Aileron
  uint16_t ch2;   // Pitch / Elevator
  uint16_t ch3;   // Throttle
  uint16_t ch4;   // Yaw / Rudder
  uint16_t ch5;   // AUX1
  uint16_t ch6;   // AUX2
  uint16_t ch7;   // AUX3
  uint16_t ch8;   // AUX4
} channel_values = {
  PPM_CHANNEL_CENTER,  // CH1 Roll
  PPM_CHANNEL_CENTER,  // CH2 Pitch
  THROTTLE_MIN,        // CH3 Throttle starts low
  PPM_CHANNEL_CENTER,  // CH4 Yaw
  THROTTLE_MIN,        // CH5 AUX1
  THROTTLE_MIN,        // CH6 AUX2
  THROTTLE_MIN,        // CH7 AUX3
  THROTTLE_MIN         // CH8 AUX4
};

// Pointer array for easier channel access inside the ISR.
volatile uint16_t *channel_array[PPM_NUM_CHANNELS] = {
  &channel_values.ch1,
  &channel_values.ch2,
  &channel_values.ch3,
  &channel_values.ch4,
  &channel_values.ch5,
  &channel_values.ch6,
  &channel_values.ch7,
  &channel_values.ch8
};

// ============================================================================
// CONTROL STATE
// ============================================================================
//
// Button states are read in the main loop.
// Channel values are updated gradually based on the pressed buttons.
//

volatile struct {
  bool btn_reset;

  bool btn_ch1_down;
  bool btn_ch1_up;

  bool btn_ch2_down;
  bool btn_ch2_up;

  bool btn_ch3_down;
  bool btn_ch3_up;

  bool btn_ch4_down;
  bool btn_ch4_up;

  unsigned long last_update_ms;

  int8_t ch1_direction;
  int8_t ch2_direction;
  int8_t ch3_direction;
  int8_t ch4_direction;
} control_state = {
  false,
  false, false,
  false, false,
  false, false,
  false, false,
  0,
  0, 0, 0, 0
};

// ============================================================================
// SETUP
// ============================================================================

void setup()
{
  Serial.begin(115200);

  // Configure PPM output pin.
  pinMode(PPM_OUTPUT_PIN, OUTPUT);

  // Start with a HIGH pulse.
  // The first Timer5 interrupt will occur after PPM_PULSE_WIDTH.
  digitalWrite(PPM_OUTPUT_PIN, HIGH);
  ppm_state.output_state = true;

  // Configure button pins.
  pinMode(RESET_BUTTON_PIN, INPUT_PULLUP);
  pinMode(CH4_DOWN_PIN, INPUT_PULLUP);
  pinMode(CH4_UP_PIN, INPUT_PULLUP);
  pinMode(CH2_DOWN_PIN, INPUT_PULLUP);
  pinMode(CH2_UP_PIN, INPUT_PULLUP);
  pinMode(CH1_DOWN_PIN, INPUT_PULLUP);
  pinMode(CH1_UP_PIN, INPUT_PULLUP);
  pinMode(CH3_DOWN_PIN, INPUT_PULLUP);
  pinMode(CH3_UP_PIN, INPUT_PULLUP);

  control_state.last_update_ms = millis();

  setupTimer5();

  Serial.println("Arduino Mega PPM Generator initialized");
  Serial.println("Timer5: CTC mode, dynamic OCR5A timing");
  Serial.println("PPM Output: D44, 5V logic level");
  Serial.println("Remove propellers during testing");
  printStatus();
}

// ============================================================================
// MAIN LOOP
// ============================================================================
//
// The main loop reads the buttons and updates the channel values.
// The actual PPM signal timing is generated independently by Timer5 ISR.
//

void loop()
{
  readButtons();
  updateChannels();

  static unsigned long last_print = 0;

  if (millis() - last_print > 200)
  {
    last_print = millis();
    printStatus();
  }
}

// ============================================================================
// BUTTON READING
// ============================================================================

void readButtons()
{
  // INPUT_PULLUP logic:
  // HIGH = released
  // LOW  = pressed

  control_state.btn_reset    = (digitalRead(RESET_BUTTON_PIN) == LOW);

  control_state.btn_ch1_down = (digitalRead(CH1_DOWN_PIN) == LOW);
  control_state.btn_ch1_up   = (digitalRead(CH1_UP_PIN) == LOW);

  control_state.btn_ch2_down = (digitalRead(CH2_DOWN_PIN) == LOW);
  control_state.btn_ch2_up   = (digitalRead(CH2_UP_PIN) == LOW);

  control_state.btn_ch3_down = (digitalRead(CH3_DOWN_PIN) == LOW);
  control_state.btn_ch3_up   = (digitalRead(CH3_UP_PIN) == LOW);

  control_state.btn_ch4_down = (digitalRead(CH4_DOWN_PIN) == LOW);
  control_state.btn_ch4_up   = (digitalRead(CH4_UP_PIN) == LOW);
}

// ============================================================================
// CHANNEL UPDATES
// ============================================================================
//
// Channels are updated gradually according to button input.
// Roll, Pitch, and Yaw return to center when released.
// Throttle holds its last value and does not auto-return.
//

void updateChannels()
{
  if (control_state.btn_reset)
  {
    resetAllChannels();
    return;
  }

  unsigned long now_ms = millis();
  unsigned long elapsed_ms = now_ms - control_state.last_update_ms;

  // Update every ~10ms.
  if (elapsed_ms < 10)
  {
    return;
  }

  control_state.last_update_ms = now_ms;

  float elapsed_sec = elapsed_ms / 1000.0;
  float movement_us = CONTROL_RATE_US_PER_SEC * elapsed_sec;

  // CH1 Roll
  updateAxis(&channel_values.ch1,
             control_state.btn_ch1_down,
             control_state.btn_ch1_up,
             movement_us,
             PPM_CHANNEL_MIN,
             PPM_CHANNEL_MAX);

  // CH2 Pitch
  updateAxis(&channel_values.ch2,
             control_state.btn_ch2_down,
             control_state.btn_ch2_up,
             movement_us,
             PPM_CHANNEL_MIN,
             PPM_CHANNEL_MAX);

  // CH3 Throttle
  updateThrottle(&channel_values.ch3,
                 control_state.btn_ch3_down,
                 control_state.btn_ch3_up,
                 movement_us);

  // CH4 Yaw
  updateAxis(&channel_values.ch4,
             control_state.btn_ch4_down,
             control_state.btn_ch4_up,
             movement_us,
             PPM_CHANNEL_MIN,
             PPM_CHANNEL_MAX);
}

// ============================================================================
// AXIS UPDATE FUNCTION
// ============================================================================
//
// Used for Roll, Pitch, and Yaw.
//
// Behavior:
// - Only down pressed: move channel down.
// - Only up pressed: move channel up.
// - Both pressed: hold neutral direction.
// - Neither pressed: gradually return to center.
//

void updateAxis(volatile uint16_t *channel,
                bool btn_down,
                bool btn_up,
                float movement_us,
                uint16_t min_val,
                uint16_t max_val)
{
  int8_t direction = 0;

  if (btn_down && btn_up)
  {
    direction = 0;
  }
  else if (btn_down)
  {
    direction = -1;
  }
  else if (btn_up)
  {
    direction = +1;
  }
  else
  {
    // Auto-return to center.
    if (*channel > PPM_CHANNEL_CENTER + 5)
    {
      direction = -1;
    }
    else if (*channel < PPM_CHANNEL_CENTER - 5)
    {
      direction = +1;
    }
    else
    {
      *channel = PPM_CHANNEL_CENTER;
      return;
    }
  }

  if (direction != 0)
  {
    float new_value = *channel + (direction * movement_us);

    if (new_value < min_val)
    {
      *channel = min_val;
    }
    else if (new_value > max_val)
    {
      *channel = max_val;
    }
    else
    {
      *channel = (uint16_t)new_value;
    }
  }
}

// ============================================================================
// THROTTLE UPDATE FUNCTION
// ============================================================================
//
// Throttle does not auto-return.
// It holds its value until changed by button input or emergency reset.
//
// During bench testing, throttle is limited to THROTTLE_TEST_MAX.
//

void updateThrottle(volatile uint16_t *throttle,
                    bool btn_down,
                    bool btn_up,
                    float movement_us)
{
  int8_t direction = 0;

  if (btn_down && btn_up)
  {
    direction = 0;
  }
  else if (btn_down)
  {
    direction = -1;
  }
  else if (btn_up)
  {
    direction = +1;
  }

  if (direction != 0)
  {
    float new_value = *throttle + (direction * movement_us);

    if (new_value < THROTTLE_MIN)
    {
      *throttle = THROTTLE_MIN;
    }
    else if (new_value > THROTTLE_TEST_MAX)
    {
      *throttle = THROTTLE_TEST_MAX;
    }
    else
    {
      *throttle = (uint16_t)new_value;
    }
  }
}

// ============================================================================
// EMERGENCY RESET
// ============================================================================
//
// Restores all channels to safe values.
// Throttle returns to minimum.
// Roll, Pitch, and Yaw return to center.
//

void resetAllChannels()
{
  channel_values.ch1 = PPM_CHANNEL_CENTER;
  channel_values.ch2 = PPM_CHANNEL_CENTER;
  channel_values.ch3 = THROTTLE_MIN;
  channel_values.ch4 = PPM_CHANNEL_CENTER;

  channel_values.ch5 = THROTTLE_MIN;
  channel_values.ch6 = THROTTLE_MIN;
  channel_values.ch7 = THROTTLE_MIN;
  channel_values.ch8 = THROTTLE_MIN;

  Serial.println("!!! EMERGENCY RESET !!!");
}

// ============================================================================
// TIMER5 SETUP
// ============================================================================
//
// Timer5 is configured in CTC mode.
// The timer resets automatically when TCNT5 reaches OCR5A.
//
// Prescaler:
// - CS51 = 1
// - CS52 = 0
// - CS50 = 0
// This gives prescaler 8.
//
// Timer clock:
// - 16MHz / 8 = 2MHz
// - 1 count = 0.5us
//
// OCR5A is not fixed.
// It is updated dynamically in the ISR according to the next PPM segment.
//

void setupTimer5()
{
  cli();

  TCCR5A = 0;
  TCCR5B = 0;
  TCNT5  = 0;

  // CTC mode using OCR5A as TOP.
  TCCR5B |= (1 << WGM52);

  // Prescaler = 8.
  TCCR5B |= (1 << CS51);

  // First interrupt after the first fixed 300us HIGH pulse.
  OCR5A = usToTimerCounts(PPM_PULSE_WIDTH);

  // Enable Timer5 Compare Match A interrupt.
  TIMSK5 |= (1 << OCIE5A);

  sei();

  Serial.println("Timer5 CTC Configuration: dynamic OCR5A timing");
  Serial.println("Prescaler=8, Timer clock=2MHz, 1us=2 counts");
}

// ============================================================================
// TIMER5 INTERRUPT SERVICE ROUTINE
// ============================================================================
//
// Dynamic PPM timing:
//
// Each interrupt means:
// "The current PPM segment has ended."
//
// The ISR then sets OCR5A to the length of the next segment.
//
// Frame structure:
// - CH1 pulse + gap
// - CH2 pulse + gap
// - ...
// - CH8 pulse + gap
// - SYNC gap
//
// Each channel value includes the pulse.
//
// Example:
// - Channel = 1500us
// - Pulse   = 300us
// - Gap     = 1200us
//

ISR(TIMER5_COMPA_vect)
{
  // Total time already used by all channel values in this frame.
  // Used to calculate the sync gap after CH8.
  // This static variable is local to the ISR and is only accessed from the ISR,
  // so there is no race condition (the ISR is atomic relative to the main loop).
  static uint32_t frameUsedUs = 0;

  switch (ppm_state.state)
  {
    // ------------------------------------------------------------------------
    // Finished a 300us HIGH pulse.
    // Now go LOW and wait for the channel gap.
    // ------------------------------------------------------------------------
    case STATE_PULSE:
    {
      digitalWrite(PPM_OUTPUT_PIN, LOW);
      ppm_state.output_state = false;

      uint16_t channel_val = *channel_array[ppm_state.channel];

      if (channel_val < PPM_CHANNEL_MIN) channel_val = PPM_CHANNEL_MIN;
      if (channel_val > PPM_CHANNEL_MAX) channel_val = PPM_CHANNEL_MAX;

      uint16_t gap_us = channel_val - PPM_PULSE_WIDTH;

      frameUsedUs += channel_val;

      OCR5A = usToTimerCounts(gap_us);

      ppm_state.state = STATE_GAP;
      break;
    }

    // ------------------------------------------------------------------------
    // Finished a channel LOW gap.
    // Start the next channel pulse, or start sync after CH8.
    // ------------------------------------------------------------------------
    case STATE_GAP:
    {
      ppm_state.channel++;

      if (ppm_state.channel < PPM_NUM_CHANNELS)
      {
        digitalWrite(PPM_OUTPUT_PIN, HIGH);
        ppm_state.output_state = true;

        OCR5A = usToTimerCounts(PPM_PULSE_WIDTH);

        ppm_state.state = STATE_PULSE;
      }
      else
      {
        digitalWrite(PPM_OUTPUT_PIN, LOW);
        ppm_state.output_state = false;

        uint16_t sync_us;

        if (frameUsedUs < PPM_FRAME_LENGTH)
        {
          sync_us = PPM_FRAME_LENGTH - frameUsedUs;
        }
        else
        {
          // Safety fallback.
          // This should not happen when all channel values are valid.
          sync_us = 5000;
        }

        OCR5A = usToTimerCounts(sync_us);

        ppm_state.state = STATE_SYNC;
      }

      break;
    }

    // ------------------------------------------------------------------------
    // Finished sync gap.
    // Start a new frame from CH1.
    // ------------------------------------------------------------------------
    case STATE_SYNC:
    {
      ppm_state.channel = 0;
      frameUsedUs = 0;

      digitalWrite(PPM_OUTPUT_PIN, HIGH);
      ppm_state.output_state = true;

      OCR5A = usToTimerCounts(PPM_PULSE_WIDTH);

      ppm_state.state = STATE_PULSE;
      break;
    }

    // ------------------------------------------------------------------------
    // Safety recovery if state somehow becomes invalid.
    // ------------------------------------------------------------------------
    default:
    {
      ppm_state.channel = 0;
      frameUsedUs = 0;

      digitalWrite(PPM_OUTPUT_PIN, HIGH);
      ppm_state.output_state = true;

      OCR5A = usToTimerCounts(PPM_PULSE_WIDTH);

      ppm_state.state = STATE_PULSE;
      break;
    }
  }
}

// ============================================================================
// DEBUGGING / STATUS OUTPUT
// ============================================================================
//
// Prints the first four main flight channels and active button states.
//

void printStatus()
{
  Serial.print("CH: ");
  Serial.print(channel_values.ch1);
  Serial.print(" ");
  Serial.print(channel_values.ch2);
  Serial.print(" ");
  Serial.print(channel_values.ch3);
  Serial.print(" ");
  Serial.print(channel_values.ch4);

  Serial.print(" | BTN: ");
  Serial.print(control_state.btn_ch1_down ? "1D" : "--");
  Serial.print(control_state.btn_ch1_up   ? "1U" : "--");
  Serial.print(control_state.btn_ch2_down ? "2D" : "--");
  Serial.print(control_state.btn_ch2_up   ? "2U" : "--");
  Serial.print(control_state.btn_ch3_down ? "3D" : "--");
  Serial.print(control_state.btn_ch3_up   ? "3U" : "--");
  Serial.print(control_state.btn_ch4_down ? "4D" : "--");
  Serial.print(control_state.btn_ch4_up   ? "4U" : "--");
  Serial.print(control_state.btn_reset    ? " [RESET]" : "");

  Serial.println();
}