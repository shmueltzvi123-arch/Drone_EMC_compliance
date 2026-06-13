/*
 * Arduino Mega PPM/CPPM Signal Generator for FT722 Flight Controller
 * 
 * This sketch generates a stable 8-channel PPM signal from an Arduino Mega.
 * Manual button control via 8 directional buttons + 1 emergency reset button.
 * 
 * Hardware: Arduino Mega 2560
 * Flight Controller: FT722
 * PPM Output: D44 (5V TTL/CMOS logic level to optical transmitter)
 * 
 * Timer5 Configuration: CTC mode (Clear Timer on Compare)
 *   - Prescaler: 8 (16 MHz / 8 = 2 MHz clock)
 *   - Compare value: 256 counts
 *   - Interrupt period: 256 × 0.5µs = 128µs (reliable, precise)
 * 
 * Safety Features:
 * - Both-button safety: pressing both buttons on same axis keeps axis neutral
 * - Throttle clamping to THROTTLE_TEST_MAX during bench testing
 * - Emergency reset button (D2) restores all channels to safe values
 * - No propellers during testing
 */

// ============================================================================
// CONFIGURATION CONSTANTS
// ============================================================================

// PPM Output Pin (D44 on Arduino Mega - far from button pins)
// D44 outputs native 5V logic (0V to 5V) suitable for optical transmitter
#define PPM_OUTPUT_PIN 44

// Button Input Pins
#define RESET_BUTTON_PIN  2    // Emergency safe reset
#define CH4_DOWN_PIN      4    // Rudder/Yaw decrease
#define CH4_UP_PIN        5    // Rudder/Yaw increase
#define CH2_DOWN_PIN      6    // Pitch decrease
#define CH2_UP_PIN        7    // Pitch increase
#define CH1_DOWN_PIN      8    // Roll decrease
#define CH1_UP_PIN        9    // Roll increase
#define CH3_DOWN_PIN     10    // Throttle decrease
#define CH3_UP_PIN       11    // Throttle increase

// PPM Frame Configuration
#define PPM_FRAME_LENGTH        22500   // Total frame length in microseconds
#define PPM_PULSE_WIDTH           300   // Pulse width in microseconds (constant)
#define PPM_CHANNEL_MIN          1000   // Minimum channel value (µs)
#define PPM_CHANNEL_CENTER       1500   // Center/neutral channel value (µs)
#define PPM_CHANNEL_MAX          2000   // Maximum channel value (µs)
#define PPM_NUM_CHANNELS            8   // Number of channels

// Safety Limits for Testing
#define THROTTLE_TEST_MAX        1300   // Throttle maximum during bench testing
#define THROTTLE_MIN             1000   // Throttle minimum (motor off)
#define CONTROL_RATE_US_PER_SEC    50   // Gradual movement: 50 µs/sec

// Timer Configuration (Timer5 on Arduino Mega - CTC Mode)
// CTC (Clear Timer on Compare) mode: Timer resets at OCR5A value
// Clock: 16 MHz / 8 prescaler = 2 MHz (0.5µs per tick)
// OCR5A = 256 counts × 0.5µs = 128µs interrupt period
#define TIMER5_PRESCALER    8           // Prescaler value for 2MHz clock
#define TIMER5_PERIOD      256          // Compare value (OCR5A)
#define TIMER_TICK_US       128         // Microseconds per timer interrupt (256 × 0.5)

// ============================================================================
// PPM SIGNAL STATE MACHINE
// ============================================================================

// State machine for PPM generation
enum PPMState {
  STATE_PULSE = 0,          // Sending pulse (HIGH, 5V)
  STATE_GAP = 1             // Sending gap/silence (LOW, 0V)
};

volatile struct {
  uint8_t state;            // Current state (pulse or gap)
  uint8_t channel;          // Current channel (0-7)
  uint16_t tick_count;      // Microseconds accumulated in current segment
  uint16_t segment_length;  // Target length of current segment
  uint16_t pulse_remaining; // Remaining frame length for sync gap
  bool output_state;        // Current pin state (HIGH/LOW)
} ppm_state = {STATE_PULSE, 0, 0, PPM_PULSE_WIDTH, 0, true};

// ============================================================================
// CHANNEL VALUES (in microseconds)
// ============================================================================

volatile struct {
  uint16_t ch1;   // Roll/Aileron
  uint16_t ch2;   // Pitch/Elevator
  uint16_t ch3;   // Throttle
  uint16_t ch4;   // Yaw/Rudder
  uint16_t ch5;   // AUX1
  uint16_t ch6;   // AUX2
  uint16_t ch7;   // AUX3
  uint16_t ch8;   // AUX4
} channel_values = {
  PPM_CHANNEL_CENTER,  // ch1
  PPM_CHANNEL_CENTER,  // ch2
  THROTTLE_MIN,        // ch3 (starts at minimum)
  PPM_CHANNEL_CENTER,  // ch4
  THROTTLE_MIN,        // ch5
  THROTTLE_MIN,        // ch6
  THROTTLE_MIN,        // ch7
  THROTTLE_MIN         // ch8
};

// Array version for easier access
const uint16_t *channel_array[PPM_NUM_CHANNELS] = {
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
// CONTROL STATE (Button States and Channel Movement)
// ============================================================================

volatile struct {
  // Button states (read in main loop)
  bool btn_reset;
  bool btn_ch1_down, btn_ch1_up;
  bool btn_ch2_down, btn_ch2_up;
  bool btn_ch3_down, btn_ch3_up;
  bool btn_ch4_down, btn_ch4_up;
  
  // Movement timers (in milliseconds)
  unsigned long last_update_ms;
  
  // Channel movement direction for each axis
  // -1: moving down/negative, 0: neutral, +1: moving up/positive
  int8_t ch1_direction;
  int8_t ch2_direction;
  int8_t ch3_direction;
  int8_t ch4_direction;
} control_state = {
  false, false, false,    // reset, ch1_down, ch1_up
  false, false,           // ch2_down, ch2_up
  false, false,           // ch3_down, ch3_up
  false, false,           // ch4_down, ch4_up
  0,                      // last_update_ms
  0, 0, 0, 0              // directions
};

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  // Initialize serial for debugging
  Serial.begin(115200);
  
  // Configure PPM output pin as OUTPUT
  // D44 outputs 5V logic directly (no level shifter needed for optical transmitter)
  pinMode(PPM_OUTPUT_PIN, OUTPUT);
  digitalWrite(PPM_OUTPUT_PIN, LOW);  // Start LOW (0V)
  ppm_state.output_state = false;
  
  // Configure all button pins as INPUT_PULLUP
  // When button pressed, pin is pulled to GND (reads LOW)
  pinMode(RESET_BUTTON_PIN, INPUT_PULLUP);
  pinMode(CH4_DOWN_PIN, INPUT_PULLUP);
  pinMode(CH4_UP_PIN, INPUT_PULLUP);
  pinMode(CH2_DOWN_PIN, INPUT_PULLUP);
  pinMode(CH2_UP_PIN, INPUT_PULLUP);
  pinMode(CH1_DOWN_PIN, INPUT_PULLUP);
  pinMode(CH1_UP_PIN, INPUT_PULLUP);
  pinMode(CH3_DOWN_PIN, INPUT_PULLUP);
  pinMode(CH3_UP_PIN, INPUT_PULLUP);
  
  // Initialize control state
  control_state.last_update_ms = millis();
  
  // Configure Timer5 for PPM generation (CTC mode)
  setupTimer5();
  
  // Initial status message
  Serial.println("Arduino Mega PPM Generator initialized");
  Serial.println("Timer5: CTC mode, 128µs interrupt period");
  Serial.println("PPM Output: D44 (5V logic level)");
  Serial.println("Waiting for signal to begin...");
  delay(500);
  printStatus();
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  // Read button states
  readButtons();
  
  // Update channel values based on button states and control logic
  updateChannels();
  
  // Print status periodically (every 200ms)
  static unsigned long last_print = 0;
  if (millis() - last_print > 200) {
    last_print = millis();
    printStatus();
  }
}

// ============================================================================
// BUTTON READING
// ============================================================================

void readButtons() {
  // Buttons use INPUT_PULLUP, so LOW = pressed, HIGH = released
  // Invert the logic for clarity
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

void updateChannels() {
  // Handle emergency reset button
  if (control_state.btn_reset) {
    resetAllChannels();
    return;  // Skip normal updates during reset
  }
  
  // Calculate time elapsed since last update
  unsigned long now_ms = millis();
  unsigned long elapsed_ms = now_ms - control_state.last_update_ms;
  
  if (elapsed_ms < 10) {
    return;  // Update only every ~10ms to avoid excessive calculation
  }
  
  control_state.last_update_ms = now_ms;
  float elapsed_sec = elapsed_ms / 1000.0;
  float movement_us = CONTROL_RATE_US_PER_SEC * elapsed_sec;
  
  // Update CH1 (Roll/Aileron)
  updateAxis(&channel_values.ch1, 
             control_state.btn_ch1_down, 
             control_state.btn_ch1_up,
             movement_us,
             PPM_CHANNEL_MIN,
             PPM_CHANNEL_MAX);
  
  // Update CH2 (Pitch/Elevator)
  updateAxis(&channel_values.ch2,
             control_state.btn_ch2_down,
             control_state.btn_ch2_up,
             movement_us,
             PPM_CHANNEL_MIN,
             PPM_CHANNEL_MAX);
  
  // Update CH3 (Throttle) - special handling, no auto-return
  updateThrottle(&channel_values.ch3,
                 control_state.btn_ch3_down,
                 control_state.btn_ch3_up,
                 movement_us);
  
  // Update CH4 (Yaw/Rudder)
  updateAxis(&channel_values.ch4,
             control_state.btn_ch4_down,
             control_state.btn_ch4_up,
             movement_us,
             PPM_CHANNEL_MIN,
             PPM_CHANNEL_MAX);
}

// ============================================================================
// AXIS UPDATE FUNCTION (with both-button safety and auto-return)
// ============================================================================

void updateAxis(volatile uint16_t *channel,
                bool btn_down,
                bool btn_up,
                float movement_us,
                uint16_t min_val,
                uint16_t max_val) {
  
  // Determine direction: explicit both-button safety logic
  int8_t direction = 0;
  
  if (btn_down && btn_up) {
    // Both pressed: stay neutral (direction = 0)
    direction = 0;
  } else if (btn_down) {
    // Only down pressed: decrease
    direction = -1;
  } else if (btn_up) {
    // Only up pressed: increase
    direction = +1;
  } else {
    // Neither pressed: return to center
    // Gradual return, not instant
    if (*channel > PPM_CHANNEL_CENTER + 5) {
      direction = -1;  // Move toward center
    } else if (*channel < PPM_CHANNEL_CENTER - 5) {
      direction = +1;  // Move toward center
    } else {
      *channel = PPM_CHANNEL_CENTER;  // Snap to center when very close
      return;
    }
  }
  
  // Apply movement
  if (direction != 0) {
    float new_value = *channel + (direction * movement_us);
    
    // Clamp to valid range
    if (new_value < min_val) {
      *channel = min_val;
    } else if (new_value > max_val) {
      *channel = max_val;
    } else {
      *channel = (uint16_t)new_value;
    }
  }
}

// ============================================================================
// THROTTLE UPDATE FUNCTION (no auto-return, clamped to THROTTLE_TEST_MAX)
// ============================================================================

void updateThrottle(volatile uint16_t *throttle,
                    bool btn_down,
                    bool btn_up,
                    float movement_us) {
  
  // Determine direction: explicit both-button safety logic
  int8_t direction = 0;
  
  if (btn_down && btn_up) {
    // Both pressed: don't move (direction = 0)
    direction = 0;
  } else if (btn_down) {
    // Only down pressed: decrease throttle
    direction = -1;
  } else if (btn_up) {
    // Only up pressed: increase throttle
    direction = +1;
  }
  
  // Apply movement
  if (direction != 0) {
    float new_value = *throttle + (direction * movement_us);
    
    // Clamp to safe range for bench testing
    if (new_value < THROTTLE_MIN) {
      *throttle = THROTTLE_MIN;
    } else if (new_value > THROTTLE_TEST_MAX) {
      *throttle = THROTTLE_TEST_MAX;
    } else {
      *throttle = (uint16_t)new_value;
    }
  }
  
  // Throttle does NOT auto-return when button is released
  // It holds its last value until changed by button input or reset
}

// ============================================================================
// EMERGENCY RESET
// ============================================================================

void resetAllChannels() {
  channel_values.ch1 = PPM_CHANNEL_CENTER;   // Roll
  channel_values.ch2 = PPM_CHANNEL_CENTER;   // Pitch
  channel_values.ch3 = THROTTLE_MIN;         // Throttle to zero
  channel_values.ch4 = PPM_CHANNEL_CENTER;   // Yaw
  channel_values.ch5 = THROTTLE_MIN;         // AUX
  channel_values.ch6 = THROTTLE_MIN;         // AUX
  channel_values.ch7 = THROTTLE_MIN;         // AUX
  channel_values.ch8 = THROTTLE_MIN;         // AUX
  
  Serial.println("!!! EMERGENCY RESET !!!");
}

// ============================================================================
// TIMER5 SETUP (16-bit timer, CTC mode)
// ============================================================================
// 
// Timer5 Configuration:
//   - Mode: CTC (Clear Timer on Compare) - Timer resets when TCNT5 == OCR5A
//   - WGM5[2:0] = 100 (binary) = mode 4
//   - Prescaler: 8 (CS5[2:0] = 010)
//   - Clock: 16 MHz / 8 = 2 MHz
//   - Tick period: 1 / 2 MHz = 0.5 µs
//   - OCR5A: 256 counts
//   - Interrupt period: 256 × 0.5 µs = 128 µs
//   - Interrupt rate: 1 / 128 µs ≈ 7.8 kHz
//
// Timer resets automatically after reaching OCR5A, providing reliable,
// precise interrupt timing without manual overflow handling.

void setupTimer5() {
  // Disable interrupts temporarily
  cli();
  
  // Clear the timer control registers
  TCCR5A = 0;
  TCCR5B = 0;
  TCNT5 = 0;
  
  // ========================================================================
  // Set Timer5 to CTC mode (Clear Timer on Compare)
  // ========================================================================
  // WGM5[2:0] = 100 (mode 4):
  //   WGM52 (TCCR5B bit 3) = 1
  //   WGM51 (TCCR5A bit 1) = 0
  //   WGM50 (TCCR5A bit 0) = 0
  // 
  // This causes the timer to:
  //   1. Count from 0 to OCR5A (256)
  //   2. Generate a compare match interrupt
  //   3. Automatically reset to 0 and repeat
  //
  // Result: Interrupt fires reliably every 256 counts = 128 µs
  
  TCCR5B |= (1 << WGM52);  // Set WGM52 (bit 3 in TCCR5B)
                            // WGM51:50 remain 0 (bits in TCCR5A stay 0)
  
  // ========================================================================
  // Set Prescaler = 8
  // ========================================================================
  // CS5[2:0] = 010:
  //   CS52 (TCCR5B bit 2) = 0
  //   CS51 (TCCR5B bit 1) = 1
  //   CS50 (TCCR5B bit 0) = 0
  // 
  // Clock source: 16 MHz / 8 = 2 MHz
  
  TCCR5B |= (1 << CS51);   // Set CS51 (bit 1 in TCCR5B)
                            // CS52 and CS50 remain 0
  
  // ========================================================================
  // Set Compare Value (TOP = OCR5A)
  // ========================================================================
  // Timer counts: 0, 1, 2, ..., 254, 255, [256 → INTERRUPT & RESET]
  // Period = 256 × (1 / 2 MHz) = 256 × 0.5 µs = 128 µs
  
  OCR5A = TIMER5_PERIOD;   // 256 counts
  
  // ========================================================================
  // Enable Timer5 Compare A Interrupt
  // ========================================================================
  TIMSK5 |= (1 << OCIE5A);  // Output Compare Interrupt Enable A
  
  // Re-enable interrupts
  sei();
  
  // Print configuration
  Serial.print("Timer5 CTC Configuration:");
  Serial.print(" WGM=100, Prescaler=8, OCR5A=");
  Serial.print(TIMER5_PERIOD);
  Serial.print(" → Interrupt every ");
  Serial.print(TIMER_TICK_US);
  Serial.println(" µs");
}

// ============================================================================
// TIMER5 INTERRUPT SERVICE ROUTINE (ISR)
// ============================================================================
// Fires every 128 microseconds (guaranteed in CTC mode)
// Generates the PPM signal by toggling D44 and managing the state machine

ISR(TIMER5_COMPA_vect) {
  // Accumulate time in current segment
  ppm_state.tick_count += TIMER_TICK_US;
  
  // Check if we need to transition to next segment
  if (ppm_state.tick_count >= ppm_state.segment_length) {
    ppm_state.tick_count -= ppm_state.segment_length;
    
    // Transition: pulse -> gap -> pulse -> gap, etc.
    if (ppm_state.state == STATE_PULSE) {
      // Just finished pulse, now enter gap
      ppm_state.state = STATE_GAP;
      ppm_state.output_state = false;
      digitalWrite(PPM_OUTPUT_PIN, LOW);  // 0V (gap/silence)
      
      // Calculate gap length = channel_value - pulse_width
      uint16_t channel_val = *channel_array[ppm_state.channel];
      ppm_state.segment_length = channel_val - PPM_PULSE_WIDTH;
      ppm_state.pulse_remaining -= channel_val;
      
    } else {
      // Just finished gap, move to next channel
      ppm_state.channel++;
      
      if (ppm_state.channel >= PPM_NUM_CHANNELS) {
        // Finished all channels, now do sync gap
        // Sync gap fills the rest of the frame
        ppm_state.segment_length = ppm_state.pulse_remaining;
        ppm_state.state = STATE_PULSE;  // After sync, we start pulse again
        ppm_state.output_state = false;
        digitalWrite(PPM_OUTPUT_PIN, LOW);  // 0V (sync gap)
        ppm_state.pulse_remaining = 0;
        
        // Check if we're done with sync gap
        if (ppm_state.tick_count >= ppm_state.segment_length) {
          // Frame complete, reset to first channel
          ppm_state.channel = 0;
          ppm_state.state = STATE_PULSE;
          ppm_state.segment_length = PPM_PULSE_WIDTH;
          ppm_state.pulse_remaining = PPM_FRAME_LENGTH;
          ppm_state.tick_count = 0;
          ppm_state.output_state = true;
          digitalWrite(PPM_OUTPUT_PIN, HIGH);  // 5V (pulse start)
        }
      } else {
        // Start pulse for next channel
        ppm_state.state = STATE_PULSE;
        ppm_state.output_state = true;
        digitalWrite(PPM_OUTPUT_PIN, HIGH);  // 5V (pulse)
        ppm_state.segment_length = PPM_PULSE_WIDTH;
      }
    }
  }
}

// ============================================================================
// DEBUGGING / STATUS OUTPUT
// ============================================================================

void printStatus() {
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
  Serial.print(control_state.btn_ch1_up ? "1U" : "--");
  Serial.print(control_state.btn_ch2_down ? "2D" : "--");
  Serial.print(control_state.btn_ch2_up ? "2U" : "--");
  Serial.print(control_state.btn_ch3_down ? "3D" : "--");
  Serial.print(control_state.btn_ch3_up ? "3U" : "--");
  Serial.print(control_state.btn_ch4_down ? "4D" : "--");
  Serial.print(control_state.btn_ch4_up ? "4U" : "--");
  Serial.print(control_state.btn_reset ? " [RESET]" : "");
  Serial.println();
}
