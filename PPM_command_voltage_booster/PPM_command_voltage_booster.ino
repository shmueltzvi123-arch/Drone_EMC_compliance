/*
  Azrieli College of Engineering - EMC/EMI Drone Project
  Ground Station Signal Conditioning Suite (PPM Passthrough via Interrupts)
  
  Description:
  Captures raw PPM stream from RadioMaster TX12 (Pin D2) using hardware interrupts,
  measures timing parameters, and outputs a clean, stabilized 5V TTL square wave 
  on Pin D3 to drive the Broadcom HFBR-1521Z Optical Transmitter.
*/

// Pin Definitions
const byte PPM_INPUT_PIN  = 2;  // Hardware Interrupt 0 on Uno
const byte PPM_OUTPUT_PIN = 3;  // Digital output to HFBR-1521Z Transmitter

// Volatile variables used within the ISR (Interrupt Service Routine)
volatile unsigned long last_edge_time = 0;
volatile unsigned long pulse_duration = 0;
volatile boolean new_pulse_detected   = false;

void setup() {
  // Initialize digital pins
  pinMode(PPM_INPUT_PIN, INPUT_PULLUP); // Safe floating protection for DSC port
  pinMode(PPM_OUTPUT_PIN, OUTPUT);
  digitalWrite(PPM_OUTPUT_PIN, LOW);    // Initialize output state

  // Attach interrupt to Pin 2 on RISING edge (Start of a PPM pulse)
  attachInterrupt(digitalPinToInterrupt(PPM_INPUT_PIN), ppmISR, RISING);
}

void loop() {
  // Main execution logic
  if (new_pulse_detected) {
    // Basic signal qualification can be performed here if needed for analysis
    new_pulse_detected = false; 
  }
}

/**
 * Interrupt Service Routine (ISR)
 * Executed instantly upon every rising edge of the incoming PPM signal.
 * Ensures minimum latency and prevents clock jitter from affecting packet delivery.
 */
void ppmISR() {
  unsigned long current_time = micros();
  unsigned long duration = current_time - last_edge_time;
  
  // Basic frame filtering (Standard PPM pulses range between 1000us to 2000us, blanking > 4000us)
  if (duration >= 900 && duration <= 22000) {
    pulse_duration = duration;
    new_pulse_detected = true;
    
    // Mirror the exact logical state change to the optical transmitter pin
    // Creating a sharp, high-slew-rate 5V TTL transition
    digitalWrite(PPM_OUTPUT_PIN, HIGH);
    delayMicroseconds(300); // Standard PPM fixed synchronization pulse width
    digitalWrite(PPM_OUTPUT_PIN, LOW);
  }
  
  last_edge_time = current_time;
}


