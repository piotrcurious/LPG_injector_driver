// Arduino code for low impedance LPG injector driver with staged injection support

// Define the pins for the injectors and the sensors
#define INJECTOR1 9 // PWM pin for injector 1
#define INJECTOR2 10 // PWM pin for injector 2
#define PETROL_INJECTOR 2 // digital input pin for petrol injector
#define GAS_PRESSURE A0 // analog input pin for gas pressure
#define GAS_TEMPERATURE A1 // analog input pin for gas temperature
#define LAMBDA_PROBE A2 // analog input pin for lambda probe state
#define CORRECTION A3 // analog input pin for general correction
#define LAMBDA_FORCE A4 // analog input pin for lambda force correction

// Define the constants for the injector opening and sustaining times and duty cycles
#define OPENING_TIME 1000 // microseconds
#define SUSTAIN_DUTY_CYCLE 127 // 0-255 scale (50% of 255)
#define OPENING_DUTY_CYCLE 255 // 0-255 scale (100% of 255)

// Define the arrays for the injector maps stored in PROGMEM
// Each array has 16 elements corresponding to 16 intervals of petrol injector pulse length from 0 to 4096 microseconds
// The values are the opening times of the injectors in microseconds for each interval
// Example values - replace with actual calibration data
const uint16_t injector1_map[16] PROGMEM = {
  0, 500, 1000, 1500, 2000, 2500, 3000, 3500,
  4000, 4500, 5000, 5500, 6000, 6500, 7000, 7500
};

const uint16_t injector2_map[16] PROGMEM = {
  0, 500, 1000, 1500, 2000, 2500, 3000, 3500,
  4000, 4500, 5000, 5500, 6000, 6500, 7000, 7500
};

// Define the variables for the sensor readings and the corrections
volatile uint32_t petrol_injector_rising_time = 0; // microseconds
volatile uint32_t petrol_injector_falling_time = 0; // microseconds
volatile uint16_t petrol_injector_pulseISR = 0; // read in ISR, microseconds
uint16_t petrol_injector_pulse = 0; // copy of ISR value, microseconds

uint16_t gas_pressure = 0; // arbitrary units
uint16_t gas_temperature = 0; // arbitrary units
uint16_t lambda_probe_state = 512; // arbitrary units (initialized to midpoint)
uint16_t correction = 0; // arbitrary units

// Define the variables for the calculated opening times of the injectors
uint16_t injector1_opening_time = 0; // microseconds
uint16_t injector2_opening_time = 0; // microseconds

// Define the variables for the current states of the injectors
bool injector1_active = false; // true if injector 1 is active
bool injector2_active = false; // true if injector 2 is active

// Define the variable for the petrol injector rising edge flag
volatile bool petrol_injector_rising_edge = false;

// Define the constants for the LPG pressure, temperature and nozzle sizes correction formula
#define K1 0.00001 // coefficient for gas pressure correction
#define K2 0.00002 // coefficient for gas temperature correction
#define K3 0.00003 // coefficient for nozzle size correction
#define NOZZLE1_SIZE 2.0 // mm, nozzle size of injector 1 
#define NOZZLE2_SIZE 2.5 // mm, nozzle size of injector 2

// Define the constants for the closed loop lambda probe correction
#define STOICHIOMETRIC_RATIO 14.7 // ideal air-fuel ratio 
#define LEAN_MIX_CORRECTION -0.05 // constant correction factor for lean mix 
#define RICH_MIX_CORRECTION -0.01 // constant correction factor for rich mix 
#define LAMBDA_FORCE_FACTOR -0.001 // coefficient for lambda force correction 
#define LAMBDA_THRESHOLD 0.02 // threshold for considering mix lean or rich

// Setup function
void setup() {
  // Set the pins as inputs or outputs
  pinMode(INJECTOR1, OUTPUT);
  pinMode(INJECTOR2, OUTPUT);
  pinMode(PETROL_INJECTOR, INPUT);
  
  // Initialize injectors to OFF state
  analogWrite(INJECTOR1, 0);
  analogWrite(INJECTOR2, 0);
  
  // Attach an interrupt to the petrol injector pin on change of state
  attachInterrupt(digitalPinToInterrupt(PETROL_INJECTOR), petrol_injector_ISR, CHANGE);
  
  // Initialize the petrol injector rising edge flag to false
  petrol_injector_rising_edge = false;
  
  // Initialize the injectors states to false
  injector1_active = false;
  injector2_active = false;
  
  // Optional: Initialize serial for debugging
  // Serial.begin(115200);
}

// Loop function
void loop() {
  // Read the sensors
  gas_pressure = analogRead(GAS_PRESSURE);
  gas_temperature = analogRead(GAS_TEMPERATURE);
  lambda_probe_state = analogRead(LAMBDA_PROBE);
  correction = analogRead(CORRECTION);

  // Safely copy the pulse value from ISR
  noInterrupts();
  uint16_t pulse_copy = petrol_injector_pulseISR;
  bool rising_edge = petrol_injector_rising_edge;
  if (rising_edge) {
    petrol_injector_rising_edge = false; // Reset flag
  }
  interrupts();
  
  petrol_injector_pulse = pulse_copy;
  
  // Apply the closed loop lambda probe correction to the petrol injector pulse length
  // Normalize lambda probe state to 0.0-2.0 range (assuming 0-1023 ADC range maps to 0-2.0 lambda)
  float lambda = (lambda_probe_state / 1023.0) * 2.0;
  
  // Calculate corrected pulse length
  float pulse_correction = 1.0;
  
  // Check if the lambda indicates lean or rich mix
  if (lambda > (1.0 + LAMBDA_THRESHOLD)) {
    // Lean mix, apply correction to increase fuel
    pulse_correction = 1.0 + LEAN_MIX_CORRECTION;
  }
  else if (lambda < (1.0 - LAMBDA_THRESHOLD)) {
    // Rich mix, apply correction with lambda force input
    uint16_t lambda_force = analogRead(LAMBDA_FORCE);
    pulse_correction = 1.0 + RICH_MIX_CORRECTION + (LAMBDA_FORCE_FACTOR * lambda_force);
  }
  
  // Apply correction (ensure we don't go negative)
  uint16_t corrected_pulse = (uint16_t)max(0, (int32_t)(petrol_injector_pulse * pulse_correction));
  
  // Calculate the opening times of the injectors based on interpolation and the corrections
  
  // Find the index of the lower bound of the interval that contains the petrol injector pulse length 
  uint8_t index = min(corrected_pulse / 256, 15); 
  
  // Find the fraction of the petrol injector pulse length within that interval 
  float fraction = (corrected_pulse % 256) / 256.0;
  
  // Interpolate between the lower and upper bound values for each injector map
  // Handle boundary case where index is 15
  uint16_t inj1_lower = pgm_read_word(&injector1_map[index]);
  uint16_t inj1_upper = (index < 15) ? pgm_read_word(&injector1_map[index + 1]) : inj1_lower;
  injector1_opening_time = inj1_lower + fraction * (inj1_upper - inj1_lower);
  
  uint16_t inj2_lower = pgm_read_word(&injector2_map[index]);
  uint16_t inj2_upper = (index < 15) ? pgm_read_word(&injector2_map[index + 1]) : inj2_lower;
  injector2_opening_time = inj2_lower + fraction * (inj2_upper - inj2_lower);
  
  // Apply the LPG pressure, temperature and nozzle sizes correction formula
  float lpg_correction = (1.0 + K1 * gas_pressure + K2 * gas_temperature);
  injector1_opening_time = (uint16_t)(injector1_opening_time * lpg_correction * (1.0 + K3 * NOZZLE1_SIZE)) + correction;
  injector2_opening_time = (uint16_t)(injector2_opening_time * lpg_correction * (1.0 + K3 * NOZZLE2_SIZE)) + correction;
  
  // Check if there is a rising edge of the petrol injector signal
  if (rising_edge && !injector1_active && !injector2_active) {
    
    // Determine which injectors to activate
    bool activate_inj1 = (injector1_opening_time > OPENING_TIME);
    bool activate_inj2 = (injector2_opening_time > OPENING_TIME);
    
    if (activate_inj1 || activate_inj2) {
      unsigned long start_time = micros();
      unsigned long current_time;
      unsigned long elapsed_time;
      
      // Open the injectors with full duty cycle
      if (activate_inj1) {
        analogWrite(INJECTOR1, OPENING_DUTY_CYCLE);
        injector1_active = true;
      }
      
      if (activate_inj2) {
        analogWrite(INJECTOR2, OPENING_DUTY_CYCLE);
        injector2_active = true;
      }
      
      // Track states for each injector
      bool inj1_in_opening = activate_inj1;
      bool inj2_in_opening = activate_inj2;
      
      // Control loop for injection timing
      uint16_t max_time = max(injector1_opening_time, injector2_opening_time);
      
      while (injector1_active || injector2_active) {
        current_time = micros();
        elapsed_time = current_time - start_time;
        
        // Handle injector 1
        if (injector1_active) {
          if (inj1_in_opening && elapsed_time >= OPENING_TIME) {
            // Switch to sustaining phase
            analogWrite(INJECTOR1, SUSTAIN_DUTY_CYCLE);
            inj1_in_opening = false;
          }
          
          if (elapsed_time >= injector1_opening_time) {
            // Close injector 1
            analogWrite(INJECTOR1, 0);
            injector1_active = false;
          }
        }
        
        // Handle injector 2
        if (injector2_active) {
          if (inj2_in_opening && elapsed_time >= OPENING_TIME) {
            // Switch to sustaining phase
            analogWrite(INJECTOR2, SUSTAIN_DUTY_CYCLE);
            inj2_in_opening = false;
          }
          
          if (elapsed_time >= injector2_opening_time) {
            // Close injector 2
            analogWrite(INJECTOR2, 0);
            injector2_active = false;
          }
        }
        
        // Safety timeout
        if (elapsed_time > max_time + 1000) {
          // Force close both injectors after timeout
          analogWrite(INJECTOR1, 0);
          analogWrite(INJECTOR2, 0);
          injector1_active = false;
          injector2_active = false;
        }
      }
    }
  }
}

// Interrupt service routine for the petrol injector signal change of state
void petrol_injector_ISR() {
  // Read the current state of the petrol injector signal
  bool state = digitalRead(PETROL_INJECTOR);
  
  // If the state is high, it means a rising edge has occurred
  if (state == HIGH) {
    // Record the rising edge time
    petrol_injector_rising_time = micros();
    
    // Set the flag to true to indicate a rising edge has occurred
    petrol_injector_rising_edge = true;
  }
  // If the state is low, it means a falling edge has occurred
  else {
    // Record the falling edge time
    petrol_injector_falling_time = micros();
    
    // Calculate the pulse length (handle timer overflow)
    if (petrol_injector_falling_time >= petrol_injector_rising_time) {
      petrol_injector_pulseISR = petrol_injector_falling_time - petrol_injector_rising_time;
    } else {
      // Handle micros() overflow (occurs every ~70 minutes)
      petrol_injector_pulseISR = (0xFFFFFFFF - petrol_injector_rising_time) + petrol_injector_falling_time + 1;
    }
  }
}
