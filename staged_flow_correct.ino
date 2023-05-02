// Arduino code for low impedance LPG injector driver with staged injection support

// Define the pins for the injectors and the sensors
#define INJECTOR1 9 // PWM pin for injector 1
#define INJECTOR2 10 // PWM pin for injector 2
#define PETROL_INJECTOR 2 // digital input pin for petrol injector
#define GAS_PRESSURE A0 // analog input pin for gas pressure
#define GAS_TEMPERATURE A1 // analog input pin for gas temperature
#define LAMBDA_PROBE A2 // analog input pin for lambda probe state
#define CORRECTION A3 // analog input pin for general correction

// Define the constants for the injector opening and sustaining times and duty cycles
#define OPENING_TIME 1000 // microseconds
#define SUSTAIN_DUTY_CYCLE 50 // percent
#define OPENING_DUTY_CYCLE 100 // percent

// Define the arrays for the injector maps stored in PROGMEM
// Each array has 16 elements corresponding to 16 intervals of petrol injector pulse length from 0 to 4096 microseconds
// The values are the opening times of the injectors in microseconds for each interval
const uint16_t injector1_map[16] PROGMEM = { /* values mapping petrol injector pulse length to opening time of injector 1 */ };
const uint16_t injector2_map[16] PROGMEM = { /* values mapping petrol injector pulse length to opening time of injector 2 */ };

// Define the variables for the sensor readings and the corrections
volatile uint32_t petrol_injector_rising_time; // microseconds
volatile uint32_t petrol_injector_falling_time; // microseconds
volatile uint16_t petrol_injector_pulse; // microseconds
uint16_t gas_pressure; // arbitrary units
uint16_t gas_temperature; // arbitrary units
uint16_t lambda_probe_state; // arbitrary units
uint16_t correction; // arbitrary units

// Define the variables for the calculated opening times of the injectors
uint16_t injector1_opening_time; // microseconds
uint16_t injector2_opening_time; // microseconds

// Define the variables for the current states of the injectors
bool injector1_opening; // true if injector 1 is in opening phase, false otherwise
bool injector2_opening; // true if injector 2 is in opening phase, false otherwise

// Define the variable for the petrol injector rising edge flag
volatile bool petrol_injector_rising_edge;

// Define the constants for the LPG pressure, temperature and nozzle sizes correction formula³
#define K1 0.00001 // coefficient for gas pressure correction
#define K2 0.00002 // coefficient for gas temperature correction
#define K3 0.00003 // coefficient for nozzle size correction
#define NOZZLE1_SIZE 2.0 // mm, nozzle size of injector 1 
#define NOZZLE2_SIZE 2.5 // mm, nozzle size of injector 2

// Setup function
void setup() {
  // Set the pins as inputs or outputs
  pinMode(INJECTOR1, OUTPUT);
  pinMode(INJECTOR2, OUTPUT);
  pinMode(PETROL_INJECTOR, INPUT);
  
  // Attach an interrupt to the petrol injector pin on change of state
  attachInterrupt(digitalPinToInterrupt(PETROL_INJECTOR), petrol_injector_ISR, CHANGE);
  
  // Initialize the petrol injector rising edge flag to false
  petrol_injector_rising_edge = false;
  
  // Initialize the injectors states to false
  injector1_opening = false;
  injector2_opening = false;
}

// Loop function
void loop() {
  // Read the sensors and apply corrections if needed
  gas_pressure = analogRead(GAS_PRESSURE);
  gas_temperature = analogRead(GAS_TEMPERATURE);
  lambda_probe_state = analogRead(LAMBDA_PROBE);
  correction = analogRead(CORRECTION);
  
  // Calculate the opening times of the injectors based on interpolation and the corrections
  
  // Find the index of the lower bound of the interval that contains the petrol injector pulse length 
  uint8_t index = min(petrol_injector_pulse / 256, 15); 
  
  // Find the fraction of the petrol injector pulse length within that interval 
  float fraction = (petrol_injector_pulse % 256) / 256.0;
  
  // Interpolate between the lower and upper bound values of that interval for each injector map 
  injector1_opening_time = pgm_read_word(&injector1_map[index]) + fraction * (pgm_read_word(&injector1_map[index + 1]) - pgm_read_word(&injector1_map[index]));
  injector2_opening_time = pgm_read_word(&injector2_map[index]) + fraction * (pgm_read_word(&injector2_map[index + 1]) - pgm_read_word(&injector2_map[index]));
  
  // Apply the LPG pressure, temperature and nozzle sizes correction formula to the opening times
  injector1_opening_time = injector1_opening_time * (1 + K1 * gas_pressure + K2 * gas_temperature + K3 * NOZZLE1_SIZE) + correction;
  injector2_opening_time = injector2_opening_time * (1 + K1 * gas_pressure + K2 * gas_temperature + K3 * NOZZLE2_SIZE) + correction;
  
  // Check if there is a rising edge of the petrol injector signal
  if (petrol_injector_rising_edge) {
    // Reset the flag to false
    petrol_injector_rising_edge = false;
    
    // Check if the opening times of the injectors are greater than their opening time constants
    if (injector1_opening_time > OPENING_TIME) {
      // Open injector 1 with full duty cycle and set its state to true
      analogWrite(INJECTOR1, OPENING_DUTY_CYCLE);
      injector1_opening = true;
    }
    
    if (injector2_opening_time > OPENING_TIME) {
      // Open injector 2 with full duty cycle and set its state to true
      analogWrite(INJECTOR2, OPENING_DUTY_CYCLE);
      injector2_opening = true;
    }
    
    // Start a timer to check when to switch to sustaining phase or close the injectors 
    unsigned long start_time = micros();
    
    while ((injector1_opening || injector2_opening) && (current_time - start_time < max(injector1_opening_time, injector2_opening_time))) {
      // Get the current time in microseconds
      unsigned long current_time = micros();
      
      // Check if injector 1 is in opening phase and has reached the opening time
      if (injector1_opening && (current_time - start_time >= OPENING_TIME)) {
        // Switch injector 1 to sustaining phase with a lower duty cycle and set its state to false
        analogWrite(INJECTOR1, SUSTAIN_DUTY_CYCLE);
        injector1_opening = false;
      }
      
      // Check if injector 2 is in opening phase and has reached the opening time
      if (injector2_opening && (current_time - start_time >= OPENING_TIME)) {
        // Switch injector 2 to sustaining phase with a lower duty cycle and set its state to false
        analogWrite(INJECTOR2, SUSTAIN_DUTY_CYCLE);
        injector2_opening = false;
      }
      
      // Check if injector 1 has reached its calculated opening time
      if (!injector1_opening && (current_time - start_time >= injector1_opening_time)) {
        // Close injector 1 by setting the duty cycle to zero
        analogWrite(INJECTOR1, 0);
      }
      
      // Check if injector 2 has reached its calculated opening time
      if (!injector2_opening && (current_time - start_time >= injector2_opening_time)) {
        // Close injector 2 by setting the duty cycle to zero
        analogWrite(INJECTOR2, 0);
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
    // Read the current time in microseconds when the signal goes high
    petrol_injector_rising_time = micros();
    
    // Set the flag to true to indicate a rising edge has occurred
    petrol_injector_rising_edge = true;
  }
  
  // If the state is low, it means a falling edge has occurred
  else {
    // Read the current time in microseconds when the signal goes low
    petrol_injector_falling_time = micros();
    
    // Calculate the pulse length of the petrol injector signal in microseconds
    petrol_injector_pulse = petrol_injector_falling_time - petrol_injector_rising_time;
  }
}

//Source: Conversation with Bing, 5/3/2023
//(1) Gas Correction Factors | Brooks Instrument. https://www.brooksinstrument.com/en/resources/sizing-tools.
//(2) PAPER OPEN ACCESS )ORZFKDUDFWHULVWLFVRIJDVLQMHFWRUV - Institute of Physics. https://iopscience.iop.org/article/10.1088/1757-899X/664/1/012021/pdf.
//(3) (PDF) Modelling of the Low-Pressure Gas Injector Operation - ResearchGate. https://www.researchgate.net/publication/341740770_Modelling_of_the_Low-Pressure_Gas_Injector_Operation.
//(4) Volume correction factors—liquefied petroleum gas or ... - Strategis. https://ised-isde.canada.ca/site/measurement-canada/en/laws-and-requirements/volume-correction-factors-liquefied-petroleum-gas-or-propane-500-kgm3.
