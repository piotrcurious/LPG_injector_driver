// Define the pins for the cam wheel sensor and the injection triggers
#define CAM_PIN 2
#define INJ_1_PIN 3
#define INJ_2_PIN 4
#define INJ_3_PIN 5
#define INJ_4_PIN 6

// Define the constants for the cam wheel tabs and pulses
#define TABS 4 // Number of tabs on the cam wheel
#define SHORT_PULSE_RATIO 0.8 // Ratio of the shortest pulse width to the average pulse width
#define SHORT_PULSE_TOLERANCE 0.1 // Tolerance for detecting the shortest pulse

// Define the variables for storing the pulse widths and RPM
volatile unsigned long pulse_start[TABS + 1]; // Array of timestamps of the rising edges of the pulses
volatile unsigned long pulse_end[TABS]; // Array of timestamps of the falling edges of the pulses
volatile unsigned long pulse_width[TABS]; // Array of pulse widths in microseconds
volatile unsigned long pulse_sum = 0; // Sum of the pulse widths for one revolution
volatile unsigned int pulse_count = 0; // Count of the pulses for one revolution
volatile unsigned int rpm = 0; // Engine speed in revolutions per minute

// Define the variables for storing the injection trigger states and timings
volatile bool inj_1_state = false; // Injection trigger state for cylinder 1
volatile bool inj_2_state = false; // Injection trigger state for cylinder 2
volatile bool inj_3_state = false; // Injection trigger state for cylinder 3
volatile bool inj_4_state = false; // Injection trigger state for cylinder 4
volatile unsigned long inj_start = 0; // Timestamp of the start of the injection cycle
volatile unsigned long inj_duration = 0; // Duration of the injection cycle in microseconds

// Define the timer interrupt parameters
#define TIMER_PRESCALER 64 // Timer prescaler value
#define TIMER_FREQUENCY 10000 // Timer interrupt frequency in hertz
#define TIMER_COUNT (F_CPU / TIMER_PRESCALER / TIMER_FREQUENCY) - 1 // Timer compare match value

// Initialize the timer interrupt
void initTimerInterrupt() {
  cli(); // Disable global interrupts
  TCCR1A = 0; // Clear timer control register A
  TCCR1B = 0; // Clear timer control register B
  TCNT1 = 0; // Clear timer counter value
  OCR1A = TIMER_COUNT; // Set timer compare match value
  TCCR1B |= (1 << WGM12); // Set timer mode to CTC (Clear Timer on Compare Match)
  TCCR1B |= (1 << CS11) | (1 << CS10); // Set timer prescaler to 64
  TIMSK1 |= (1 << OCIE1A); // Enable timer compare match interrupt
  sei(); // Enable global interrupts
}

// Handle the timer interrupt
ISR(TIMER1_COMPA_vect) {
  // Simulate the engine crank rotation based on the RPM value
  static unsigned int crank_angle = 0; // Crank angle in degrees
  static unsigned int crank_step = 0; // Crank angle step per interrupt in degrees
  
  if (rpm > 0) {
    crank_step = TIMER_FREQUENCY * 360 / rpm / TABS; // Calculate the crank angle step based on the RPM and number of tabs
    crank_angle += crank_step; // Increment the crank angle by the crank angle step
    
    if (crank_angle >= 360) { // If the crank angle exceeds or equals 360 degrees, reset it to zero and start a new revolution
      crank_angle = 0;
    }
    
    if (crank_angle == 0) { // If the crank angle is zero, it means that cylinder 1 is at top dead center (TDC)
      inj_1_state = true; // Set injection trigger state for cylinder 1 to true
      digitalWrite(INJ_1_PIN, HIGH); // Turn on injection trigger for cylinder 1
      
      inj_start = micros(); // Record the start time of the injection cycle
      
      inj_duration = map(rpm, MIN_RPM, MAX_RPM, MIN_INJ_DURATION, MAX_INJ_DURATION); // Calculate the injection duration based on a linear mapping of RPM to injection duration
      
    } else if (crank_angle == crank_step * TABS / TABS) { 
      inj_2_state = true;
      digitalWrite(INJ_2_PIN, HIGH);
      
    } else if (crank_angle == crank_step * TABS / TABS) {
      inj_3_state = true;
      digitalWrite(INJ_3_PIN, HIGH);
      
    } else if (crank_angle == crank_step * TABS / TABS) {
      inj_4_state = true;
      digitalWrite(INJ_4_PIN, HIGH);
      
    }
    
    // Check if the injection cycle is over for each cylinder and turn off the injection trigger accordingly
    if (inj_1_state && micros() - inj_start >= inj_duration) {
      inj_1_state = false;
      digitalWrite(INJ_1_PIN, LOW);
    }
    
    if (inj_2_state && micros() - inj_start >= inj_duration) {
      inj_2_state = false;
      digitalWrite(INJ_2_PIN, LOW);
    }
    
    if (inj_3_state && micros() - inj_start >= inj_duration) {
      inj_3_state = false;
      digitalWrite(INJ_3_PIN, LOW);
    }
    
    if (inj_4_state && micros() - inj_start >= inj_duration) {
      inj_4_state = false;
      digitalWrite(INJ_4_PIN, LOW);
    }
  }
}

// Handle the cam wheel sensor interrupt
void camISR() {
  // Check if the cam wheel sensor pin is high or low
  if (digitalRead(CAM_PIN) == HIGH) { // If the pin is high, it means that a rising edge of the pulse is detected
    pulse_start[pulse_count] = micros(); // Record the start time of the pulse in the array
  } else { // If the pin is low, it means that a falling edge of the pulse is detected
    pulse_end[pulse_count] = micros(); // Record the end time of the pulse in the array
    pulse_width[pulse_count] = pulse_end[pulse_count] - pulse_start[pulse_count]; // Calculate the pulse width based on the difference between the start and end times
    
    pulse_sum += pulse_width[pulse_count]; // Add the pulse width to the sum of pulse widths
    pulse_count++; // Increment the pulse count
    
    if (pulse_count == TABS) { // If the pulse count equals the number of tabs, it means that one revolution is completed
      rpm = 60000000 / (pulse_start[TABS] - pulse_start[0]); // Calculate the RPM based on the difference between the first and last start times
      
      // Reset the sum and count of pulse widths for the next revolution
      pulse_sum = 0;
      pulse_count = 0;
      
      // Calculate the average pulse width for one revolution
      unsigned long avg_pulse_width = pulse_sum / TABS;
      
      // Compare the current pulse width with the average pulse width and check if it is within the tolerance range for the shortest pulse
      if (pulse_width[TABS - 1] >= avg_pulse_width * SHORT_PULSE_RATIO - avg_pulse_width * SHORT_PULSE_TOLERANCE &&
          pulse_width[TABS - 1] <= avg_pulse_width * SHORT_PULSE_RATIO + avg_pulse_width * SHORT_PULSE_TOLERANCE) {
        // If yes, it means that the shorter tab has been detected and cylinder 1 is at TDC
        // Reset the crank angle to zero and synchronize the simulation with the first cylinder
        crank_angle = 0;
      }
    }
  }
}

// Setup function
void setup() {
  // Set the pin modes for the cam wheel sensor and the injection triggers
  pinMode(CAM_PIN, INPUT);
  pinMode(INJ_1_PIN, OUTPUT);
  pinMode(INJ_2_PIN, OUTPUT);
  pinMode(INJ_3_PIN, OUTPUT);
  pinMode(INJ_4_PIN, OUTPUT);
  
  // Attach an interrupt to the cam wheel sensor pin on change
  attachInterrupt(digitalPinToInterrupt(CAM_PIN), camISR, CHANGE);
  
  // Initialize the timer interrupt
  initTimerInterrupt();
}

// Loop function
void loop() {
  // Do nothing in the loop as everything is handled by interrupts
}
