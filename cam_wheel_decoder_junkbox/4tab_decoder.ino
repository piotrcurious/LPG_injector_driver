
// Define the pins for the cam wheel sensor and the injection triggers
#define CAM_PIN 2 // The pin connected to the cam wheel sensor
#define INJ_1_PIN 3 // The pin for the first cylinder injection trigger
#define INJ_2_PIN 4 // The pin for the second cylinder injection trigger
#define INJ_3_PIN 5 // The pin for the third cylinder injection trigger
#define INJ_4_PIN 6 // The pin for the fourth cylinder injection trigger

// Define some constants for the cam wheel geometry and timing
#define NUM_TABS 4 // The number of tabs on the cam wheel
#define SHORT_PULSE 0.8 // The ratio of the shortest pulse width to the average pulse width
#define INJ_DUR 2.0 // The injection duration in milliseconds
#define INJ_ADV 10.0 // The injection advance angle in degrees

// Define some variables to store the pulse widths and RPM
volatile unsigned long pulse_start = 0; // The start time of the current pulse in microseconds
volatile unsigned long pulse_end = 0; // The end time of the current pulse in microseconds
volatile unsigned long pulse_width = 0; // The width of the current pulse in microseconds
volatile unsigned long prev_pulse_width = 0; // The width of the previous pulse in microseconds
volatile unsigned long avg_pulse_width = 0; // The average width of the four pulses in microseconds
volatile unsigned long rpm = 0; // The engine RPM calculated from the pulse widths

// Define some variables to keep track of the cylinder order and injection timing
volatile byte cyl_count = 0; // The current cylinder count (0 to 3)
volatile unsigned long inj_start = 0; // The start time of the injection in microseconds
volatile unsigned long inj_end = 0; // The end time of the injection in microseconds
volatile unsigned long inj_delay = 0; // The delay time before the injection in microseconds

// Define some variables to store the timer values and prescalers
volatile unsigned int timer1_val = 0; // The value of timer1 (16-bit)
volatile byte timer1_ps = 0; // The prescaler of timer1 (1, 8, 64, or 256)
volatile unsigned int timer2_val = 0; // The value of timer2 (8-bit)
volatile byte timer2_ps = 0; // The prescaler of timer2 (1, 8, 32, or 64)

// Define an interrupt service routine for the cam wheel sensor
void camISR() {
  // Check if the sensor is rising or falling
  if (digitalRead(CAM_PIN) == HIGH) {
    // Sensor is rising, record the start time of the pulse
    pulse_start = micros();
    // Calculate the width of the previous pulse
    prev_pulse_width = pulse_start - pulse_end;
    // Update the average pulse width
    avg_pulse_width = (avg_pulse_width * (NUM_TABS - 1) + prev_pulse_width) / NUM_TABS;
    // Calculate the engine RPM from the average pulse width
    rpm = 60000000UL / (avg_pulse_width * NUM_TABS);
    // Increment the cylinder count
    cyl_count++;
    if (cyl_count >= NUM_TABS) {
      cyl_count = 0;
    }
    // Calculate the delay time before the injection based on the injection advance angle
    inj_delay = (unsigned long)(INJ_ADV / 360.0 * avg_pulse_width);
    // Set the start and end time of the injection
    inj_start = pulse_start - inj_delay;
    inj_end = inj_start + INJ_DUR * 1000;
    // Calculate the timer values and prescalers for each injection pin based on the start and end time
    setTimerValues();
    // Enable timer interrupts for each injection pin
    enableTimerInterrupts();
    
  } else {
    // Sensor is falling, record the end time of the pulse
    pulse_end = micros();
    // Calculate the width of the current pulse
    pulse_width = pulse_end - pulse_start;
    // Check if the current pulse is shorter than the average pulse by a factor of SHORT_PULSE
    if (pulse_width < avg_pulse_width * SHORT_PULSE) {
      // Shortest pulse detected, reset the cylinder count to zero
      cyl_count = 0;
    }
    
  }
}

// Define a function to calculate the timer values and prescalers for each injection pin based on their start and end time
void setTimerValues() {
  
}

// Define a function to enable timer interrupts for each injection pin based on their timer values and prescalers
void enableTimerInterrupts() {
  
}

// Define a function to disable timer interrupts for each injection pin
void disableTimerInterrupts() {
  
}

// Define an interrupt service routine for timer1 compare match A
void timer1AISR() {
  // Check which cylinder is next and trigger the corresponding injection pin
  switch (cyl_count) {
    case 0: // First cylinder, shortest pulse
      digitalWrite(INJ_1_PIN, HIGH);
      break;
    case 1: // Second cylinder, normal pulse
      digitalWrite(INJ_2_PIN, HIGH);
      break;
    case 2: // Third cylinder, normal pulse
      digitalWrite(INJ_3_PIN, HIGH);
      break;
    case 3: // Fourth cylinder, normal pulse
      digitalWrite(INJ_4_PIN, HIGH);
      break;
  }
  // Disable timer1 compare match A interrupt
  TIMSK1 &= ~(1 << OCIE1A);
}

// Define an interrupt service routine for timer1 compare match B
void timer1BISR() {
  // Check which cylinder is current and turn off the corresponding injection pin
  switch (cyl_count) {
    case 0: // First cylinder, shortest pulse
      digitalWrite(INJ_1_PIN, LOW);
      break;
    case 1: // Second cylinder, normal pulse
      digitalWrite(INJ_2_PIN, LOW);
      break;
    case 2: // Third cylinder, normal pulse
      digitalWrite(INJ_3_PIN, LOW);
      break;
    case 3: // Fourth cylinder, normal pulse
      digitalWrite(INJ_4_PIN, LOW);
      break;
  }
  // Disable timer1 compare match B interrupt
  TIMSK1 &= ~(1 << OCIE1B);
}

// Define an interrupt service routine for timer2 compare match A
void timer2AISR() {
  
}

// Define an interrupt service routine for timer2 compare match B
void timer2BISR() {
  
}

// Define a setup function to initialize the pins and the interrupt
void setup() {
  // Set the cam wheel sensor pin as input with pull-up resistor
  pinMode(CAM_PIN, INPUT_PULLUP);
  // Set the injection trigger pins as output and initialize them to low
  pinMode(INJ_1_PIN, OUTPUT);
  pinMode(INJ_2_PIN, OUTPUT);
  pinMode(INJ_3_PIN, OUTPUT);
  pinMode(INJ_4_PIN, OUTPUT);
  digitalWrite(INJ_1_PIN, LOW);
  digitalWrite(INJ_2_PIN, LOW);
  digitalWrite(INJ_3_PIN, LOW);
  digitalWrite(INJ_4_PIN, LOW);
  // Attach an interrupt to the cam wheel sensor pin on change
  attachInterrupt(digitalPinToInterrupt(CAM_PIN), camISR, CHANGE);
}

// Define a loop function to do nothing
void loop() {
  // Do nothing
}
