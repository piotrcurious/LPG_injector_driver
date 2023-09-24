// Define the angle of injection relative to the start of intake valve opening
#define INJECTION_ANGLE 30

// Define the injector dead times in microseconds
#define DEAD_TIME_OPEN 1000
#define DEAD_TIME_CLOSE 500

// Define the peak current and hold current for the injector in milliamps
#define PEAK_CURRENT 4000
#define HOLD_CURRENT 1000

// Define the resistance and inductance of the injector in ohms and henrys
#define RESISTANCE 3
#define INDUCTANCE 0.01

// Define the pins for the injector driver and the hall cam sensor
#define INJECTOR_PIN 9
#define HALL_PIN 2

// Define the number of cylinders , the firing order and which cylinder this injector serves
#define NUM_CYLINDERS 4
#define FIRING_ORDER {1, 3, 4, 2}
#define I_AM_THE_CYLINDER_NUMBER 1

// Define the number of degrees per crank revolution
#define DEGREES_PER_REV 720

// Define the analog input pin for the injector open time signal
#define ANALOG_PIN A0

// Define the analog input range and resolution
#define ANALOG_MIN 0
#define ANALOG_MAX 1023
#define ANALOG_RES 10 // in bits

// Define the minimum and maximum injector open time in microseconds
#define OPEN_TIME_MIN 1000
#define OPEN_TIME_MAX 10000

// Initialize some global variables
volatile int current_cylinder = 0; // the current cylinder number
volatile int current_angle = 0; // the current crank angle in degrees
volatile bool injection_flag = false; // a flag to indicate if injection is needed
volatile unsigned long injection_start = 0; // the start time of injection in microseconds
volatile unsigned long injection_end = 0; // the end time of injection in microseconds
volatile unsigned long peak_time = 0; // the time to switch from peak to hold current in microseconds
volatile int engine_speed = 3000; // the current engine speed in RPM

// A function to map a value from one range to another
long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// A function to calculate the time to switch from peak to hold current
unsigned long calculate_peak_time(int peak_current, int hold_current, int resistance, int inductance) {
  return (unsigned long) (inductance / resistance * log((float) peak_current / hold_current));
}

// A function to set the PWM duty cycle for the injector driver pin
void set_duty_cycle(int duty_cycle) {
  analogWrite(INJECTOR_PIN, duty_cycle);
}

// A function to calculate the timer compare match value based on the engine speed and the desired interrupt frequency
unsigned int calculate_timer_value(int engine_speed, int interrupt_frequency) {
  // Calculate the timer clock frequency based on the prescaler value of 8
  unsigned long timer_clock = F_CPU / 8;

  // Calculate the timer compare match value based on the formula: OCR1A = (timer_clock / interrupt_frequency) - 1
  unsigned int timer_value = (timer_clock / interrupt_frequency) - 1;

  // Return the timer value
  return timer_value;
}

// A function to handle the hall cam sensor interrupt
void hall_interrupt() {
  // Get the state of the hall sensor pin
  int hall_state = digitalRead(HALL_PIN);

  // If the hall sensor is high, it means a new cylinder is starting its intake stroke
  if (hall_state == HIGH) {
    // Increment the current cylinder number and wrap around if needed
    current_cylinder++;
    if (current_cylinder > NUM_CYLINDERS) {
      current_cylinder = 1;
    }

    // Reset the current crank angle to zero
    current_angle = 0;

    // Check if injection is needed for this cylinder based on the firing order
    int firing_order[] = FIRING_ORDER;
    if (firing_order[current_cylinder - 1] == I_AM_THE_CYLINDER_NUMBER) {
      injection_flag = true;
    }
    else {
      injection_flag = false;
    }

    // Read the engine speed from the hall sensor input and update it if needed 
    int new_engine_speed = read_engine_speed();
    if (new_engine_speed != engine_speed) {
      engine_speed = new_engine_speed;

      // Calculate the new timer compare match value based on the new engine speed and the desired interrupt frequency of one degree per crank rotation 
      unsigned int timer_value = calculate_timer_value(engine_speed, engine_speed * DEGREES_PER_REV / 60);

      // Update the timer value
      cli(); // disable global interrupts
      OCR1A = timer_value; // set compare match register to new value
      sei(); // enable global interrupts
    }
  }
}

// A function to handle the timer interrupt for crank angle calculation
void timer_interrupt() {
  // Increment the current crank angle by one degree and wrap around if needed
  current_angle++;
  if (current_angle > DEGREES_PER_REV) {
    current_angle = 0;
  }

  // Check if injection is needed and the current angle matches the injection angle
  if (injection_flag && current_angle == INJECTION_ANGLE) {
    // Read the analog input for the injector open time signal and map it to the desired range
    int analog_value = analogRead(ANALOG_PIN);
    unsigned long open_time = map(analog_value, ANALOG_MIN, ANALOG_MAX, OPEN_TIME_MIN, OPEN_TIME_MAX);

    // Calculate the start and end time of injection by adding or subtracting the dead times
    injection_start = micros() + DEAD_TIME_OPEN;
    injection_end = injection_start + open_time - DEAD_TIME_CLOSE;

    // Set the PWM duty cycle to 100% to start injecting with peak current
    set_duty_cycle(255);

    // Set a flag to indicate that injection has started
    injection_flag = false;
  }
}

// A function to read the engine speed from the hall sensor input
int read_engine_speed() {
  // Get the current time in microseconds
  static unsigned long last_time = 0;
  unsigned long current_time = micros();

  // Calculate the time difference between two consecutive hall sensor interrupts in microseconds
  unsigned long time_difference = current_time - last_time;

  // Update the last time to the current time
  last_time = current_time;

  // Calculate the engine speed in RPM based on the time difference and the number of cylinders
  int engine_speed = (int) (60000000.0 / (time_difference * NUM_CYLINDERS));

  // Return the engine speed
  return engine_speed;
}

// The setup function runs once when the Arduino board is powered on or reset
void setup() {
  // Set the injector driver pin as output and initialize it to low
  pinMode(INJECTOR_PIN, OUTPUT);
  digitalWrite(INJECTOR_PIN, LOW);

  // Set the hall sensor pin as input and enable the pull-up resistor
  pinMode(HALL_PIN, INPUT_PULLUP);

  // Attach an interrupt to the hall sensor pin to trigger on rising edge
  attachInterrupt(digitalPinToInterrupt(HALL_PIN), hall_interrupt, RISING);

  // Configure a timer to generate an interrupt every one degree of crank rotation
  // Assuming an initial engine speed of 3000 RPM, this corresponds to a frequency of 3600 Hz
  // For Arduino Uno, we can use Timer1 with a prescaler of 8 and a compare match value of 278
  cli(); // disable global interrupts
  TCCR1A = 0; // set TCCR1A register to zero
  TCCR1B = 0; // set TCCR1B register to zero
  TCNT1 = 0; // initialize counter value to zero
  OCR1A = calculate_timer_value(engine_speed, engine_speed * DEGREES_PER_REV / 60); // set compare match register to initial value
  TCCR1B |= (1 << WGM12); // turn on CTC mode
  TCCR1B |= (1 << CS11); // set prescaler to 8
  TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
  sei(); // enable global interrupts

  // Calculate the peak time for the injector based on the given parameters
  peak_time = calculate_peak_time(PEAK_CURRENT, HOLD_CURRENT, RESISTANCE, INDUCTANCE);
}

// The loop function runs repeatedly after the setup function is completed
void loop() {
  // Check if the current time is within the injection period
  unsigned long current_time = micros();
  if (current_time >= injection_start && current_time <= injection_end) {
    // Check if the current time is past the peak time and switch to hold current if needed
    if (current_time >= injection_start + peak_time) {
      // Calculate the duty cycle for the hold current based on the resistance and supply voltage
      // Assuming a supply voltage of 12 volts, the duty cycle is given by (hold_current * resistance) / (supply_voltage * 255)
      int duty_cycle = (HOLD_CURRENT * RESISTANCE) / (12 * 255);
      set_duty_cycle(duty_cycle);
    }
  }
  else {
        // Set the PWM duty cycle to zero to stop injecting
    set_duty_cycle(0);
  }
}

// The ISR for Timer1 compare match interrupt
ISR(TIMER1_COMPA_vect) {
  timer_interrupt();
}

