// Define the parameters for the LPG injector driver
#define INJECTOR_PIN 2 // The pin number for the injector output
#define ANALOG_PIN 0 // The pin number for the analog input
#define INJECTION_ANGLE 30 // The angle relative to start of intake valve opening (in degrees)
#define DEAD_TIME 1.0 // The injector dead time (in milliseconds)
#define PEAK_CURRENT 4.0 // The peak current for the injector (in amps)
#define HOLD_CURRENT 1.0 // The hold current for the injector (in amps)
#define CYLINDER_NUMBER 1 // The cylinder number that the device services

// Define some constants for the calculations
#define PI 3.14159 // The value of pi
#define DEG_TO_RAD PI / 180.0 // The conversion factor from degrees to radians
#define RAD_TO_DEG 180.0 / PI // The conversion factor from radians to degrees
#define PWM_FREQUENCY 10000 // The PWM frequency for the injector output (in Hz)
#define PWM_RESOLUTION 8 // The PWM resolution for the injector output (in bits)
#define PWM_MAX_VALUE 255 // The maximum PWM value for the injector output
#define SENSOR_PIN 4 // The pin number for the sensor input
#define SENSOR_ANGLE 90 // The angle of the sensor relative to TDC (in degrees)
#define PULSES_PER_REV 4 // The number of pulses per revolution for the sensor input

// Declare some global variables for the program
volatile int rpm = 0; // The engine speed (in RPM)
volatile int cam_angle = 0; // The current cam angle (in degrees)
volatile int injection_time = 0; // The injection time (in microseconds)
volatile bool injection_flag = false; // The flag to indicate if injection is needed

// Initialize the program
void setup() {
  // Set the pin modes
  pinMode(INJECTOR_PIN, OUTPUT);
  pinMode(ANALOG_PIN, INPUT);
  pinMode(SENSOR_PIN, INPUT_PULLUP);

  // Attach an interrupt to the sensor input
  attachInterrupt(digitalPinToInterrupt(SENSOR_PIN), sensor_isr, RISING);

  // Configure the PWM output
  ledcSetup(0, PWM_FREQUENCY, PWM_RESOLUTION); // Use channel 0 for the injector output
  ledcAttachPin(INJECTOR_PIN, 0); // Attach the injector pin to channel 0
}

// Run the main loop
void loop() {
  // Read the analog input and map it to injection time
  int analog_value = analogRead(ANALOG_PIN); // Read the analog value (0-4095)
  injection_time = map(analog_value, 0, 4095, 0, 20000); // Map the analog value to injection time (0-20000 microseconds)

  // Check if injection is needed
  if (injection_flag) {
    // Calculate the injection angle and duration
    int injection_angle = cam_angle + INJECTION_ANGLE; // Add the injection angle to the current cam angle
    if (injection_angle >= 360) { // Wrap around if necessary
      injection_angle -= 360;
    }
    int injection_duration = injection_time + DEAD_TIME * 1000; // Add the dead time to the injection time

    // Calculate the peak and hold duty cycles
    float peak_duty_cycle = PEAK_CURRENT / (3.3 * PWM_FREQUENCY); // Calculate the peak duty cycle as a fraction of one period
    float hold_duty_cycle = HOLD_CURRENT / (3.3 * PWM_FREQUENCY); // Calculate the hold duty cycle as a fraction of one period
    int peak_pwm_value = round(peak_duty_cycle * PWM_MAX_VALUE); // Convert the peak duty cycle to a PWM value (0-255)
    int hold_pwm_value = round(hold_duty_cycle * PWM_MAX_VALUE); // Convert the hold duty cycle to a PWM value (0-255)

    // Calculate the peak and hold durations
    int peak_duration = round(peak_duty_cycle * injection_duration); // Calculate the peak duration in microseconds
    int hold_duration = injection_duration - peak_duration; // Calculate the hold duration in microseconds

    // Wait until the injection angle is reached
    while (cam_angle != injection_angle) {
      delayMicroseconds(1); // Wait for one microsecond
    }

    // Start the injection with peak current
    ledcWrite(0, peak_pwm_value); // Write the peak PWM value to channel 0

    // Wait for the peak duration
    delayMicroseconds(peak_duration); // Wait for the peak duration

    // Switch to hold current
    ledcWrite(0, hold_pwm_value); // Write the hold PWM value to channel 0

    // Wait for the hold duration
    delayMicroseconds(hold_duration); // Wait for the hold duration

    // Stop the injection
    ledcWrite(0, 0); // Write zero to channel 0

    // Reset the injection flag
    injection_flag = false;
  }
}

// Define the interrupt service routine for the sensor input
void sensor_isr() {
  // Calculate the time between pulses
  static unsigned long last_time = 0; // The last time a pulse was detected
  unsigned long current_time = micros(); // The current time
  unsigned long pulse_time = current_time - last_time; // The time between pulses

  // Update the last time
  last_time = current_time;

  // Calculate the RPM
  rpm = (60 * 1000000) / (pulse_time * PULSES_PER_REV); // Convert the pulse time to RPM

  // Update the cam angle based on the RPM
  cam_angle += (360 * pulse_time * RPM) / (60 * 1000000); // Add the angular displacement to the cam angle

  // Wrap around if necessary
  if (cam_angle >= 360) {
    cam_angle -= 360;
  }

  // Check if the sensor angle matches
  if (cam_angle == SENSOR_ANGLE) {
    // Check if the cylinder number matches
    if (CYLINDER_NUMBER == 1) {
      // Set the injection flag
      injection_flag = true;
    }

    // Increment the cylinder number
    CYLINDER_NUMBER++;

    // Wrap around if necessary
    if (CYLINDER_NUMBER > 4) {
      CYLINDER_NUMBER = 1;
    }
  }
}
