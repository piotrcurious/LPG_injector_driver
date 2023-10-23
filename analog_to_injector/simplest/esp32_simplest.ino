// Define the digital pin for detecting rising edge
#define RISING_EDGE_PIN 2

// Define the analog pin for determining injector open time
#define INJECTOR_OPEN_TIME_PIN A0

// Define the peak current in mA
#define PEAK_CURRENT 4000

// Define the hold current in mA
#define HOLD_CURRENT 1000

// Define the peak time in us
#define PEAK_TIME 2000

// Define the PWM frequency in Hz
#define PWM_FREQ 1000

// Define the PWM resolution in bits
#define PWM_RES 8

// Define the PWM channel for injector control
#define PWM_CHANNEL 0

// Calculate the peak duty cycle in %
#define PEAK_DUTY (PEAK_CURRENT * 255 / 3300)

// Calculate the hold duty cycle in %
#define HOLD_DUTY (HOLD_CURRENT * 255 / 3300)

// Declare a variable to store the injector open time in us
int injector_open_time = 0;

// Declare a variable to store the state of the rising edge pin
int rising_edge_state = LOW;

// Declare a variable to store the previous state of the rising edge pin
int previous_rising_edge_state = LOW;

void setup() {
  // Set the rising edge pin as input
  pinMode(RISING_EDGE_PIN, INPUT);

  // Set the PWM frequency and resolution
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RES);

  // Attach the PWM channel to the injector control pin
  ledcAttachPin(5, PWM_CHANNEL);
}

void loop() {
  // Read the state of the rising edge pin
  rising_edge_state = digitalRead(RISING_EDGE_PIN);

  // Check if the rising edge pin has changed from low to high
  if (rising_edge_state == HIGH && previous_rising_edge_state == LOW) {
    // Read the analog value of the injector open time pin
    int analog_value = analogRead(INJECTOR_OPEN_TIME_PIN);

    // Map the analog value to a range of 0 to 100000 us
    injector_open_time = map(analog_value, 0, 4095, 0, 100000);

    // Start the injection by setting the peak duty cycle
    ledcWrite(PWM_CHANNEL, PEAK_DUTY);

    // Wait for the peak time using udelay function
    udelay(PEAK_TIME);

    // Switch to the hold duty cycle
    ledcWrite(PWM_CHANNEL, HOLD_DUTY);

    // Wait for the remaining injector open time minus the peak time using udelay function
    udelay(injector_open_time - PEAK_TIME);

    // Stop the injection by setting the duty cycle to zero
    ledcWrite(PWM_CHANNEL, 0);
  }

  // Update the previous state of the rising edge pin
  previous_rising_edge_state = rising_edge_state;
}
