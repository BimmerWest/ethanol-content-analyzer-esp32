#include "main.h"

// State variables
float ethanol = 0.f;
float fuelTemperature = 0.f;

// Frequency and duty cycle readings
float frequency = 0.f, dutyCycle = 0.f;
const float frequencyScaler = ETHANOL_FREQUENCY_SCALER;

// ISR shared variables
volatile uint32_t risingEdgeTime = 0;
volatile uint32_t fallingEdgeTime = 0;
volatile uint32_t period = 0;
volatile float rawDutyCycle = 0;
volatile bool newData = false;

void setup()
{
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  // Setup GPIO interrupt for sensor input
  pinMode(ECA_INPUT, INPUT);
  attachInterrupt(digitalPinToInterrupt(ECA_INPUT), onSensorEdge, CHANGE);

  Serial.println("Ethanol Content Analyzer - ESP32-C3");
  Serial.println("Waiting for sensor data...");
}

void loop()
{
  // get ethanol content from sensor frequency
  if (newData && calculateFrequency()) {
    frequencyToEthanolContent(frequency, frequencyScaler);
    dutyCycleToFuelTemperature(dutyCycle);
    newData = false;

    Serial.print("Period (us): ");
    Serial.print(period);
    Serial.print("\tFrequency: ");
    Serial.print(frequency, 1);
    Serial.print(" Hz");
    Serial.print("\tEthanol: ");
    Serial.print(ethanol, 1);
    Serial.print("%");
    Serial.print("\tDuty Cycle: ");
    Serial.print(dutyCycle, 1);
    Serial.print("%");
    Serial.print("\tFuel Temp: ");
    Serial.print(fuelTemperature, 1);
    Serial.println(" C");
  }
}

/**
 * GPIO interrupt handler - fires on both rising and falling edges
 * Measures period (rising-to-rising) and pulse width (rising-to-falling)
 * to get both frequency (ethanol %) and duty cycle (fuel temperature)
 */
void IRAM_ATTR onSensorEdge() {
  uint32_t now = micros();
  bool level = digitalRead(ECA_INPUT);

  if (level) {
    // Rising edge: calculate period from last rising edge
    if (risingEdgeTime > 0) {
      period = now - risingEdgeTime;

      // Calculate duty cycle from the previous pulse width
      if (fallingEdgeTime > risingEdgeTime) {
        uint32_t highTime = fallingEdgeTime - risingEdgeTime;
        rawDutyCycle = (float)highTime / (float)period * 100.0f;
      }

      newData = true;
    }
    risingEdgeTime = now;
  } else {
    // Falling edge: record time for pulse width calculation
    fallingEdgeTime = now;
  }
}

/**
 * Calculates frequency using the period measured by GPIO interrupts
 * @return boolean - true if valid frequency
 */
bool calculateFrequency() {
  uint32_t capturedPeriod = period;
  if (capturedPeriod == 0)
    return false;

  float tempFrequency = 1000000.f / (float)capturedPeriod; // period is in microseconds

  if (tempFrequency < 0 || tempFrequency > MAX_FREQUENCY)
    return false;

  // Capture duty cycle from ISR
  dutyCycle = rawDutyCycle;

  // if we haven't calculated frequency, use the current frequency.
  // Otherwise, run it through an exponential filter to smooth readings
  if (frequency == 0)
    frequency = tempFrequency;
  else
    frequency = (1 - FREQUENCY_ALPHA) * frequency + FREQUENCY_ALPHA * tempFrequency;

  return true;
}

/**
 * Converts a sensor frequency to ethanol percentage
 * Ethanol % = Frequency (in Hz) - 50.0.
 * A value of 180 Hz - 190 Hz indicates contaminated fuel.
 * @param frequency - Input frequency (1 / period)
 * @param scaler - Value by which interpolate frequencies between E0 and E100
 */
void frequencyToEthanolContent(float frequency, float scaler) {
  ethanol = (frequency - E0_FREQUENCY) / scaler;

  // bound ethanol content by a max of 0 to 100
  ethanol = max(0.0f, min(100.0f, ethanol));
}

/**
 * Converts the duty cycle of the sensor signal to fuel temperature
 * The Continental flex fuel sensor encodes temperature in the pulse width:
 * Duty cycle maps linearly from -40°C to 125°C across the 10%-90% range
 * @param dutyCycle - Duty cycle percentage (0-100)
 */
void dutyCycleToFuelTemperature(float dutyCycle) {
  // Clamp duty cycle to valid range
  float clampedDuty = max(10.0f, min(90.0f, dutyCycle));

  // Linear interpolation: 10% duty = -40°C, 90% duty = 125°C
  fuelTemperature = TEMP_MIN + (clampedDuty - 10.0f) * (TEMP_MAX - TEMP_MIN) / 80.0f;
}
