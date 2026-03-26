#pragma once

#include "Arduino.h"

// I/O
#define ECA_INPUT                 4             // Flex fuel sensor signal input (use voltage divider from 5V!)

// constants
#define FREQUENCY_ALPHA           0.01f
#define MAX_FREQUENCY             200.f         // Hz

#define E0_FREQUENCY              50.f          // Hz
#define E100_FREQUENCY            150.f         // Hz
#define ETHANOL_FREQUENCY_SCALER  ((E100_FREQUENCY - E0_FREQUENCY) / 100.0f)

// Temperature
#define TEMP_MIN                  -40.0f        // °C
#define TEMP_MAX                  125.0f        // °C

// State variables
extern float ethanol;
extern float fuelTemperature;

// Frequency and duty cycle readings
extern float frequency, dutyCycle;
extern const float frequencyScaler;

// ISR shared variables
extern volatile uint32_t risingEdgeTime;
extern volatile uint32_t fallingEdgeTime;
extern volatile uint32_t period;
extern volatile float rawDutyCycle;
extern volatile bool newData;

// Function declarations
bool calculateFrequency();
void frequencyToEthanolContent(float frequency, float scaler);
void dutyCycleToFuelTemperature(float dutyCycle);

void IRAM_ATTR onSensorEdge();
