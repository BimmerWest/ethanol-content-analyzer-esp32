#pragma once

#include "Arduino.h"
#include "driver/twai.h"

// I/O
#define ECA_INPUT                 4             // Flex fuel sensor signal input (use voltage divider from 5V!)
#define CAN_TX_PIN                5             // TWAI TX -> CAN transceiver TX
#define CAN_RX_PIN                6             // TWAI RX -> CAN transceiver RX

// constants
#define FREQUENCY_ALPHA           0.01f
#define MAX_FREQUENCY             200.f         // Hz

#define E0_FREQUENCY              50.f          // Hz
#define E100_FREQUENCY            150.f         // Hz
#define ETHANOL_FREQUENCY_SCALER  ((E100_FREQUENCY - E0_FREQUENCY) / 100.0f)

// Temperature
#define TEMP_MIN                  -40.0f        // °C
#define TEMP_MAX                  125.0f        // °C

// Zeitronix ECA-2 CAN Bus defaults
#define ZEITRONIX_CAN_ID          0x00EC        // Default standard 11-bit CAN ID
#define ZEITRONIX_CAN_SPEED       TWAI_TIMING_CONFIG_500KBITS()  // 500 Kbps
#define ZEITRONIX_CAN_INTERVAL_MS 250           // 4 Hz update rate
#define ZEITRONIX_SENSOR_OK       0x00
#define ZEITRONIX_SENSOR_FAULT    0x01

// Sensor timeout
#define SENSOR_TIMEOUT_MS         1000          // No updates for 1s => FAULT

// State variables
extern float ethanol;
extern float fuelTemperature;
extern bool canReady;
extern uint32_t lastSensorUpdateMs;

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

void initCAN();
void sendZeitronixCANMessage();
