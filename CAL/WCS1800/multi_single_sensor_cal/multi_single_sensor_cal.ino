#include <EEPROM.h> // Include EEPROM library for ESP32

// Pin configuration
const int currentSensorPins[] = {39, 36, 34}; // ADC pins connected to the WCS sensor outputs
const int numSensors = 3;                    // Number of current sensors

// Sensor configuration
const float sensitivity = 66.0;    // Sensitivity in mV/A for WCS1800
const float voltageReference = 3.3; // ESP32 ADC reference voltage
const int adcResolution = 4096;    // ESP32 ADC resolution (12-bit)

// EEPROM configuration
const int eepromSize = 512;        // Total EEPROM size (ESP32 uses flash for this)
const int offsetAddresses[] = {0, 10, 20}; // EEPROM addresses for storing offsets for each sensor

// Sampling configuration
const int numSamples = 50;         // Number of samples to average
float offsetVoltages[numSensors];  // Offset voltages for each sensor
float currents[numSensors];        // Measured currents for each sensor

void setup() {
  Serial.begin(115200);
  delay(1000); // Allow some time for the serial monitor to start
  Serial.println("Multi-Sensor Current Measurement with EEPROM Calibration");

  // Initialize EEPROM
  if (!EEPROM.begin(eepromSize)) {
    Serial.println("Failed to initialize EEPROM!");
    while (1); // Halt the program if EEPROM initialization fails
  }

  // Load offsets from EEPROM
  for (int i = 0; i < numSensors; i++) {
    offsetVoltages[i] = readCalibrationFromEEPROM(i);
    Serial.print("Loaded offset voltage for sensor ");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.print(offsetVoltages[i], 3);
    Serial.println(" V");
  }

  Serial.println("Send 'cal0', 'cal1', or 'cal2' to calibrate sensors 1, 2, or 3 respectively.");
}

void loop() {
  // Check for serial input
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim(); // Remove any leading/trailing whitespace

    if (command == "cal0") {
      calibrateZeroCurrent(0);
    } else if (command == "cal1") {
      calibrateZeroCurrent(1);
    } else if (command == "cal2") {
      calibrateZeroCurrent(2);
    }
  }

  // Measure current for each sensor
  for (int i = 0; i < numSensors; i++) {
    currents[i] = measureCurrent(i);
    Serial.print("Sensor ");
    Serial.print(i + 1);
    Serial.print(" Current: ");
    Serial.print(currents[i], 2); // Print with 2 decimal points
    Serial.println(" A");
  }

  delay(1000); // Wait for a second before the next reading
}

// Function to measure current for a specific sensor
float measureCurrent(int sensorIndex) {
  float totalADC = 0.0;

  // Take multiple samples and accumulate the ADC readings
  for (int i = 0; i < numSamples; i++) {
    totalADC += analogRead(currentSensorPins[sensorIndex]);
    delayMicroseconds(100); // Small delay to allow ADC to stabilize (optional)
  }

  // Calculate the average ADC value
  float avgADC = totalADC / numSamples;

  // Convert average ADC value to voltage
  float sensorVoltage = (avgADC * voltageReference) / adcResolution;

  // Calculate current (sensitivity is in mV/A, so convert to V/A)
  return (sensorVoltage - offsetVoltages[sensorIndex]) / (sensitivity / 1000.0);
}

// Function to calibrate the zero-current offset for a specific sensor
void calibrateZeroCurrent(int sensorIndex) {
  float totalADC = 0.0;

  // Take multiple samples with no current
  Serial.print("Calibrating zero-current offset for sensor ");
  Serial.print(sensorIndex + 1);
  Serial.println("...");
  for (int i = 0; i < numSamples; i++) {
    totalADC += analogRead(currentSensorPins[sensorIndex]);
    delay(10); // Short delay between samples
  }

  // Calculate average ADC value and determine offset voltage
  float avgADC = totalADC / numSamples;
  offsetVoltages[sensorIndex] = (avgADC * voltageReference) / adcResolution;

  Serial.print("Calibration complete. New zero-current offset for sensor ");
  Serial.print(sensorIndex + 1);
  Serial.print(": ");
  Serial.print(offsetVoltages[sensorIndex], 3); // Print with 3 decimal points
  Serial.println(" V");

  // Save the new offset to EEPROM
  saveCalibrationToEEPROM(sensorIndex, offsetVoltages[sensorIndex]);
}

// Save the calibration offset to EEPROM for a specific sensor
void saveCalibrationToEEPROM(int sensorIndex, float offset) {
  int baseAddress = offsetAddresses[sensorIndex];
  uint32_t offsetData = *(uint32_t *)&offset; // Convert float to uint32_t for storage
  for (int i = 0; i < 4; i++) {
    EEPROM.write(baseAddress + i, (offsetData >> (i * 8)) & 0xFF);
  }
  EEPROM.commit(); // Save changes to EEPROM
  Serial.println("Calibration offset saved to EEPROM.");
}

// Read the calibration offset from EEPROM for a specific sensor
float readCalibrationFromEEPROM(int sensorIndex) {
  int baseAddress = offsetAddresses[sensorIndex];
  uint32_t offsetData = 0;
  for (int i = 0; i < 4; i++) {
    offsetData |= (EEPROM.read(baseAddress + i) << (i * 8));
  }
  return *(float *)&offsetData; // Convert uint32_t back to float
}
