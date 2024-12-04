#include <EEPROM.h> // Include EEPROM library for ESP32

// Pin configuration
const int currentSensorPin = 39; // ADC pin connected to the WCS sensor output

// Sensor configuration
const float sensitivity = 50.0;    // Sensitivity in mV/A for WCS1800
const float voltageReference = 3.3; // ESP32 ADC reference voltage
const int adcResolution = 4096;    // ESP32 ADC resolution (12-bit)

// EEPROM configuration
const int eepromSize = 512;         // Total EEPROM size
const int offsetAddress = 0;        // EEPROM address for storing the offset ideal 1.816

// Sampling configuration
const int numSamples = 50;          // Number of samples to average
float offsetVoltage = 0.0;          // Offset voltage for the sensor
float current = 0.0;                // Measured current

void setup() {
  Serial.begin(115200);
  delay(1000); // Allow some time for the serial monitor to start
  Serial.println("Single Sensor Current Measurement with EEPROM Calibration");

  // Initialize EEPROM
  if (!EEPROM.begin(eepromSize)) {
    Serial.println("Failed to initialize EEPROM!");
    while (1);
  }

  // Load offset from EEPROM
  offsetVoltage = readCalibrationFromEEPROM();
  Serial.print("Loaded offset voltage: ");
  Serial.print(offsetVoltage, 3);
  Serial.println(" V");

  Serial.println("Send 'cal' to calibrate the sensor.");
}

void loop() {
  // Check for serial input
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim(); // Remove any leading/trailing whitespace

    if (command == "cal") {
      calibrateZeroCurrent();
    }
  }

  // Measure current
  current = measureCurrent();
  Serial.print("Current: ");
  Serial.print(current, 2); // Print with 2 decimal points
  Serial.println(" A");

  Serial.print("Offset Voltage: ");
  Serial.print(offsetVoltage, 3); // Print with 3 decimal points
  Serial.println(" V");

  delay(1000); // Wait for a second before the next reading
}

// Function to measure current
float measureCurrent() {
  float totalADC = 0.0;

  // Take multiple samples and accumulate the ADC readings
  for (int i = 0; i < numSamples; i++) {
    totalADC += analogRead(currentSensorPin);
    delayMicroseconds(100); // Small delay to allow ADC to stabilize
  }

  // Calculate the average ADC value
  float avgADC = totalADC / numSamples;

  // Convert average ADC value to voltage
  float sensorVoltage = (avgADC * voltageReference) / adcResolution;

  // Calculate current (sensitivity is in mV/A, so convert to V/A)
  return (sensorVoltage - offsetVoltage) / (sensitivity / 1000.0);
}

// Function to calibrate the zero-current offset
void calibrateZeroCurrent() {
  float totalADC = 0.0;

  // Take multiple samples with no current
  Serial.println("Calibrating zero-current offset...");
  for (int i = 0; i < numSamples; i++) {
    totalADC += analogRead(currentSensorPin);
    delay(10); // Short delay between samples
  }

  // Calculate average ADC value and determine offset voltage
  float avgADC = totalADC / numSamples;
  offsetVoltage = (avgADC * voltageReference) / adcResolution;

  Serial.print("Calibration complete. New zero-current offset: ");
  Serial.print(offsetVoltage, 3); // Print with 3 decimal points
  Serial.println(" V");

  // Save the new offset to EEPROM
  saveCalibrationToEEPROM(offsetVoltage);
}

// Save the calibration offset to EEPROM
void saveCalibrationToEEPROM(float offset) {
  uint32_t offsetData = *(uint32_t *)&offset; // Convert float to uint32_t for storage
  for (int i = 0; i < 4; i++) {
    EEPROM.write(offsetAddress + i, (offsetData >> (i * 8)) & 0xFF);
  }
  EEPROM.commit(); // Save changes to EEPROM
  Serial.println("Calibration offset saved to EEPROM.");
}

// Read the calibration offset from EEPROM
float readCalibrationFromEEPROM() {
  uint32_t offsetData = 0;
  for (int i = 0; i < 4; i++) {
    offsetData |= (EEPROM.read(offsetAddress + i) << (i * 8));
  }
  return *(float *)&offsetData; // Convert uint32_t back to float
}
