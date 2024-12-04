#include "painlessMesh.h"

#define MESH_PREFIX "whateverYouLike"
#define MESH_PASSWORD "somethingSneaky"
#define MESH_PORT 5555

#include <EEPROM.h>  // Include EEPROM library for ESP32

// Pin configuration
const int currentSensorPins[] = { 39, 36, 34 };  // ADC pins connected to the WCS sensor outputs
const int numSensors = 3;                        // Number of current sensors

// Sensor configuration
const float sensitivity = 66.0;      // Sensitivity in mV/A for WCS1800
const float voltageReference = 3.3;  // ESP32 ADC reference voltage
const int adcResolution = 4096;      // ESP32 ADC resolution (12-bit)

// EEPROM configuration
const int eepromSize = 512;                   // Total EEPROM size (ESP32 uses flash for this)
const int offsetAddresses[] = { 0, 10, 20 };  // EEPROM addresses for storing offsets for each sensor

// Sampling configuration
const int numSamples = 50;         // Number of samples to average
float offsetVoltages[numSensors];  // Offset voltages for each sensor
float currents[numSensors];        // Measured currents for each sensor


#define THRESHOLD_OF_MAX 70  // Current difference threshold for detecting faults

Scheduler userScheduler;
painlessMesh mesh;

struct SensorData {
  float current1;
  float current2;
  float current3;
  String location;
  String status1;
  String status2;
  String status3;
};

SensorData data;
void sendData();

String msg;

#define POS_THRESHOLD 0.9

// Global variable to track the fault count
int faultCount = 0;

void detectFault(float sensor1, float sensor2, float sensor3) {
  // Calculate the maximum sensor value
  float max_sensor = (sensor1 > sensor2) ? ((sensor1 > sensor3) ? sensor1 : sensor3) : ((sensor2 > sensor3) ? sensor2 : sensor3);
  printf("max = %f\n", max_sensor);
  
  // Check if one sensor is positive while the other two are close to zero
  float zero_threshold = 0.08;  // Adjust this threshold for "close to zero" as needed
  if ((sensor1 > 0.2 && sensor2 < zero_threshold && sensor3 < zero_threshold) || 
      (sensor2 > 0.2 && sensor1 < zero_threshold && sensor3 < zero_threshold) || 
      (sensor3 > 0.2 && sensor1 < zero_threshold && sensor2 < zero_threshold)) {

    if (sensor1 > POS_THRESHOLD && sensor2 < zero_threshold && sensor3 < zero_threshold) {
      printf("Fault detected: Sensor 2 and Sensor 3 near zero while Sensor 1 is positive\n");
      msg = "String2 and String3 Fault";
      faultCount++;  // Increment fault count
    } else if (sensor2 > POS_THRESHOLD && sensor1 < zero_threshold && sensor3 < zero_threshold) {
      printf("Fault detected: Sensor 1 and Sensor 3 near zero while Sensor 2 is positive\n");
      msg = "String1 and String3 Fault";
      faultCount++;  // Increment fault count
    } else if (sensor3 > POS_THRESHOLD && sensor1 < zero_threshold && sensor2 < zero_threshold) {
      printf("Fault detected: Sensor 1 and Sensor 2 near zero while Sensor 3 is positive\n");
      msg = "String1 and String2 Fault";
      faultCount++;  // Increment fault count
    }
  } else if (max_sensor > 0.2) {
    float threshold = 0;
    // Threshold configuration
    if (max_sensor > 0.3 && max_sensor <= 0.5) {
      threshold = max_sensor * (float)(90 / 100.0);  // Set threshold
      printf("cat 1, threshold = %f\n", threshold);
    } else if (max_sensor > 0.5 && max_sensor <= 1.5) {
      threshold = max_sensor * (float)(85 / 100.0);  // Set threshold
      printf("cat 2, threshold = %f\n", threshold);
    } else if (max_sensor > 1.5 && max_sensor <= 5.0) {
      threshold = max_sensor * (float)(80 / 100.0);  // Set threshold
      printf("cat 3, threshold = %f\n", threshold);
    } else {
      threshold = max_sensor * (float)(THRESHOLD_OF_MAX / 100.0);  // Set threshold
      printf("normal threshold = %f\n", threshold);
    }

    // Calculate deviations directly without intermediate variables
    float dev1 = (sensor1 > (sensor2 + sensor3) / 2.0) ? (sensor1 - (sensor2 + sensor3) / 2.0) : ((sensor2 + sensor3) / 2.0 - sensor1);
    float dev2 = (sensor2 > (sensor1 + sensor3) / 2.0) ? (sensor2 - (sensor1 + sensor3) / 2.0) : ((sensor1 + sensor3) / 2.0 - sensor2);
    float dev3 = (sensor3 > (sensor1 + sensor2) / 2.0) ? (sensor3 - (sensor1 + sensor2) / 2.0) : ((sensor1 + sensor2) / 2.0 - sensor3);

    // Determine the sensor with the highest deviation above the threshold
    if (dev1 > threshold && dev1 > dev2 && dev1 > dev3) {
      printf("Fault detected in Sensor 1\n");
      msg = "String1 Fault\n" + String(sensor1, 1) + "A, " + String(sensor2, 1) + "A, " + String(sensor3, 1) + "A";
      faultCount++;  // Increment fault count

    } else if (dev2 > threshold && dev2 > dev1 && dev2 > dev3) {
      printf("Fault detected in Sensor 2\n");
      msg = "String2 Fault\n" + String(sensor1, 1) + "A, " + String(sensor2, 1) + "A, " + String(sensor3, 1) + "A";
      faultCount++;  // Increment fault count
    } else if (dev3 > threshold && dev3 > dev1 && dev3 > dev2) {
      printf("Fault detected in Sensor 3\n");
      msg = "String3 Fault\n" + String(sensor1, 1) + "A, " + String(sensor2, 1) + "A, " + String(sensor3, 1) + "A";
      faultCount++;  // Increment fault count
    } else {
      printf("No fault detected\n");
      msg = String(sensor1, 1) + "A, " + String(sensor2, 1) + "A, " + String(sensor3, 1) + "A";
    }
  } else {
    msg = String(sensor1, 1) + "A, " + String(sensor2, 1) + "A, " + String(sensor3, 1) + "A";
  }
  
  // Add the fault count to the message
  msg += "\nFault Count: " + String(faultCount);

  // Print the result with fault count
  String st = String(sensor1, 1) + "A, " + String(sensor2, 1) + "A, " + String(sensor3, 1) + "A";
  st += "\nFault Count: " + String(faultCount);
  Serial.println(st);
}

// void detectFault(float sensor1, float sensor2, float sensor3) {
//   // Calculate the maximum sensor value
//   float max_sensor = (sensor1 > sensor2) ? ((sensor1 > sensor3) ? sensor1 : sensor3) : ((sensor2 > sensor3) ? sensor2 : sensor3);
//   printf("max = %f\n", max_sensor);
//   // Check if one sensor is positive while the other two are close to zero
//   float zero_threshold = 0.08;  // Adjust this threshold for "close to zero" as needed
//   if ((sensor1 > 0.2 && sensor2 < zero_threshold && sensor3 < zero_threshold) || (sensor2 > 0.2 && sensor1 < zero_threshold && sensor3 < zero_threshold) || (sensor3 > 0.2 && sensor1 < zero_threshold && sensor2 < zero_threshold)) {

//     if (sensor1 > POS_THRESHOLD && sensor2 < zero_threshold && sensor3 < zero_threshold) {
//       printf("Fault detected: Sensor 2 and Sensor 3 near zero while Sensor 1 is positive\n");
//       msg = "String2 and String3 Fault";
//     } else if (sensor2 > POS_THRESHOLD && sensor1 < zero_threshold && sensor3 < zero_threshold) {
//       printf("Fault detected: Sensor 1 and Sensor 3 near zero while Sensor 2 is positive\n");
//       msg = "String1 and String3 Fault";
//     } else if (sensor3 > POS_THRESHOLD && sensor1 < zero_threshold && sensor2 < zero_threshold) {
//       printf("Fault detected: Sensor 1 and Sensor 2 near zero while Sensor 3 is positive\n");
//       msg = "String1 and String2 Fault";
//     }
//   } else if (max_sensor > 0.2) {
//     float threshold = 0;
//     //threshold configure
//     if (max_sensor > 0.3 && max_sensor <= 0.5) {
//       threshold = max_sensor * (float)(90 / 100.0);  // Set threshold
//       printf("cat 1, threshold = %f\n", threshold);
//     } else if (max_sensor > 0.5 && max_sensor <= 1.5) {
//       threshold = max_sensor * (float)(85 / 100.0);  // Set threshold
//       printf("cat 2, threshold = %f\n", threshold);
//     } else if (max_sensor > 1.5 && max_sensor <= 5.0) {
//       threshold = max_sensor * (float)(80 / 100.0);  // Set threshold
//       printf("cat 3, threshold = %f\n", threshold);
//     } else {
//       threshold = max_sensor * (float)(THRESHOLD_OF_MAX / 100.0);  // Set threshold
//       printf("normal threshold = %f\n", threshold);
//     }

//     // Calculate deviations directly without intermediate variables
//     float dev1 = (sensor1 > (sensor2 + sensor3) / 2.0) ? (sensor1 - (sensor2 + sensor3) / 2.0) : ((sensor2 + sensor3) / 2.0 - sensor1);
//     float dev2 = (sensor2 > (sensor1 + sensor3) / 2.0) ? (sensor2 - (sensor1 + sensor3) / 2.0) : ((sensor1 + sensor3) / 2.0 - sensor2);
//     float dev3 = (sensor3 > (sensor1 + sensor2) / 2.0) ? (sensor3 - (sensor1 + sensor2) / 2.0) : ((sensor1 + sensor2) / 2.0 - sensor3);

//     // Determine the sensor with the highest deviation above the threshold
//     if (dev1 > threshold && dev1 > dev2 && dev1 > dev3) {
//       printf("Fault detected in Sensor 1\n");
//       msg = "String1 Fault\n" + String(sensor1, 1) + "A, " + String(sensor2, 1) + "A, " + String(sensor3, 1) + "A";

//     } else if (dev2 > threshold && dev2 > dev1 && dev2 > dev3) {
//       printf("Fault detected in Sensor 2\n");
//       msg = "String2 Fault\n" + String(sensor1, 1) + "A, " + String(sensor2, 1) + "A, " + String(sensor3, 1) + "A";
//     } else if (dev3 > threshold && dev3 > dev1 && dev3 > dev2) {
//       printf("Fault detected in Sensor 3\n");
//       msg = "String3 Fault\n" + String(sensor1, 1) + "A, " + String(sensor2, 1) + "A, " + String(sensor3, 1) + "A";
//     } else {
//       printf("No fault detected\n");
//       msg = String(sensor1, 1) + "A, " + String(sensor2, 1) + "A, " + String(sensor3, 1) + "A";
//     }
//   } else {
//     msg = String(sensor1, 1) + "A, " + String(sensor2, 1) + "A, " + String(sensor3, 1) + "A";
//   }
//   String st = String(sensor1, 1) + "A, " + String(sensor2, 1) + "A, " + String(sensor3, 1) + "A";
//   Serial.println(st);
// }


// data.status1 = "OK";
// data.status2 = "OK";
// data.status3 = "OK";



Task taskSendData(TASK_SECOND * 5, TASK_FOREVER, &sendData);
void sendData() {

  data.current1 = measureCurrent(0);
  data.current2 = measureCurrent(1);
  data.current3 = measureCurrent(2);
  // Check and set any current value less than 0 to 0
  if (data.current1 < 0) {
    data.current1 = 0;
  }

  if (data.current2 < 0) {
    data.current2 = 0;
  }

  if (data.current3 < 0) {
    data.current3 = 0;
  }

  data.location = "Node" + String(mesh.getNodeId());

  // Detect faults between sensors
  detectFault(data.current1, data.current2, data.current3);

  // Create a JSON string for sending
  //String msg = String("{\"s1\":") + data.current1 + String(",\"s2\":") + data.current2 + String(",\"s3\":") + data.current3 + String(",\"loc\":\"") + data.location + String("\",\"status1\":\"") + data.status1 + String("\",\"status2\":\"") + data.status2 + String("\",\"status3\":\"") + data.status3 + "\"}";

  mesh.sendBroadcast(msg);
  Serial.printf("Sent: %s\n", msg.c_str());
}

void newConnectionCallback(uint32_t nodeId) {
  Serial.printf("New connection established with node %u\n", nodeId);
}

void changedConnectionCallback() {
  Serial.println("Changed connections detected");
}

void setup() {
  Serial.begin(115200);
  delay(1000);  // Allow some time for the serial monitor to start
  Serial.println("Multi-Sensor Current Measurement with EEPROM Calibration");

  // Initialize EEPROM
  if (!EEPROM.begin(eepromSize)) {
    Serial.println("Failed to initialize EEPROM!");
    while (1)
      ;  // Halt the program if EEPROM initialization fails
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

  Serial.println("Send 'cal0' to calibrate sensors r.");
  // mesh.setDebugMsgTypes(ERROR | STARTUP | CONNECTION | COMMUNICATION);
  mesh.setDebugMsgTypes(ERROR);

  mesh.init(MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT);
  mesh.onNewConnection(newConnectionCallback);
  mesh.onChangedConnections(changedConnectionCallback);
  userScheduler.addTask(taskSendData);
  taskSendData.enable();

  analogReadResolution(12);  // ESP32 uses 12-bit ADC resolution by default
  delay(2000);               // Delay to ensure all nodes are connected
}

void loop() {
  mesh.update();

  // Check for serial input
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();  // Remove any leading/trailing whitespace

    if (command == "cal") {
      int totalSensor = sizeof(currentSensorPins) / sizeof(currentSensorPins[0]);
      for (int i = 0; i < totalSensor; i++) {
        calibrateZeroCurrent(i);
      }
    }
  }
}
//MESH RELATED CODE
//SENSIN CODE

// Function to measure current for a specific sensor
float measureCurrent(int sensorIndex) {
  float totalADC = 0.0;

  // Take multiple samples and accumulate the ADC readings
  for (int i = 0; i < numSamples; i++) {
    totalADC += analogRead(currentSensorPins[sensorIndex]);
    delayMicroseconds(100);  // Small delay to allow ADC to stabilize (optional)
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
    delay(10);  // Short delay between samples
  }

  // Calculate average ADC value and determine offset voltage
  float avgADC = totalADC / numSamples;
  offsetVoltages[sensorIndex] = (avgADC * voltageReference) / adcResolution;

  Serial.print("Calibration complete. New zero-current offset for sensor ");
  Serial.print(sensorIndex + 1);
  Serial.print(": ");
  Serial.print(offsetVoltages[sensorIndex], 3);  // Print with 3 decimal points
  Serial.println(" V");

  // Save the new offset to EEPROM
  saveCalibrationToEEPROM(sensorIndex, offsetVoltages[sensorIndex]);
}

// Save the calibration offset to EEPROM for a specific sensor
void saveCalibrationToEEPROM(int sensorIndex, float offset) {
  int baseAddress = offsetAddresses[sensorIndex];
  uint32_t offsetData = *(uint32_t *)&offset;  // Convert float to uint32_t for storage
  for (int i = 0; i < 4; i++) {
    EEPROM.write(baseAddress + i, (offsetData >> (i * 8)) & 0xFF);
  }
  EEPROM.commit();  // Save changes to EEPROM
  Serial.println("Calibration offset saved to EEPROM.");
}

// Read the calibration offset from EEPROM for a specific sensor
float readCalibrationFromEEPROM(int sensorIndex) {
  int baseAddress = offsetAddresses[sensorIndex];
  uint32_t offsetData = 0;
  for (int i = 0; i < 4; i++) {
    offsetData |= (EEPROM.read(baseAddress + i) << (i * 8));
  }
  return *(float *)&offsetData;  // Convert uint32_t back to float
}
