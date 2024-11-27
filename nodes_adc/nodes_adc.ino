#include "painlessMesh.h"

#define MESH_PREFIX "whateverYouLike"
#define MESH_PASSWORD "somethingSneaky"
#define MESH_PORT 5555

#define ADC_PIN_1 36     // First ADC pin S_VP (String 1)
#define ADC_PIN_2 39     // Second ADC pin S_VN (String 2)
#define ADC_PIN_3 34     // Third ADC pin (String 3)
#define NUM_SAMPLES 100  // Number of samples for averaging

// Calibration points
#define ADC_NO_CURRENT 2080  // ADC value when no current flows
#define S2OFFSET ADC_NO_CURRENT - (ADC_NO_CURRENT-53)
#define S3OFFSET ADC_NO_CURRENT - (ADC_NO_CURRENT+13)
#define ADC_CURRENT_5A 2375     // ADC value when 5A flows
#define ADC_CURRENT_5AS23 2401  // ADC value when 5A flows

// #define ADC_NO_CURRENT 2080  // ADC value when no current flows
// #define S2OFFSET ADC_NO_CURRENT - 2027
// #define S3OFFSET ADC_NO_CURRENT - 2093
// #define ADC_CURRENT_5A 2375     // ADC value when 5A flows
// #define ADC_CURRENT_5AS23 2401  // ADC value when 5A flows

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

float get_current(int adc_value, int pin) {
  // Convert the ADC value to current using linear interpolation
  float current = 0;
  if (pin == 36) {
    current = (5.00 * (adc_value - ADC_NO_CURRENT)) / (ADC_CURRENT_5A - ADC_NO_CURRENT);
    // Serial.print("pin39: ");
    // Serial.println(avg_adc_value);
  } else {
    current = (5.00 * (adc_value - ADC_NO_CURRENT)) / (ADC_CURRENT_5AS23 - ADC_NO_CURRENT);
  }
  return current;
}

float read_average_current(int pin) {
  int total_adc_value = 0;

  // Take multiple ADC readings and calculate the average
  for (int i = 0; i < NUM_SAMPLES; i++) {
    int adc_value = analogRead(pin);
    total_adc_value += adc_value;
    delay(10);  // Small delay between samples
  }

  // Calculate the average ADC value
  int avg_adc_value = total_adc_value / NUM_SAMPLES;

  if (pin == ADC_PIN_2) {
    avg_adc_value += S2OFFSET;
    // Serial.print("pin39: ");
    // Serial.println(avg_adc_value);
  }
  if (pin == ADC_PIN_3) {
    avg_adc_value += S3OFFSET;
    // Serial.println(avg_adc_value);
  }
  Serial.println(avg_adc_value);
  return get_current(avg_adc_value, pin);
}

String msg;

#define POS_THRESHOLD 0.9

void detectFault(float sensor1, float sensor2, float sensor3) {
  // Calculate the maximum sensor value
  float max_sensor = (sensor1 > sensor2) ? ((sensor1 > sensor3) ? sensor1 : sensor3) : ((sensor2 > sensor3) ? sensor2 : sensor3);
  printf("max = %f\n", max_sensor);
  // Check if one sensor is positive while the other two are close to zero
  float zero_threshold = 0.05;  // Adjust this threshold for "close to zero" as needed
  if ((sensor1 > 0.2 && sensor2 < zero_threshold && sensor3 < zero_threshold) || (sensor2 > 0.2 && sensor1 < zero_threshold && sensor3 < zero_threshold) || (sensor3 > 0.2 && sensor1 < zero_threshold && sensor2 < zero_threshold)) {

    if (sensor1 > POS_THRESHOLD && sensor2 < zero_threshold && sensor3 < zero_threshold) {
      printf("Fault detected: Sensor 2 and Sensor 3 near zero while Sensor 1 is positive\n");
      msg = "String2 and String3 Fault";
    } else if (sensor2 > POS_THRESHOLD && sensor1 < zero_threshold && sensor3 < zero_threshold) {
      printf("Fault detected: Sensor 1 and Sensor 3 near zero while Sensor 2 is positive\n");
      msg = "String1 and String3 Fault";
    } else if (sensor3 > POS_THRESHOLD && sensor1 < zero_threshold && sensor2 < zero_threshold) {
      printf("Fault detected: Sensor 1 and Sensor 2 near zero while Sensor 3 is positive\n");
      msg = "String1 and String2 Fault";
    }
  } else if (max_sensor > 0.2) {
    float threshold = 0;
    //threshold configure
    if (max_sensor > 0.2 && max_sensor <= 0.5) {
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
      msg = "String1 Fault";
    } else if (dev2 > threshold && dev2 > dev1 && dev2 > dev3) {
      printf("Fault detected in Sensor 2\n");
      msg = "String2 Fault";
    } else if (dev3 > threshold && dev3 > dev1 && dev3 > dev2) {
      printf("Fault detected in Sensor 3\n");
      msg = "String3 Fault";
    } else {
      printf("No fault detected\n");
      msg = String(sensor1, 1) + "A, " + String(sensor2, 1) + "A, " + String(sensor3, 1) + "A";
    }
  } else {
    msg = String(sensor1, 1) + "A, " + String(sensor2, 1) + "A, " + String(sensor3, 1) + "A";
  }
  String st = String(sensor1, 1) + "A, " + String(sensor2, 1) + "A, " + String(sensor3, 1) + "A";
  Serial.println(st);
}


// data.status1 = "OK";
// data.status2 = "OK";
// data.status3 = "OK";

Task taskSendData(TASK_SECOND * 5, TASK_FOREVER, &sendData);
void sendData() {
  // Read average currents from all ADC pins
  data.current1 = read_average_current(ADC_PIN_1);  // String 1
  data.current2 = read_average_current(ADC_PIN_2);  // String 2
  data.current3 = read_average_current(ADC_PIN_3);  // String 3
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
}
