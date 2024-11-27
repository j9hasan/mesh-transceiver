#include "painlessMesh.h"



#define MESH_PREFIX "whateverYouLike"
#define MESH_PASSWORD "somethingSneaky"
#define MESH_PORT 5555



Scheduler userScheduler;  // to control your other task
painlessMesh mesh;

struct SensorData {
  float temperature;
  int humidity;
  String location;
};

SensorData data;
void sendData();

Task taskSendData(TASK_SECOND * 5, TASK_FOREVER, &sendData);
void sendData() {
  // Populate the struct with example data
  data.temperature = random(15, 30);  // Example temperature value
  data.humidity = random(30, 70);     // Example humidity value
  data.location = "Node" + String(mesh.getNodeId());

  // Create a JSON string for sending
  String msg = String("{\"temperature\":") + data.temperature + String(",\"humidity\":") + data.humidity + String(",\"location\":\"") + data.location + "\"}";

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
  mesh.setDebugMsgTypes(ERROR | STARTUP | CONNECTION | COMMUNICATION);

  // mesh.setDebugMsgTypes(ERROR | STARTUP);  // set before init() so that you can see startup messages
  mesh.init(MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT);
  mesh.onNewConnection(newConnectionCallback);
  mesh.onChangedConnections(changedConnectionCallback);
  userScheduler.addTask(taskSendData);
  // userScheduler.addTask(TASK_SECOND * 1, TASK_FOREVER, );

  // userScheduler.addTask( taskSendMessage );
  taskSendData.enable();
  delay(2000);  // Delay to ensure all nodes are connected
}

void loop() {
  // userScheduler.execute();
  mesh.update();
}
