#include "main.h"
#include "app_mros_range_pub.h"
#include "app_tag_ranging.h"

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_WIFI)
#error This example is only avaliable for Arduino framework with wifi transport.
#endif


IPAddress mRos_Agent_IP(MROS_AGENT_IP);
uint16_t mRos_Agent_Port = MROS_AGENT_PORT;

QueueHandle_t ranging_queue;

void setup() {
  // Initialize Serial
  Serial.begin(115200);
  while(!Serial); // Wait for Serial to be ready
  delay(1000);

  LOG_SET_LEVEL(DEBUG_LOG_LEVEL); 

  // Initialize SPI
  SPI.begin(DW_SPI_SCK, DW_SPI_MISO, DW_SPI_MOSI, DW_SPI_CS);

  set_microros_wifi_transports((char *)WIFI_SSID, (char *)WIFI_PSK, mRos_Agent_IP, mRos_Agent_Port);
  delay(2000);

  app_mros_range_pub_init();

  app_tag_ranging_init();

  // thread setup
  ranging_queue = xQueueCreate(10, sizeof(float) * TAG_NUMBER_OF_ANCHOR_MAX);

  xTaskCreatePinnedToCore(
    app_tag_ranging_task,   // Function to implement the task
    "app_tag_ranging_task", // Name of the task
    4096,                   // Stack size in words
    NULL,                   // Task input parameter    
    5,                      // Priority of the task  
    NULL,                   // Task handle.
    1                       // Core where the task should run
  );

  xTaskCreatePinnedToCore(
    app_mros_range_pub_task,   // Function to implement the task
    "app_mros_range_pub_task", // Name of the task
    4096,                      // Stack size in words
    NULL,                      // Task input parameter    
    5,                         // Priority of the task  
    NULL,                      // Task handle.
    1                          // Core where the task should run
  );

}

void loop() {
    delay(10);
}