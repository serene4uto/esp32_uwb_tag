#ifndef __MAIN_H__
#define __MAIN_H__

#include <stdio.h>

#include <Arduino.h>
#include <SPI.h>

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uwb_interfaces/msg/uwb_range.h>

#include "DebugLog.h"
#include "DW1000.h"
#include "DW1000Ranging.h"

// Program Options
#define UWB_RANGE_PUB_ENABLED
// #define UWB_LOCALIZATION_PUB_ENABLED

// Debug Options
#define DEBUG_PRINT_RECEIVED_ANCHOR_RANGE
// #define DEBUG_PRINT_TAG_POSITION
#define DEBUG_LOG_LEVEL DebugLogLevel::LVL_INFO


// connection pins
#define DW_SPI_SCK 18
#define DW_SPI_MISO 19
#define DW_SPI_MOSI 23
#define DW_SPI_CS 4

#define DW_PIN_RST 27
#define DW_PIN_IRQ 34


// WiFi info
#define WIFI_SSID "serene_2G"
#define WIFI_PSK "10541054"

// Tag info
#define TAG_ADDRESS_STR "7D:01:22:EA:82:60:3B:9C"
#define TAG_OPERATION_MODE DW1000.MODE_LONGDATA_RANGE_ACCURACY
#define TAG_TIMER_DELAY 80
#define TAG_MAX_DISTANCE 30.0 // meters


// Ranging Network info
#define TAG_NUMBER_OF_ANCHOR_MAX 4
#define TAG_NUMBER_OF_ANCHOR_MIN 4
#define TAG_ANCHOR_NETWORK_ID 0x84
#define TAG_ANCHOR_RANGE_EXPIRE_MS 5000 // milliseconds

// #define RANGING_NETWORK_ANCHOR_ADDRESSES { \
//     {0x84, 0x01, 0x22, 0xEA, 0x82, 0x60, 0x3B, 0x9C}, \
//     {0x84, 0x02, 0x22, 0xEA, 0x82, 0x60, 0x3B, 0x9C}, \
//     {0x84, 0x03, 0x22, 0xEA, 0x82, 0x60, 0x3B, 0x9C}, \
//     {0x84, 0x04, 0x22, 0xEA, 0x82, 0x60, 0x3B, 0x9C}, \
//     {0x84, 0x05, 0x22, 0xEA, 0x82, 0x60, 0x3B, 0x9C}, \
//     {0x84, 0x06, 0x22, 0xEA, 0x82, 0x60, 0x3B, 0x9C}, \
//     {0x84, 0x07, 0x22, 0xEA, 0x82, 0x60, 0x3B, 0x9C}, \
//     {0x84, 0x08, 0x22, 0xEA, 0x82, 0x60, 0x3B, 0x9C}  \
// }




// mRos info
#define MROS_AGENT_IP {192,168,1,3}
#define MROS_AGENT_PORT 8888
#define MROS_RANGE_PUB_TOPIC_NAME "uros_esp32_uwb_tag_range"
#define MROS_RANGE_PUB_NAMESPACE ""
#define MROS_TAG_POSITION_PUB_TOPIC_NAME "uros_esp32_uwb_tag_position"


extern QueueHandle_t ranging_queue;


#endif // __MAIN_H__