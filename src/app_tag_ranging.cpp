#include "app_tag_ranging.h"
#include "main.h"

union {
    struct 
    {
        uint8_t network_id : 8;
        uint8_t device_id : 8;
    };
    uint16_t short_address;
} anchor_address;

uint8_t anchor_received_count = 0;
float current_anchor_range = 0;
uint32_t last_anchor_update_time[TAG_NUMBER_OF_ANCHOR_MAX] = {0}; 
float last_anchor_distance[TAG_NUMBER_OF_ANCHOR_MAX] = {0.0}; //last distance from anchor

static void new_range_cb() {
  anchor_received_count = 0;
  anchor_address.short_address = DW1000Ranging.getDistantDevice()->getShortAddress();
  current_anchor_range = DW1000Ranging.getDistantDevice()->getRange();

  // check if anchor is in the accepted condition
  if (anchor_address.network_id != TAG_ANCHOR_NETWORK_ID ) {
    LOG_INFO("Anchor not in acceptable network: ", String(anchor_address.short_address, HEX));
    return;
  }
  if (anchor_address.device_id < 0 || anchor_address.device_id > TAG_NUMBER_OF_ANCHOR_MAX) {
    LOG_INFO("Anchor not in available device range: ", String(anchor_address.short_address, HEX));
    return;
  }

  last_anchor_update_time[anchor_address.device_id-1] = millis();  //(-1) => array index
  float range = DW1000Ranging.getDistantDevice()->getRange();
  last_anchor_distance[anchor_address.device_id-1] = range;
  if (range < 0.0 || range > TAG_MAX_DISTANCE) {
    last_anchor_update_time[anchor_address.device_id-1] = 0;  //sanity check, ignore this measurement
  }

  for (int i = 0; i < TAG_NUMBER_OF_ANCHOR_MAX; i++) {
    if (millis() - last_anchor_update_time[i] > TAG_ANCHOR_RANGE_EXPIRE_MS) {
      last_anchor_update_time[i] = 0;  //expired, not from this one
    }

    if (last_anchor_update_time[i] > 0) {
      anchor_received_count++;
    }
  }
  if( (anchor_received_count == TAG_NUMBER_OF_ANCHOR_MAX)) { 
    // send to queue
    xQueueSend(ranging_queue, last_anchor_distance, portMAX_DELAY);
  }
}

static void new_anchor_cb(DW1000Device *device) {
  LOG_INFO("Device added: ", String(device->getShortAddress(), HEX));
}

static void inactive_anchor_cb(DW1000Device *device) {
  LOG_INFO("Inactive device removed: ", String(device->getShortAddress(), HEX));
}

void app_tag_ranging_init() {
  // Initialize DW1000
  DW1000Ranging.initCommunication(DW_PIN_RST, DW_SPI_CS, DW_PIN_IRQ);
  DW1000Ranging.attachNewRange(new_range_cb);
  DW1000Ranging.attachNewDevice(new_anchor_cb);
  DW1000Ranging.attachInactiveDevice(inactive_anchor_cb);
  DW1000Ranging.startAsTag((char *)TAG_ADDRESS_STR, TAG_OPERATION_MODE, false);

}

void app_tag_ranging_task(void *pvParameters) {
  while(1)
  {
    DW1000Ranging.loop();
  }
}
