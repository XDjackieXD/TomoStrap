#include <pebble.h>
#include "common.h"
#include "main_window.h"

static const SmartstrapServiceId s_service_id = SMARTSTRAP_RAW_DATA_SERVICE_ID;
static const SmartstrapAttributeId s_attribute_id = SMARTSTRAP_RAW_DATA_ATTRIBUTE_ID;
static SmartstrapAttribute *s_attribute;
static bool s_service_available = false;

int32_t pulse = -1;
int32_t oxygen = -1;

#define KEY_OXYGEN 0x00
#define KEY_IMPEDANCE 'i'

// ----- Smartstrap -----

static void strap_availability_handler(SmartstrapServiceId service_id, bool is_available) {
  // A service's availability has changed
  APP_LOG(APP_LOG_LEVEL_INFO, "Service %d is %s available", (int)service_id, is_available ? "now" : "NOT");

  // Remember if the raw service is available
  s_service_available = (is_available && service_id == s_service_id);
  
  if(!s_service_available){
    pulse = -1;
    oxygen = -1;
    main_window_update();
  }
}

static void strap_notify_handler(SmartstrapAttribute *attribute) {
  // Received data message from smartstrap
  APP_LOG(APP_LOG_LEVEL_INFO, "Got notification from smartstrap");
  
  //read data
  smartstrap_attribute_read(attribute);
}

static void strap_read_handler(SmartstrapAttribute *attribute, SmartstrapResult result, const uint8_t *data, size_t length) {
  if(result == SmartstrapResultOk) {
    // Data has been read into the data buffer provided
    //APP_LOG(APP_LOG_LEVEL_INFO, "Smartstrap sent: %x", data[0]);
    
    if(data[0] == KEY_OXYGEN && length == 10) {
      //APP_LOG(APP_LOG_LEVEL_INFO, "Got SpO2 data \\o/");
      
      pulse = (data[2] << 24) | (data[3] << 16) | (data[4] << 8) | data[5];
      oxygen = (data[6] << 24) | (data[7] << 16) | (data[8] << 8) | data[9];
      
      if(pulse >= 2000 || pulse <= 100)
        pulse = -1;
      if(oxygen >= 1000 || oxygen <= 500)
        oxygen = -1;
      
      main_window_update();
      
      //APP_LOG(APP_LOG_LEVEL_INFO, "Data: %x %x%x%x%x %x%x%x%x", data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8], data[9]);
      APP_LOG(APP_LOG_LEVEL_INFO, "Pulse: %d, Oxygen: %d", pulse, oxygen);
    } else if(data[0] == KEY_IMPEDANCE && length == 7) {
      int16_t real = (data[1] << 8) | data[2];
      int16_t imaginary = (data[3] << 8) | data[4];
      uint8_t electrode1 = data[5];
      uint8_t electrode2 = data[6];
      
      APP_LOG(APP_LOG_LEVEL_INFO, "Real: %d, Imaginary: %d, Electrodes: %d, %d", real, imaginary, electrode1, electrode2);
    }
  } else {
    // Some error has occured
    APP_LOG(APP_LOG_LEVEL_ERROR, "Error in read handler: %d", (int)result);
  }
}

static void strap_init() {
  s_attribute = smartstrap_attribute_create(s_service_id, s_attribute_id, 64);

  // Subscribe to smartstrap events
  smartstrap_subscribe((SmartstrapHandlers) {
    .availability_did_change = strap_availability_handler,
    .notified = strap_notify_handler,
    .did_read = strap_read_handler
  });
}

static void select_click_handler(ClickRecognizerRef recognizer, void *context) {
  if(s_service_available) {
    size_t buff_size;
    uint8_t *buffer;
    smartstrap_attribute_begin_write(s_attribute, &buffer, &buff_size);
    snprintf((char*)buffer, buff_size, "i");
    smartstrap_attribute_end_write(s_attribute, buff_size, false);
  }
}

static void click_config_provider(void *context) {
  window_single_click_subscribe(BUTTON_ID_SELECT, select_click_handler);
}


// ----- Init -----

void handle_init(void) {
  strap_init();
  main_window_init(&pulse, &oxygen);
  window_set_click_config_provider(get_main_window(), click_config_provider);
}

void handle_deinit(void) {
  main_window_deinit();
}

int main(void) {
  handle_init();
  app_event_loop();
  handle_deinit();
}
