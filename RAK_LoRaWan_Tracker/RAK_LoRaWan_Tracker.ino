/*
   MCM - LoRaWAN Tracker

   @Author: Christopher Mendez | MCMCHRIS
   @Date: 04/14/2023 (mm/dd/yy)
   @Brief:
   This firmware runs on a nRF52840 (RAK4631) and a GNSS ZOE-M8Q u-Blox module to gather the device location
   and publish it using the Helium network and forwards it to Ubidots for visualization.

   Average power consumption: 45 mA
   With a 820mah battery should last for 17 hours.
*/

#include "main.h"
#include <LoRaWan-RAK4630.h>  //http://librarymanager/All#SX126x
#include "Wire.h"
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>  //http://librarymanager/All#SparkFun_u-blox_GNSS

SFE_UBLOX_GNSS g_myGNSS;

long ilat, ilon, speed;
float lat1, lng1, lat2, lng2;
byte SIV;
float metros = 10;
bool sent = 0;

// RAK4630 supply two LED
#ifndef LED_BUILTIN
#define LED_BUILTIN 35
#endif

#ifndef LED_BUILTIN2
#define LED_BUILTIN2 36
#endif

bool doOTAA = true;                                               // OTAA is used by default.
#define SCHED_MAX_EVENT_DATA_SIZE APP_TIMER_SCHED_EVENT_DATA_SIZE /**< Maximum size of scheduler events. */
#define SCHED_QUEUE_SIZE 60                                       /**< Maximum number of events in the scheduler queue. */
#define LORAWAN_DATERATE DR_3                                     /*LoRaMac datarates definition, from DR_0 to DR_5*/
#define LORAWAN_TX_POWER TX_POWER_5                               /*LoRaMac tx power definition, from TX_POWER_0 to TX_POWER_15*/
#define JOINREQ_NBTRIALS 6                                        /**< Number of trials for the join request. */
DeviceClass_t gCurrentClass = CLASS_A;                            /* class definition*/
LoRaMacRegion_t gCurrentRegion = LORAMAC_REGION_AU915;            /* Region defined by your Country standard*/
lmh_confirm gCurrentConfirm = LMH_UNCONFIRMED_MSG;                /* confirm/unconfirm packet definition*/
uint8_t gAppPort = LORAWAN_APP_PORT;                              /* data port*/

/**@brief Structure containing LoRaWan parameters, needed for lmh_init()
*/
static lmh_param_t lora_param_init = { LORAWAN_ADR_OFF, LORAWAN_DATERATE, LORAWAN_PUBLIC_NETWORK, JOINREQ_NBTRIALS, LORAWAN_TX_POWER, LORAWAN_DUTYCYCLE_OFF };

// Foward declaration
static void lorawan_has_joined_handler(void);
static void lorawan_join_failed_handler(void);
static void lorawan_rx_handler(lmh_app_data_t *app_data);
static void lorawan_confirm_class_handler(DeviceClass_t Class);
static void send_lora_frame(void);

/**@brief Structure containing LoRaWan callback functions, needed for lmh_init()
*/
static lmh_callback_t lora_callbacks = { BoardGetBatteryLevel, BoardGetUniqueId, BoardGetRandomSeed,
                                         lorawan_rx_handler, lorawan_has_joined_handler, lorawan_confirm_class_handler, lorawan_join_failed_handler };

//OTAA keys !!!! KEYS ARE MSB !!!!
uint8_t nodeDeviceEUI[8] = { 0xAC, 0x1F, 0x09, 0xFF, 0xFE, 0x05, 0x75, 0x30 };
uint8_t nodeAppEUI[8] = { 0x60, 0x81, 0xF9, 0x21, 0xC4, 0x76, 0x07, 0xE3 };
uint8_t nodeAppKey[16] = { 0x8E, 0x8D, 0xB9, 0x3A, 0x82, 0x22, 0xFE, 0x1D, 0x09, 0xE2, 0x58, 0x3F, 0x0F, 0x08, 0x65, 0x5C };



// Private defination
#define LORAWAN_APP_DATA_BUFF_SIZE 64 /**< buffer size of the data to be transmitted. */

static uint8_t m_lora_app_data_buffer[LORAWAN_APP_DATA_BUFF_SIZE];               //< Lora user application data buffer.
static lmh_app_data_t m_lora_app_data = { m_lora_app_data_buffer, 0, 0, 0, 0 };  //< Lora user application data structure.

static uint32_t count = 0;
static uint32_t count_fail = 0;

bool control = 0;
bool sameplace = 0;

long g_lastTime = 0;  //Simple local timer. Limits amount if I2C traffic to u-blox module.

/** Semaphore used by events to wake up loop task */
SemaphoreHandle_t taskEvent = NULL;

/** Timer to wakeup task frequently and send message */
SoftwareTimer taskWakeupTimer;

/**
 * @brief Flag for the event type
 * -1 => no event
 * 0 => LoRaWan data received
 * 1 => Timer wakeup
 * 2 => tbd
 * ...
 */
uint8_t eventType = -1;

/**
 * @brief Timer event that wakes up the loop task frequently
 * 
 * @param unused 
 */
void periodicWakeup(TimerHandle_t unused) {

  eventType = 1;
  // Give the semaphore, so the loop task will wake up
  xSemaphoreGiveFromISR(taskEvent, pdFALSE);
}


/**
 * @brief Arduino setup function. Called once after power-up or reset
 * 
 */
void setup(void) {

  pinMode(WB_IO2, OUTPUT);
  digitalWrite(WB_IO2, 0);
  delay(1000);
  digitalWrite(WB_IO2, 1);
  delay(1000);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  pinMode(LED_BUILTIN2, OUTPUT);
  digitalWrite(LED_BUILTIN2, LOW);

  Wire.begin();  // Initialize I2C communication.

  // Create the LoRaWan event semaphore
  taskEvent = xSemaphoreCreateBinary();
  // Initialize semaphore
  xSemaphoreGive(taskEvent);

#ifndef MAX_SAVE
  // Initialize Serial for debug output
  Serial.begin(115200);
#endif

#ifndef MAX_SAVE
  Serial.println("=====================================");
  Serial.println("Welcome to RAK4630 LoRaWan!!!");
  if (doOTAA) {
    Serial.println("Type: OTAA");
  } else {
    Serial.println("Type: ABP");
  }

  switch (gCurrentRegion) {
    case LORAMAC_REGION_AS923:
      Serial.println("Region: AS923");
      break;
    case LORAMAC_REGION_AU915:
      Serial.println("Region: AU915");
      break;
    case LORAMAC_REGION_CN470:
      Serial.println("Region: CN470");
      break;
    case LORAMAC_REGION_EU433:
      Serial.println("Region: EU433");
      break;
    case LORAMAC_REGION_IN865:
      Serial.println("Region: IN865");
      break;
    case LORAMAC_REGION_EU868:
      Serial.println("Region: EU868");
      break;
    case LORAMAC_REGION_KR920:
      Serial.println("Region: KR920");
      break;
    case LORAMAC_REGION_US915:
      Serial.println("Region: US915");
      break;
  }
  Serial.println("=====================================");
#endif

  // Initialize LoRa chip.
  lora_rak4630_init();

  //gps init
  //Serial.println("GPS ZOE-M8Q Example(I2C)");
  if (g_myGNSS.begin() == false)  //Connect to the u-blox module using Wire port
  {
    //Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    while (1) {
      digitalWrite(LED_BUILTIN2, HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN2, LOW);
      delay(100);
    }
  }

  g_myGNSS.setI2COutput(COM_TYPE_UBX);                  //Set the I2C port to output UBX only (turn off NMEA noise)
  g_myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT);  //Save (only) the communications port settings to flash and BBR

  //creat a user timer to send data to server period
  uint32_t err_code;


  // Setup the EUIs and Keys
  if (doOTAA) {
    lmh_setDevEui(nodeDeviceEUI);
    lmh_setAppEui(nodeAppEUI);
    lmh_setAppKey(nodeAppKey);
  }


  // Initialize LoRaWan
  err_code = lmh_init(&lora_callbacks, lora_param_init, doOTAA, gCurrentClass, gCurrentRegion);
  if (err_code != 0) {
#ifndef MAX_SAVE
    Serial.printf("lmh_init failed - %d\n", err_code);
#endif
    return;
  }

  // Start Join procedure
  lmh_join();


  // Take the semaphore so the loop will go to sleep until an event happens
  xSemaphoreTake(taskEvent, 10);
  taskWakeupTimer.begin(SLEEP_TIME, periodicWakeup);
  taskWakeupTimer.start();
}

/**
 * @brief Arduino loop task. Called in a loop from the FreeRTOS task handler
 * 
 */
void loop(void) {

  // Sleep until we are woken up by an event
  if (xSemaphoreTake(taskEvent, portMAX_DELAY) == pdTRUE) {

    // Check the wake up reason
    switch (eventType) {

      case 1:  // Wakeup reason is timer

        ilat = g_myGNSS.getLatitude();
        ilon = g_myGNSS.getLongitude() * -1;
        speed = g_myGNSS.getGroundSpeed();
        SIV = g_myGNSS.getSIV();

        if (control == 0) {
          lat1 = ilat;
          lng1 = ilon;
          control = 1;
        } else if (control == 1) {
          lat2 = ilat;
          lng2 = ilon;
          control = 0;
        }
        if (control == 0 && sent == 1) {
          metros = distance_between(lat1 / 10000000, lng1 / 10000000, lat2 / 10000000, lng2 / 10000000);
          if (lat1 == lat2 && lng1 == lng2) {
            sameplace = 1;
          } else {
            sameplace = 0;
          }

#ifndef MAX_SAVE
          Serial.print("Distancia = ");
          Serial.print(metros);
          Serial.println(" m");
#endif
        }

#ifndef MAX_SAVE
        Serial.print("Latitud: ");
        Serial.print(lat1 / 10000000, 7);
        Serial.print(" Longitud: -");
        Serial.print(lng1 / 10000000, 7);
        Serial.print(" SIV: ");
        Serial.println(SIV);
#endif

        if (ilat > 0 && ilon > 0 && metros < 100 && sameplace == 0 && SIV >= 5 && metros >= 10) {  //
#ifndef MAX_SAVE
          Serial.println("Sending frame now...");
#endif

          sent = 1;
          send_lora_frame();
        } else {
          //Serial.println("Not sending frame");
        }

        break;
      default:
        break;
    }

    // Go back to sleep
    xSemaphoreTake(taskEvent, 10);
  }
}

/**@brief LoRa function for handling HasJoined event.
*/
void lorawan_has_joined_handler(void) {
  //Serial.println("OTAA Mode, Network Joined!");

  lmh_error_status ret = lmh_class_request(gCurrentClass);
  if (ret == LMH_SUCCESS) {
    for (int i = 0; i < 10; i++) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(50);
      digitalWrite(LED_BUILTIN, LOW);
      delay(50);
    }
  }
}
/**@brief LoRa function for handling OTAA join failed
*/
static void lorawan_join_failed_handler(void) {
  for (int i = 0; i < 10; i++) {
    digitalWrite(LED_BUILTIN2, HIGH);
    delay(50);
    digitalWrite(LED_BUILTIN2, LOW);
    delay(50);
  }
}
/**@brief Function for handling LoRaWan received data from Gateway

   @param[in] app_data  Pointer to rx data
*/
void lorawan_rx_handler(lmh_app_data_t *app_data) {
  //Serial.printf("LoRa Packet received on port %d, size:%d, rssi:%d, snr:%d, data:%s\n",
  //app_data->port, app_data->buffsize, app_data->rssi, app_data->snr, app_data->buffer);
}

void lorawan_confirm_class_handler(DeviceClass_t Class) {
  //Serial.printf("switch to class %c done\n", "ABC"[Class]);
  // Informs the server that switch has occurred ASAP
  m_lora_app_data.buffsize = 0;
  m_lora_app_data.port = gAppPort;
  lmh_send(&m_lora_app_data, gCurrentConfirm);
}

void send_lora_frame(void) {

  if (lmh_join_status_get() != LMH_SET) {
    //Not joined, try again later
    return;
  }

  // && metros > 10) || (ilat > 0 && ilon > 0 && metros < 100 && control == 1) || ilat > 0 && ilon > 0
  memset(m_lora_app_data.buffer, 0, LORAWAN_APP_DATA_BUFF_SIZE);
  m_lora_app_data.port = gAppPort;
  m_lora_app_data.buffer[0] = 0x09;
  //lat data
  m_lora_app_data.buffer[1] = (ilat & 0xFF000000) >> 24;
  m_lora_app_data.buffer[2] = (ilat & 0x00FF0000) >> 16;
  m_lora_app_data.buffer[3] = (ilat & 0x0000FF00) >> 8;
  m_lora_app_data.buffer[4] = ilat & 0x000000FF;

  //lon data
  m_lora_app_data.buffer[5] = (ilon & 0xFF000000) >> 24;
  m_lora_app_data.buffer[6] = (ilon & 0x00FF0000) >> 16;
  m_lora_app_data.buffer[7] = (ilon & 0x0000FF00) >> 8;
  m_lora_app_data.buffer[8] = ilon & 0x000000FF;

  //speed data
  m_lora_app_data.buffer[9] = (speed & 0xFF000000) >> 24;
  m_lora_app_data.buffer[10] = (speed & 0x00FF0000) >> 16;
  m_lora_app_data.buffer[11] = (speed & 0x0000FF00) >> 8;
  m_lora_app_data.buffer[12] = speed & 0x000000FF;

  m_lora_app_data.buffsize = 13;

  lmh_error_status error = lmh_send(&m_lora_app_data, gCurrentConfirm);

  if (error == LMH_SUCCESS) {
    count++;
    //Serial.printf("lmh_send ok count %d\n", count);
    digitalWrite(LED_BUILTIN2, HIGH);
    delay(1000);
    digitalWrite(LED_BUILTIN2, LOW);

  } else {

    count_fail++;
    //Serial.printf("lmh_send fail count %d\n", count_fail);
  }
}

float distance_between(float lat1, float long1, float lat2, float long2) {
  // returns distance in meters between two positions, both specified
  // as signed decimal-degrees latitude and longitude. Uses great-circle
  // distance computation for hypothetical sphere of radius 6372795 meters.
  // Because Earth is no exact sphere, rounding errors may be up to 0.5%.
  // Courtesy of Maarten Lamers
  float delta = radians(long1 - long2);
  float sdlong = sin(delta);
  float cdlong = cos(delta);
  lat1 = radians(lat1);
  lat2 = radians(lat2);
  float slat1 = sin(lat1);
  float clat1 = cos(lat1);
  float slat2 = sin(lat2);
  float clat2 = cos(lat2);
  delta = (clat1 * slat2) - (slat1 * clat2 * cdlong);
  delta = sq(delta);
  delta += sq(clat2 * sdlong);
  delta = sqrt(delta);
  float denom = (slat1 * slat2) + (clat1 * clat2 * cdlong);
  delta = atan2(delta, denom);
  return delta * 6372795;
}
