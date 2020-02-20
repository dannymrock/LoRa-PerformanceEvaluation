#include <Arduino.h>
#include <SPI.h>
#include <lmic.h>
#include <hal/hal.h>
#include <WiFi.h>
#include <CayenneLPP.h>
#include <Wire.h>
//#include <Adafruit_BME280.h>


// UPDATE the config.h file in the same folder WITH YOUR TTN KEYS AND ADDR.
// LMIC library will complain that it couldn't initialize the module without these keys
#include "config.h"
#include "gps.h"

#define DEBUG 1 // for real use comment this out
#define I2C_SDA 21 // SDA1
#define I2C_SCL 22 // SCL1


#define BUILTIN_LED 14 // T-Beam blue LED, see: http://tinymicros.com/wiki/TTGO_T-Beam
#define BATTERY_PIN 35 // battery level measurement pin, here is the voltage divider connected
#define Button 39

// UPDATE WITH YOUR TTN KEYS AND ADDR.
static PROGMEM u1_t NWKSKEY[16] = { 0x74, 0x9D, 0x7A, 0x2B, 0xBC, 0x0B, 0x8E, 0x0C, 0xA9, 0x92, 0xAF, 0x62, 0x6B, 0x25, 0x7A, 0x60 }; // LoRaWAN NwkSKey, network session key 
static u1_t PROGMEM APPSKEY[16] = { 0x32, 0x84, 0xC9, 0x3F, 0xC6, 0xA1, 0xB7, 0x91, 0x6B, 0xD2, 0xE8, 0x88, 0x14, 0xE9, 0xD4, 0x15 }; // LoRaWAN AppSKey, application session key 
static const u4_t DEVADDR = 0x26011402 ; // LoRaWAN end-device address (DevAddr)


CayenneLPP lpp(51); // here we will construct Cayenne Low Power Payload (LPP) - see https://community.mydevices.com/t/cayenne-lpp-2-0/7510
gps gps1; // class that is encapsulating additional GPS functionality

double lat, lon, alt, kmph; // GPS data are saved here: Latitude, Longitude, Altitude, Speed in km/h
float tmp=0.0;
int sats; // GPS satellite count
char s[32]; // used to sprintf for Serial output
float vBat; // battery voltage
long nextPacketTime;
volatile int pressed = 0;

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }


void IRAM_ATTR isr() {
  pressed = 1;
  Serial.print("ISR button"); 
}

static osjob_t sendjob; // callback to LoRa send packet 
void do_send(osjob_t* j); // declaration of send function


// Schedule TX every this many seconds (might become longer due to duty cycle limitations).
#ifdef DEBUG
const unsigned int TX_INTERVAL = 30;
#elif
const unsigned int TX_INTERVAL = 120;
#endif

const unsigned int GPS_FIX_RETRY_DELAY = 10; // wait this many seconds when no GPS fix is received to retry
const unsigned int SHORT_TX_INTERVAL = 20; // when driving, send packets every SHORT_TX_INTERVAL seconds
const double MOVING_KMPH = 10.0; // if speed in km/h is higher than MOVING_HMPH, we assume that car is moving

// Pin mapping T-Beam
const lmic_pinmap lmic_pins = {
  .nss = 18,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 23,
  .dio = {26, 33, 32},  // PIN 33 HAS TO BE PHYSICALLY CONNECTED TO PIN Lora1 OF TTGO
};                      // the second connection from Lora2 to pin 32 is not necessary


void getBatteryVoltage() {
  // we've set 10-bit ADC resolution 2^10=1024 and voltage divider makes it half of maximum readable value (which is 3.3V)
  vBat = analogRead(BATTERY_PIN) * 2.0 * (3.3 / 1024.0);
  Serial.print("Battery voltage: ");
  Serial.print(vBat);
  Serial.println("V");  
}


void onEvent (ev_t ev) {
  switch (ev) {
    case EV_SCAN_TIMEOUT:
      Serial.println(F("EV_SCAN_TIMEOUT"));
      break;
    case EV_BEACON_FOUND:
      Serial.println(F("EV_BEACON_FOUND"));
      break;
    case EV_BEACON_MISSED:
      Serial.println(F("EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
      Serial.println(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:
      Serial.println(F("EV_JOINING. If program stops here, set correct LoRaWAN keys in config.h!"));
      break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED"));
      // Disable link check validation (automatically enabled
      // during join, but not supported by TTN at this time).
      LMIC_setLinkCheckMode(0);
      break;
    case EV_RFU1:
      Serial.println(F("EV_RFU1"));
      break;
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      break;
    case EV_TXCOMPLETE:
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      digitalWrite(BUILTIN_LED, LOW);
      if (LMIC.txrxFlags & TXRX_ACK) {
        Serial.println(F("Yup, received ACK!"));
      }
      if (LMIC.dataLen) {
        sprintf(s, "Yeey! Received %i bytes of payload!", LMIC.dataLen);
        Serial.println(s);
        sprintf(s, "RSSI %d SNR %.1d", LMIC.rssi, LMIC.snr);
        Serial.println(s);
      }
      // Schedule next transmission
      nextPacketTime = (kmph > MOVING_KMPH ? SHORT_TX_INTERVAL : TX_INTERVAL); // depend on current GPS speed
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(nextPacketTime), do_send);
      Serial.print(F("Next LoRa packet scheduled in "));
      Serial.print(nextPacketTime);
      Serial.println(F(" seconds!"));
      Serial.println(F("------------------------------------------------"));
      break;
    case EV_LOST_TSYNC:
      Serial.println(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      Serial.println(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      Serial.println(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      Serial.println(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      Serial.println(F("EV_LINK_ALIVE"));
      break;
    default:
      Serial.println(F("Unknown event"));
      break;
  }
}

void do_send(osjob_t* j) {  

  getBatteryVoltage();
  Serial.println(F("Pasó por acá: do send"));
      

  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND)
  {
    Serial.println(F("OP_TXRXPEND, not sending"));
  }
  else
  { 
    if (gps1.checkGpsFix())
    {
      Serial.println(F("Pasó por acá: GPS Fix"));
      digitalWrite(BUILTIN_LED,HIGH);
    }
    else
    {
      // try again in a few 'GPS_FIX_RETRY_DELAY' seconds...
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(GPS_FIX_RETRY_DELAY), do_send);
      digitalWrite(BUILTIN_LED,LOW);
    }

    if(pressed == 1){
      pressed = 0;
      Serial.println(F("Pasó por acá: pressed"));
      // Prepare upstream data transmission at the next possible time.
      gps1.getLatLon(&lat, &lon, &alt, &kmph, &sats);

      // we have all the data that we need, let's construct LPP packet for Cayenne
      lpp.reset();
      lpp.addGPS(1, lat, lon, alt);
      lpp.addTemperature(2, tmp);
      lpp.addAnalogInput(5, vBat);
      lpp.addAnalogInput(7, sats);
      
      // read LPP packet bytes, write them to FIFO buffer of the LoRa module, queue packet to send to TTN
      LMIC_setTxData2(1, lpp.getBuffer(), lpp.getSize(), 0);
      
      Serial.print(lpp.getSize());
      Serial.println(F(" bytes long LPP packet queued."));
      digitalWrite(BUILTIN_LED, LOW);
    }
  }
  // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
  Serial.begin(115200);
  Serial.println(F("LoRa & GSM based TTN car tracker"));

  // set battery measurement pin
  adcAttachPin(BATTERY_PIN);
  adcStart(BATTERY_PIN);
  analogReadResolution(10); // Default of 12 is not very linear. Recommended to use 10 or 11 depending on needed resolution.
  
  //Turn off WiFi and Bluetooth
  WiFi.mode(WIFI_OFF);
  btStop();
  
  gps1.init();


  Serial.println(F("Initializing LoRa module"));
  
  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  // Set static session parameters. Instead of dynamically establishing a session
  // by joining the network, precomputed session parameters are be provided.
  LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);

  #if defined(CFG_us915)
  // NA-US channels 0-71 are configured automatically
  // but only one group of 8 should (a subband) should be active
  // TTN recommends the second sub band, 1 in a zero based count.
  // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
  LMIC_selectSubBand(1);
  #endif

  // Disable link check validation
  LMIC_setLinkCheckMode(0);

  // TTN uses SF9 for its RX2 window.
  LMIC.dn2Dr = DR_SF9;

  // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
  LMIC_setDrTxpow(DR_SF7, 14);

  pinMode(BUILTIN_LED, OUTPUT);
  pinMode(Button,INPUT);
  attachInterrupt(Button, isr, FALLING);
  
  digitalWrite(BUILTIN_LED, LOW);

  Serial.println(F("Ready to track"));
  
  // Start job
  pressed = 0;
  do_send(&sendjob);
}

void loop() {
    os_runloop_once();
}
