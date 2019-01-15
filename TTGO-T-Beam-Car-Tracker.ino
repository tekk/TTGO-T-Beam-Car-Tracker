#include <SPI.h>
#include <lmic.h>
#include <hal/hal.h>
#include <WiFi.h>
#include <CayenneLPP.h>
#include <Wire.h>
#include <Adafruit_BME280.h>

// UPDATE the config.h file in the same folder WITH YOUR TTN KEYS AND ADDR.
// LMIC library will complain that it couldn't initialize the module without these keys
#include "config.h"
#include "gps.h"

#define I2C_SDA 21 // SDA1
#define I2C_SCL 22 // SCL1
#define SEALEVELPRESSURE_HPA (1013.25) // this should be set according to the weather forecast
#define BME280_ADDRESS 0x76 // you can use I2C scanner demo to find your BME280 I2C address
#define BUILTIN_LED 14 // T-Beam blue LED, see: http://tinymicros.com/wiki/TTGO_T-Beam

CayenneLPP lpp(51); // here we will construct Cayenne Low Power Payload (LPP) - see https://community.mydevices.com/t/cayenne-lpp-2-0/7510
gps gps; // class that is encapsulating additional GPS functionality
Adafruit_BME280 bme(I2C_SDA, I2C_SCL); // these pins are defined above

double lat, lon, alt, kmph; // GPS data are saved here: Latitude, Longitude, Altitude, Speed in km/h
float tmp, hum, pressure, alt_barometric; // BME280 data are saved here: Temperature, Humidity, Pressure, Altitude calculated from atmospheric pressure
int sats; // GPS satellite count
char s[32]; // used to sprintf for Serial output
bool status; // status after reading from BME280

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static osjob_t sendjob; // callback to LoRa send packet 
void getBME280Values(void); // declaration for function below

// Schedule TX every this many seconds (might become longer due to duty cycle limitations).
const unsigned int TX_INTERVAL = 240;
const unsigned int GPS_FIX_RETRY_DELAY = 10; // wait this many seconds when no GPS fix is received to retry
const unsigned int SHORT_TX_INTERVAL = 20; // when driving, send packets every SHORT_TX_INTERVAL seconds
const double MOVING_KMPH = 10.0; // if speed in km/h is higher than MOVING_HMPH, we assume that car is moving

// Pin mapping
const lmic_pinmap lmic_pins = {
  .nss = 18,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 23,
  .dio = {26, 33, 32},  // PIN 33 HAS TO BE PHYSICALLY CONNECTED TO PIN Lora1 OF TTGO
};                      // the second connection from Lora2 to pin 32 is not necessary


void getBME280Values() {

  if (!status) { // we don't have BME280 connection, clear the values and exit
    tmp = 0.0f;
    pressure = 0.0f;
    alt_barometric = 0.0f;
    hum = 0.0f;
    return;
  }
  
  tmp = bme.readTemperature();
  pressure = bme.readPressure() / 100.0F;
  alt_barometric = bme.readAltitude(SEALEVELPRESSURE_HPA);
  hum = bme.readHumidity();
  
  Serial.print(F("Temperature = "));
  Serial.print(tmp);
  Serial.print("C, ");
  Serial.print("Pressure = ");
  Serial.print(pressure);
  Serial.print("hPa, ");
  Serial.print("Approx. Altitude = ");
  Serial.print(alt_barometric);
  Serial.print("m, ");
  Serial.print("Humidity = ");
  Serial.print(hum);
  Serial.println("%");

  delay(100);
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
      Serial.println(F("EV_JOINING"));
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
        Serial.println(F("Received ACK!"));
      }
      if (LMIC.dataLen) {
        sprintf(s, "Yey! Received %i bytes of payload!", LMIC.dataLen);
        Serial.println(s);
        sprintf(s, "RSSI %d SNR %.1d", LMIC.rssi, LMIC.snr);
        Serial.println(s);
      }
      // Schedule next transmission
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(kmph > MOVING_KMPH ? SHORT_TX_INTERVAL : TX_INTERVAL), do_send);
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

  getBME280Values();
  
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND)
  {
    Serial.println(F("OP_TXRXPEND, not sending"));
  }
  else
  { 
    if (gps.checkGpsFix())
    {
      // Prepare upstream data transmission at the next possible time.
      gps.getLatLon(&lat, &lon, &alt, &kmph, &sats);

      // we have all the data that we need, let's construct LPP packet for Cayenne
      lpp.reset();
      lpp.addGPS(1, lat, lon, alt);
      lpp.addTemperature(2, tmp);
      lpp.addRelativeHumidity(3, hum);
      lpp.addBarometricPressure(4, pressure);
      lpp.addAnalogInput(5, kmph);
      // optional: Satellite count and altitude from barometric sensor
      //lpp.addAnalogInput(6, sats);
      //lpp.addAnalogInput(7, alt_barometric);

      // read LPP packet bytes, write them to FIFO buffer of the LoRa module, queue packet to send to TTN
      LMIC_setTxData2(1, lpp.getBuffer(), lpp.getSize(), 0);
      
      Serial.print(lpp.getSize());
      Serial.println(F(" bytes long LPP packet queued."));
      digitalWrite(BUILTIN_LED, HIGH);
    }
    else
    {
      // try again in a few 'GPS_FIX_RETRY_DELAY' seconds...
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(GPS_FIX_RETRY_DELAY), do_send);
    }
  }
  // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
  Serial.begin(115200);
  Serial.println(F("LoRa & GSM based TTN car tracker"));
  
  //Turn off WiFi and Bluetooth
  WiFi.mode(WIFI_OFF);
  btStop();
  
  gps.init();

  status = bme.begin(BME280_ADDRESS);
  
  if (!status) {
    Serial.println(F("Could not find a valid BME280 sensor, check wiring!"));
  } else {
    Serial.println(F("BME280 initialized sucessfully"));
    delay(1000); // stabilize sensor readings
  }

  if (status) {
    Serial.println(F("normal mode, 16x pressure / 2x temperature / 1x humidity oversampling,"));
    Serial.println(F("0.5ms standby period, filter 16x"));
    bme.setSampling(Adafruit_BME280::MODE_NORMAL,
                    Adafruit_BME280::SAMPLING_X2,  // temperature
                    Adafruit_BME280::SAMPLING_X16, // pressure
                    Adafruit_BME280::SAMPLING_X1,  // humidity
                    Adafruit_BME280::FILTER_X16,
                    Adafruit_BME280::STANDBY_MS_0_5 );


    delay(500);
  }

  Serial.println(F("Initializing LoRa module"));
  
  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  // Set static session parameters. Instead of dynamically establishing a session
  // by joining the network, precomputed session parameters are be provided.
  LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);

  #if defined(CFG_eu868)
  // Set up the channels used by the Things Network, which corresponds
  // to the defaults of most gateways. Without this, only three base
  // channels from the LoRaWAN specification are used, which certainly
  // works, so it is good for debugging, but can overload those
  // frequencies, so be sure to configure the full frequency range of
  // your network here (unless your network autoconfigures them).
  // Setting up channels should happen after LMIC_setSession, as that
  // configures the minimal channel set.
  // NA-US channels 0-71 are configured automatically
  LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
  LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
  // TTN defines an additional channel at 869.525Mhz using SF9 for class B
  // devices' ping slots. LMIC does not have an easy way to define set this
  // frequency and support for class B is spotty and untested, so this
  // frequency is not configured here.
  #elif defined(CFG_us915)
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
  digitalWrite(BUILTIN_LED, LOW);

  Serial.println(F("Ready to track"));
  
  // Start job
  do_send(&sendjob);
}

void loop() {
    os_runloop_once();
}
