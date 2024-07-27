/*******************************************************************************
Carte Heltec wifi lora 32
send pulse led duration for consumption
*******************************************************************************/

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <CayenneLPP.h>
#include <WiFi.h>

#define _TEST_WITHOUT_LORA

#define ARDUINO_HELTEC_WIFI_LORA_32

#define WAIT_1S 1000

#define PIN_RESET_RADIO 23

#define PIN_BAT 35
#define PIN1_DIODE 17
#define PIN2_DIODE 16


int batvalue = 0;
int batVolt = 0;
int photodiode = 0;
long int pulseduration = 0;
long int pulsedurationfinal = 0;


#define uS_TO_S_FACTOR 1000000ULL /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP 600         /* Time ESP32 will go to sleep (in seconds) */

RTC_DATA_ATTR int bootCount = 0;

/*
Method to print the reason by which ESP32
has been awaken from sleep
*/
void print_wakeup_reason() {
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch (wakeup_reason) {
    case ESP_SLEEP_WAKEUP_EXT0: Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1: Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER: Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD: Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP: Serial.println("Wakeup caused by ULP program"); break;
    default: Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason); break;
  }
}

unsigned long now = 0;

unsigned long now_wait1 = 0;

unsigned long now_wait2 = 0;
unsigned long count1sec = 0;

int increment_sleep = 0;

int compteur = 0;
int onetime = 1;


DynamicJsonDocument jsonBuffer(1024);
CayenneLPP lpp(160);

JsonObject root = jsonBuffer.to<JsonObject>();

//
// For normal use, we require that you edit the sketch to replace FILLMEIN
// with values assigned by the TTN console. However, for regression tests,
// we want to be able to compile these scripts. The regression tests define
// COMPILE_REGRESSION_TEST, and in that case we define FILLMEIN to a non-
// working but innocuous value.
//
#ifdef COMPILE_REGRESSION_TEST
#define FILLMEIN 0
#else
#warning "You must replace the values marked FILLMEIN with real values from the TTN control panel!"
#define FILLMEIN (#dont edit this, edit the lines that use FILLMEIN)
#endif

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t PROGMEM APPEUI[8] = { FILLMEIN };
void os_getArtEui(u1_t* buf) {
  memcpy_P(buf, APPEUI, 8);
}

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8] = { FILLMEIN };
void os_getDevEui(u1_t* buf) {
  memcpy_P(buf, DEVEUI, 8);
}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
static const u1_t PROGMEM APPKEY[16] = { FILLMEIN };
void os_getDevKey(u1_t* buf) {
  memcpy_P(buf, APPKEY, 16);
}

static uint8_t mydata[] = "Hello, world!";
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = TIME_TO_SLEEP;

// Pin mapping
const lmic_pinmap lmic_pins = {
  .nss = 18,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 14,
  .dio = { 26, 33, 32 },  // { 26, 34, 35 } {LORA_PIN_SPI_DIO0, LORA_PIN_SPI_DIO1, LMIC_UNUSED_PIN},
};

void printHex2(unsigned v) {
  v &= 0xff;
  if (v < 16)
    Serial.print('0');
  Serial.print(v, HEX);
}

void onEvent(ev_t ev) {
  Serial.print(os_getTime());
  Serial.print(": ");
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
      {
        u4_t netid = 0;
        devaddr_t devaddr = 0;
        u1_t nwkKey[16];
        u1_t artKey[16];
        LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
        Serial.print("netid: ");
        Serial.println(netid, DEC);
        Serial.print("devaddr: ");
        Serial.println(devaddr, HEX);
        Serial.print("AppSKey: ");
        for (size_t i = 0; i < sizeof(artKey); ++i) {
          if (i != 0)
            Serial.print("-");
          printHex2(artKey[i]);
        }
        Serial.println("");
        Serial.print("NwkSKey: ");
        for (size_t i = 0; i < sizeof(nwkKey); ++i) {
          if (i != 0)
            Serial.print("-");
          printHex2(nwkKey[i]);
        }
        Serial.println();
      }
      // Disable link check validation (automatically enabled
      // during join, but because slow data rates change max TX
      // size, we don't use it in this example.
      LMIC_setLinkCheckMode(0);
      break;
    /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_RFU1:
        ||     Serial.println(F("EV_RFU1"));
        ||     break;
        */
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      break;
    case EV_TXCOMPLETE:
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      if (LMIC.txrxFlags & TXRX_ACK)
        Serial.println(F("Received ack"));
      if (LMIC.dataLen) {
        Serial.print(F("Received "));
        Serial.print(LMIC.dataLen);
        Serial.println(F(" bytes of payload"));
      }
      // Schedule next transmission
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
      //radio off
      digitalWrite(PIN_RESET_RADIO, LOW);

      Serial.println("Going to sleep now");
      Serial.flush();
      esp_deep_sleep_start();

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
    /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_SCAN_FOUND:
        ||    Serial.println(F("EV_SCAN_FOUND"));
        ||    break;
        */
    case EV_TXSTART:
      Serial.println(F("EV_TXSTART"));
      break;
    case EV_TXCANCELED:
      Serial.println(F("EV_TXCANCELED"));
      break;
    case EV_RXSTART:
      /* do not print anything -- it wrecks timing */
      break;
    case EV_JOIN_TXCOMPLETE:
      Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
      break;

    default:
      Serial.print(F("Unknown event: "));
      Serial.println((unsigned)ev);
      break;
  }
}

void do_send(osjob_t* j) {
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
    // Prepare upstream data transmission at the next possible time.
    LMIC.bands[BAND_MILLI].avail = os_getTime();
    LMIC.bands[BAND_CENTI].avail = os_getTime();
    LMIC.bands[BAND_DECI].avail = os_getTime();

    lpp.decodeTTN(lpp.getBuffer(), lpp.getSize(), root);
    serializeJsonPretty(root, Serial);
    Serial.println();

    LMIC_setTxData2(1, lpp.getBuffer(), lpp.getSize(), 0);
    //LMIC_setTxData2(1, mydata, sizeof(mydata) - 1, 0);
    Serial.println(F("Packet queued"));
  }
  // Next TX is scheduled after TX_COMPLETE event.
}








void setup() {

  Serial.begin(115200);
  Serial.println(F("Starting"));

  // Désactiver le WiFi
  WiFi.mode(WIFI_OFF);
  delay(1);

  lpp.reset();

  pinMode(PIN_RESET_RADIO, OUTPUT);
  //radio on
  digitalWrite(PIN_RESET_RADIO, LOW);
  delay(10);
  //radio on
  digitalWrite(PIN_RESET_RADIO, HIGH);
  delay(10);

  pinMode(PIN1_DIODE, OUTPUT);
  digitalWrite(PIN1_DIODE, LOW);
  pinMode(PIN2_DIODE, INPUT_PULLUP);

  photodiode = digitalRead(PIN2_DIODE);

  analogReadResolution(12);
  batvalue = analogRead(PIN_BAT);
  batvalue = analogRead(PIN_BAT);
  batvalue = analogRead(PIN_BAT);
  batvalue = analogRead(PIN_BAT);
  batvalue = analogRead(PIN_BAT);
  batvalue = analogRead(PIN_BAT);

  //Increment boot number and print it every reboot
  ++bootCount;
  Serial.println("Boot number: " + String(bootCount));
  //Print the wakeup reason for ESP32
  print_wakeup_reason();

  /*
  First we configure the wake up source
  We set our ESP32 to wake up every 5 seconds
  */
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) + " Seconds");

#ifdef VCC_ENABLE
  // For Pinoccio Scout boards
  //pinMode(VCC_ENABLE, OUTPUT);
  //digitalWrite(VCC_ENABLE, HIGH);
  //delay(1000);
#endif

  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  LMIC_setLinkCheckMode(0);
  LMIC_setClockError(MAX_CLOCK_ERROR * 10 / 100);

  // Start job (sending automatically starts OTAA too)
  //do_send(&sendjob);

#ifdef _TEST_WITHOUT_LORA
  while (true) {
  photodiode = digitalRead(PIN2_DIODE);

  batvalue = analogRead(PIN_BAT);
  batvalue = analogRead(PIN_BAT);
  batvalue = analogRead(PIN_BAT);
  batvalue = analogRead(PIN_BAT);
  batvalue = analogRead(PIN_BAT);
  batvalue = analogRead(PIN_BAT);

  batVolt = analogReadMilliVolts(PIN_BAT);
  batVolt = analogReadMilliVolts(PIN_BAT);
  batVolt = analogReadMilliVolts(PIN_BAT);
  batVolt = analogReadMilliVolts(PIN_BAT);
  batVolt = analogReadMilliVolts(PIN_BAT);
  batVolt = analogReadMilliVolts(PIN_BAT);

  if (true) {
    now = millis();

    if (digitalRead(PIN2_DIODE) == 0 && compteur == 0) {
      compteur = 1;
      pulseduration = millis();
      pulsedurationfinal = millis();
    }

    if (millis() > (pulseduration + 10)) {
      if (digitalRead(PIN2_DIODE) == 1 && compteur == 1) {
        compteur = 2;
        pulseduration = millis();
      }
    }

    if (millis() > (pulseduration + 10)) {
      if (digitalRead(PIN2_DIODE) == 0 && compteur == 2) {
        compteur = 0;
        pulsedurationfinal = millis() - pulsedurationfinal;
        Serial.print(photodiode);
        Serial.print(",");
        Serial.print(pulsedurationfinal);
        Serial.print(",");
        Serial.print(batvalue);
        Serial.print(",");
        Serial.println(batVolt);
        delay(1000);
      }
    }
  }
  }
#endif

  lpp.addAnalogInput(1, batvalue);

  pulseduration = 0;

  now = millis();
  now_wait1 = now + 10 * WAIT_1S;
  now_wait2 = now + 40 * WAIT_1S;
  count1sec=0;

  compteur = 0;
  onetime = 1;
}


void loop() {
  now = millis();

  if (now> count1sec) {
    count1sec=millis()+WAIT_1S;
    Serial.print("*");
  }

  if (digitalRead(PIN2_DIODE) == 0 && compteur == 0) {
    compteur = 1;
    pulseduration = millis();
    pulsedurationfinal = millis();
  }

  if (millis() > (pulseduration + 10)) {
    if (digitalRead(PIN2_DIODE) == 1 && compteur == 1) {
      compteur = 2;
      pulseduration = millis();
    }
  }

  if (millis() > (pulseduration + 10)) {
    if (digitalRead(PIN2_DIODE) == 0 && compteur == 2) {
      compteur = 3;
      now_wait1 = 0;
      pulsedurationfinal = millis() - pulsedurationfinal;
      lpp.addLuminosity(2, pulsedurationfinal);
      Serial.println(pulsedurationfinal);
    }
  }

  if (now > now_wait1) {
    // Start job (sending automatically starts OTAA too)
    if (compteur != 3) lpp.addLuminosity(2, 0);
    if (onetime == 1) {
      onetime = 0;
      now_wait2 = millis() + 15 * WAIT_1S;
      do_send(&sendjob);
      delay(10);
    }
  }

  if (onetime == 0) {
    os_runloop_once();
    delay(10);
  }


  if (now > now_wait2) {
    //radio off
    LMIC_shutdown();
    delay(10);
    digitalWrite(PIN_RESET_RADIO, LOW);

    Serial.println("Going to sleep now");
    Serial.flush();
    esp_deep_sleep_start();
    // Schedule next transmission
    //os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
    Serial.println("This will never be printed");
  }
}
