#include <Arduino.h>

//#define WTD_DEBUG
//#define WTD_DEBUG_VERBOSE
//#define VALID_TX_ID_ONLY

// WeMos D1 mini
#define INT_PIN D2  // GPIO0
#define LED_PIN D4  // GPIO2

// Arducam ESP8266
//#define INT_PIN D6  // GPIO0
//#define LED_PIN LED_BUILTIN  // GPIO2

#define MAX_CODES 20
#define MAX_RECORDS 20

class WTDecoder {
public:
  struct rec
  {
    bool valid;         // record is valid
    byte txid;          // transmitter ID random when battery gets inserted
    bool battery;       // 1 if battery is low
    bool button;        // true if was triggerd by tx transmit button
    byte channel;       // channel switch of transmitter 1,2,3
    int16_t temperature;// terperature in 1/10 °C
    int16_t humidity;   // humidity in %
    uint32_t timestamp; // time this record was created
  };

  WTDecoder() {};
  ~WTDecoder() {};

  void Loop();
  void Setup();
  bool GetRecord(struct rec &record);
  char *Record2String(struct rec &record);

private:
  bool decodeRecord(uint64_t value, struct rec &record);
  void storeRecord(struct rec &record);
  int  mLastReadRec = -1;
  int  mLastStoredRec = -1;
  struct rec mRecords[MAX_RECORDS];
  char mBuffer[64];
};

extern WTDecoder wtd;
