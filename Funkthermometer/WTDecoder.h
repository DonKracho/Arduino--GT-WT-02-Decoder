#include <Arduino.h>

#define WTD_DEBUG
//#define WTD_DEBUG_VERBOSE
//#define VALID_TX_ID_ONLY

// WeMos D1 mini
#define INT_PIN D2  // GPIO0
#ifdef WTD_DEBUG
#define LED_PIN D4  // GPIO2
#endif

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
    int16_t temprature; // terperature in 1/10 °C
    int16_t humidity;   // humidity in %
    unsigned long timestamp;
  };

  WTDecoder() {};
  ~WTDecoder() {};

  void Loop();
  void Setup();
  bool GetRecord(struct rec &record);

private:
  bool decodeRecord(uint64_t value, struct rec &record);
  void printRecord(struct rec &record);
};

extern WTDecoder wtd;
