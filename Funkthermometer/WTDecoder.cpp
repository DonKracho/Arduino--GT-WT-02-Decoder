/*
  Kompatible 433MHz Funk Temperatur/Luftfeuchte Sensoren:
  Funksensor GT-WT-02 (Globaltronics Gmbh & Co. KG, DomStr. 19, 20095 Hamburg)
  Funksensor NT-1959 (Latupo GmbH, Walerloohain 5, 22769 Hamburg PJN-Nr.: EL201523-01) (Pollin)
  und baugleiche Sensoren
  
  für Wetter Stationen wie GT-WS-08 und GT-WS-09 (Aldi)

  Ein Sendeimpuls (Modulation aktiv) ist zwischen 480 und 600us lang gefolgt von einer Pause (Modulation ausgeschaltet)
  Eine Puls Pause Folge entspricht einem Bit oder Sync Impuls
  die Länge der Pause ist hier definiert zu:
  2070us 0
  4140us 1
  9060us Start/End
  Sync zwischen den Ütertragungen der 37 Bit Pakete 9060 20180 9060
     _
  0 | |_ _ _ 2700us             ca. 2/7 * 300us
     _
  1 | |_ _ _ _ _ _  4730 us     ca. 2/14 * 300us
     _                                 _       _
  S | |_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _| |_ _ _| |_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ Beginn der Übertragung
  
  Protokoll: 37 Bit pro Daten Paket
  Zwischen den Datenpaketen ist ein Sync Impuls und wird insgesamt 6 mal wiederholt
  Der Anfang der ersten Übertragung kann stark gestört sein bis die AGC des Empfängers (Pendel-Audion) eingeregelt ist.
  
  Beispiel LOG 6 aufeinander folgender 6 Datenpakete mit Programm RTL_433:
  
  [01] {37} d9 01 07 61 20 : 11011001 00000001 00000111 01100001 00100000 
  [02] {37} d9 01 07 61 20 : 11011001 00000001 00000111 01100001 00100000 
  [03] {37} d9 01 07 61 20 : 11011001 00000001 00000111 01100001 00100000 
  [04] {37} d9 01 07 61 20 : 11011001 00000001 00000111 01100001 00100000 
  [05] {37} d9 01 07 61 20 : 11011001 00000001 00000111 01100001 00100000 
  [16] {37} d9 01 07 61 20 : 11011001 00000001 00000111 01100001 00100000 
  
  Codierung des Signals:
  
  0000000001111111111222222222233333333
  0123456789012345678901234567890123456
  KKKKKKKKBSCCTTTTTTTTTTTTHHHHHHHPPPPPP
  OOOOOOOOAEHHEEEEEEEEEEEEUUUUUUURRRRRR
  DDDDDDDDTNNNMMMMMMMMMMMMMMMMMMMÜÜÜÜÜÜ
  EEEEEEEETDLLPPPPPPPPPPPPIIIIIIIFFFFFF
  
  K = 00-07,  8 Bit = Wechselnder Code bei Batteriewechsel
  B = 08-08,  1 Bit = Batteriestatus 0 = ok, 1 = low
  S = 09-09,  1 Bit = Sendtaste gedrückt = 1, pairing?
  C = 10-11,  2 Bit = Kanal, 00 = K1, 01 = K2, 10 = K3
  T = 12-24, 12 Bit = Temperatur, binär MSB->LSB o. Komma 26,0=260, 2er Komplement
                Bit 12 MSB 0=positiv, 1=negativ (4095+1 - Bit13-23 bin2dez)
  H = 25-30,  7 Bit = Feuchte, binär, Bereich 20-90% LL=10%, HH=110%
  P = 31-36,  6 Bit = Prüfsumme 8 Nibbles aufsummiert
                      bin2Dez Bit 00-03 +
                      bin2Dez Bit 04-07 +
                      bin2Dez Bit 08-11 +
                      bin2Dez Bit 12-15 +
                      bin2Dez Bit 16-21 +
                      bin2Dez Bit 22-23 +
                      bin2Dez Bit 24-27 +
                      bin2Dez Bit 28-30 + Fill Bit 0 =
                      Summe
                      Prüfsumme = Summe Modulo 64
*/

#include "WTDecoder.h"

WTDecoder wtd;

// global variables to be accessed from interrupt handler and Loop
uint64_t Codes[MAX_CODES];
volatile bool dataReady = false;
volatile byte CodeID = 0;

// interrupt service for 433MHz receiver attached to INT_PIN
void receiveWTsignal()
{
  static unsigned long lastRise = 0;
  static unsigned long lastTime = 0;
  static uint64_t code = 1;
  static byte id = 0;

  bool valid = false;
  unsigned long time_us = micros();

  if (digitalRead(INT_PIN)) {                   // rising edge
    lastRise = time_us;                         // store for measuring active modulation pulse length
  } else {                                      // falling edge
    unsigned long duration = time_us - lastRise;
    if (duration > 460ul && duration < 800ul) { // detected pulse between 460 and 800us
      // check for valid pulse length
      duration = lastRise - lastTime;           // calculate duration to last rising edge
      lastTime = lastRise;                      // store current rising edge
      if (duration > 2500ul && duration < 5000ul) {
        code <<= 1;                             // 0 detected
        if (duration > 4500ul) {                // 1 detected 
          code |= 1;                            // just put a 1 to LSB
        }
        valid = true;
      } else {                                  // wrong length or sync
        if (duration > 9500ul) {                // sync detected
          if ((code>>37) == 1ull) {             // rereived data of 37 bit length
            CodeID = id;                        // set current id for loop()
            Codes[id] = code;
            if (++id >= MAX_CODES) id = 0;
            dataReady = true;                   // loop() has to decode last transmission
            valid = true;
          }
          code = 1;                             // 1 is for length detection (gets shifted to pos 38 for a valid transmission)
        }
      }
    }
  }
#ifdef LED_PIN
  digitalWrite(LED_PIN, !valid);                // LED signals stable reception with long flashes for > 6x120 ms 
#endif
}

void WTDecoder::Setup()
{
#ifdef LED_PIN
  pinMode(LED_PIN, OUTPUT);
#endif
  attachInterrupt(digitalPinToInterrupt(INT_PIN), receiveWTsignal, CHANGE);
}

void WTDecoder::Loop()
{
  static int last_txid = 0;
  static unsigned last_recv = 0;
  
  if (dataReady) {
    byte index = CodeID;
    dataReady = false;
#ifdef WTD_DEBUG_VERBOSE
    // print unsigned long long as HEX number (ull not supported, split in two ul)
    unsigned long hi = Codes[index]>>32;
    unsigned long lo = Codes[index];
    char buf[20];
    sprintf(buf,"0x%x%08x ", hi, lo);
    Serial.print("Received: ");
    Serial.print(buf);
#endif
    unsigned long time_us = millis();
    byte index_prev = ((index == 0) ? MAX_CODES : index) - 1;
    // if received repeated codes (should be 6 identical transmissions) just accept the first one.
    // if there is only one tansmitter in reach the code may not change. Check for last reception time too. 
    if (Codes[index] != Codes[index_prev] || last_recv < time_us) {
      struct rec data;
      if (decodeRecord(Codes[index], data)) {
        storeRecord(data);
#ifdef WTD_DEBUG
        Serial.println(Record2String(data));
#endif
        last_recv = time_us + 30000ul;      // accept the next sequence in 30 seconds 
#ifdef WTD_DEBUG_VERBOSE
      } else {
        Serial.println("not valid");
#endif
      }
    } else {
#ifdef WTD_DEBUG_VERBOSE
      Serial.println();
#endif
    }
  }
}

bool WTDecoder::GetRecord(struct rec &record)
{
  bool ret = false;
  if (mLastStoredRec >= 0) {
    int last_read_rec = mLastReadRec + 1;
    if (last_read_rec >= MAX_RECORDS) last_read_rec = 0;
    if (last_read_rec != mLastStoredRec) {
      record = mRecords[last_read_rec];
      mLastReadRec = last_read_rec;
      ret = true;
    }
  }
  return ret; 
}

char *WTDecoder::Record2String(struct rec &record)
{
   sprintf(mBuffer, "Sensor %d : %0.1f°C %d%% (%02x%s%s)",
    record.channel + 1,
    (float) record.temperature / 10.0,
    record.humidity,
    record.txid,
    record.battery ? ", LOW BAT" : "",
    record.button ? ", TX BUT" : ""
  );
  return mBuffer;
}

void WTDecoder::storeRecord(struct rec &record)
{
  if (record.valid) {
    if (mLastStoredRec < 0) mLastStoredRec++;
    int last_stored_rec = mLastStoredRec + 1;
    if (last_stored_rec >= MAX_RECORDS) last_stored_rec = 0;
    if (last_stored_rec == mLastReadRec) mLastReadRec++;
    if (mLastReadRec >= MAX_RECORDS) mLastReadRec = 0;
    mRecords[mLastStoredRec] = record;
    mLastStoredRec = last_stored_rec;
  }
}

bool WTDecoder::decodeRecord(uint64_t value, struct rec &record)
{
  int check_calc = 0;
  int check_recv = value & 0x3Full;
  unsigned long tmp = (value>>5) & ~1l;         // clear LSB

  // calulate check sum
  for (int i = 0; i < 8; i++)
  {
    check_calc += (tmp & 0xFl);
    tmp >>= 4;
  }
  check_calc &= 0x3F;

  record.valid = check_recv == check_calc;
  
  if (record.valid)
  {
    tmp = value>>6;                            // skip checksum
    record.humidity = tmp & 0x7F;              // 7 bit humidity
    tmp >>= 7;
    int16_t val = (tmp&0xFFF)<<4;              // temperature is given in 12 bit 2th complement
    record.temperature = val / 16;             // preserve sign bit by shifting to MSB and dividing by shift factor
    tmp >>= 12;
    record.channel = tmp & 0x3;                // 2 bit channel
    tmp >>= 2;
    record.button = tmp & 0x1;                 // 1 bit button
    tmp >>= 1;
    record.battery = tmp & 0x1;                // 1 bit battery status
    tmp >>= 1;
    record.txid = tmp & 0xFF;                  // 8 bit transmitter ID
    
    // validate channel (exclude false positives)
    record.valid = record.channel >= 0 && record.channel <= 2;
#ifdef VALID_TX_ID_ONLY
    // Exclude unwanded results (e. g. false positives in the neighborhood)
    record.valid &= !(txid == 0xF0 || txid == 0x97);
#endif
  }
  return record.valid;
}

