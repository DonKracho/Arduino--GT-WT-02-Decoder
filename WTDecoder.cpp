/*
Funksensor GT-WT-02 (Globaltronics Gmbh & Co. KG, DomStr. 19, 20095 Hamburg)
Funksensor NT-1959 (Latupo GmbH, Walerloohain 5, 22769 Hamburg PJN-Nr.: EL201523-01) (Pollin)

für Wetter Stationen wie GT-WS-08 und GT-WS-09 (Aldi)

Ein Sendimpuls ist ca. 600us lang gefolgt von einer Pause
Eine Puls Pause Folge entspricht einem Bit ode Sync Impuls
die Länge der Pause ist hier definiert zu:
2070us 0
4140us 1
9060us Start/End
Sync between Transmits 9060 20180 9060
   _
0 | |_ _ _ 2700us             ca. 2/7 * 300us
   _
1 | |_ _ _ _ _ _  4730 us     ca. 2/14 * 300us
   _                                 _       _
S | |_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _| |_ _ _| |_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ Beginn der Übertragung

Protokoll: 37 Bit pro Daten Paket
Zwischen den Datenpaketen ist ein Sync Impuls und wird insgesamt 6 mal wiederholt
Der Anfang der ersten Übertragung kann stark gestört sein bis die AGC des Empfägers (Pendel-Audion) eingeregelt ist.

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

// global variables to be accessed from interupt handler and Loop
uint64_t Codes[MAX_CODES];
volatile bool dataReady = false;
volatile byte CodeID = 0;

// interrupt service for 433MHz receiver attached to INT_PIN
void receiveWTsignal()
{
  static unsigned long lastInt  = 0;
  static unsigned long lastTime = 0;
  static uint64_t code = 0;
  static byte id = 0;

  bool valid = false;
  unsigned long duration;
  unsigned long time_us = micros();

  if (digitalRead(INT_PIN)) {       // was rising edge
    lastInt = time_us;
    // check duration to last rising edge
    duration = time_us - lastTime;
    if (duration < 2400ul) {         // no valid duration
      valid = false;
    } else if (duration < 3000ul) {  // 0 detected
      code <<= 1;
    } else if (duration < 5000ul) {  // 1 detected 
      code <<= 1;
      code |= 1;
    } else {                        // sync detected
      if (code & 0xFF00000000ull) {
        CodeID = id;                // set current id for loop()
        Codes[id] = code;
        if (++id >= MAX_CODES) id = 0;
        dataReady = true;           // loop() has to decode last transmission
      }
      code = 0;
    }
  } else {                          // was falling edge
    duration = time_us - lastInt;
    if (duration < 500ul) {         // noise filter - pulse to short
      valid = false;
    } else if (duration < 1100ul) { // detected pulse between 500 and 1100us
      valid = true;
      lastTime = lastInt;
    }
  }
#ifdef WTD_DEBUG
  digitalWrite(LED_PIN, !valid);
#endif
}

void WTDecoder::Setup()
{
#ifdef WTD_DEBUG
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
    byte index_prev = ((index == 0) ? MAX_CODES : index) - 1;
    if (Codes[index] != Codes[index_prev]) {
      struct rec data;
      if (decodeRecord(Codes[index], data)) {
        unsigned long time_us = millis();
        if (last_txid != data.txid || last_recv < time_us) {
#ifdef WTD_DEBUG
          printRecord(data);
#endif
          last_txid = data.txid;
          last_recv = time_us + 30000ul;
        }
#ifdef WTD_DEBUG_VERBOSE
      } else {
        unsigned long hi = Codes[index]>>32;
        unsigned long lo = Codes[index];
        Serial.print("Received ");
        Serial.print(": ");
        Serial.print(hi, HEX);
        Serial.print(lo, HEX);
        Serial.println(" not valid");
#endif
      }
    }
  }
}

bool WTDecoder::decodeRecord(uint64_t value, struct rec &record)
{
  int check_calc = 0;
  int check_recv = value & 0x3FL;
  unsigned long tmp = (value>>5) & ~1l;

  for (int i = 0; i < 8; i++)
  {
    check_calc += (tmp & 0xFl);
    tmp >>= 4;
  }
  check_calc &= 0x3FL;

  record.valid = check_recv == check_calc;
  
  if (record.valid)
  {
    tmp = value>>6;                             // skip checksum
    record.humidity = tmp & 0x7F;              // 7 bit humidity
    tmp >>= 7;
    record.temprature = ((tmp&0xFFF)<<4) / 16; // 12 bit 2th complement
    tmp >>= 12;
    record.channel = (tmp & 0x3);              // 2 bit channel
    tmp >>= 2;
    record.button = tmp & 0x1;                 // 1 bit button
    tmp >>= 1;
    record.battery = tmp & 0x1;                // 1 bit battery status
    tmp >>= 1;
    record.txid = tmp & 0xFF;                  // 8 bit transmitter ID
    
    // validate channel (exclude false positives)
    record.valid = record.channel >= 0 && record.channel <= 3;
#ifdef VALID_TX_ID_ONLY
    // Exclude unwanded results (e. g. false positives in the neighborhood)
    record.valid &= !(txid == 0xF0 || txid == 0x97);
#endif
  }
  return record.valid;
}

void WTDecoder::printRecord(struct rec &record)
{
  if (record.valid) {
    Serial.print("Sensor ID ");
    Serial.print(record.channel + 1);
    Serial.print(": ");
    Serial.print(record.temprature / 10);
    Serial.print(".");
    Serial.print(abs(record.temprature) % 10);
    Serial.print("°C ");
    Serial.print(record.humidity);
    Serial.print("% ");
    Serial.print(record.txid, HEX);
    Serial.print(" ");
    if (record.button) Serial.print("BUTTON ");
    if (record.battery) Serial.print("BAT LOW");
    Serial.println();
  }
}

