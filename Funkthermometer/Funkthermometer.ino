/*****************************************************************************************************
*  Author:          Wolfgang Kracht
*  Initial release: 09/09/2018
*  Dependencies:    ESP8266 core for Arduino (refer to: https://github.com/esp8266/Arduino)
*****************************************************************************************************/

#include "WTDecoder.h"

void setup()
{
  Serial.begin(115200);
  delay(500);
  Serial.println("Starting scanning for GT-WT-02 codes");
  wtd.Setup();
}

void loop()
{
  wtd.Loop();
}
