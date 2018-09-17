#include "WTDecoder.h"

void setup()
{
  Serial.begin(115200);
  wtd.Setup();
}

void loop()
{
  wtd.Loop();
}
