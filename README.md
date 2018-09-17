# GT-WT-02-Decoder for Arduino projects

This project is for decoding 433MHz AM transmissions of a GT-WT-02 weather station transmitter.
There are a lot of identical transmitters sending a 37bit code for temperature and humidity.

The code is designed for the Arduino IDE and a ESP8266 device but should work with Arduino hardware too.

As additional hardware you will need a 433MHz AM receiver for generating pin interrupts.
It is strongly recommended to use a Superheterodyne 433MHz RF module like the 3400RF or RXB6.
These receivers operate at 3.3V without any issues.

This project just incudes a class WTDecoder with the decoding algorithm and storing the decoded
data into a structure. This structue is visualized by simple Serial.print() commands. It is up to
you to implement the data publishing to a destination of your choice (e. g. a web page or a spread sheet)

The Rx data pulses are captured and analyzed interrupt driven and the decoding is done in the arduino loop().
The challenge of this project was the digital filtering the AM signal out of the heavy noise of the 433MHz band,
to achieve the best possible reception range.
