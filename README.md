# This repository is not maintained anymore!

Please refer to new ESPhome repository:
[ESPHome-component-433MHz-Sensor-Hub](https://github.com/DonKracho/ESPHome-component-433MHz-Sensor-Hub)

# GT-WT-02 Decoder for Arduino projects

This project is for decoding 433MHz AM transmissions of a GT-WT-02 and AFT 77 A1 weather station transmitters.
There are a lot of identical transmitters sending a 37bit code for temperature and humidity.

The code is designed for the Arduino IDE and a ESP8266 device but should work with Arduino hardware too.

As additional hardware you will need a 433MHz AM receiver for generating pin interrupts.
It is strongly recommended to use a Superheterodyne 433MHz RF module like the 3400RF or RXB6.
These receivers operate at 3.3V without any issues. NOTE: the data output of these receivers is level based,
but has a high impedance. Therefore it may not work at GPIO pins with integrated pull up/pull down resistors.

This project just incudes a class WTDecoder with the decoding algorithm and storing the decoded
data into a structure. This structue is visualized by simple Serial.print() commands. It is up to
you to implement the data publishing to a sink of your choice (e. g. web page or data base)

The Rx data pulses are captured and analyzed interrupt driven and the decoding is done in the arduino loop().

The challenge of this project has been the detection and digital noise canceling of the weak AM signal of the
transmitter out of a very noisy 433MHz band environment. Therefore pulse detection in the interrupt service
routine relies on active periods of the AM signal only to achieve the best possible reception range.
