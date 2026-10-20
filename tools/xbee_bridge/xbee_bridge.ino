// USB <-> XBee bridge for the GUI's "Pair XBees" window.
// Wiring: XBee DOUT -> D12, XBee DIN -> D13, XBee 5V/GND on an Explorer board.
// D0/D1 stay free, so the USB upload and the PC link never fight the XBee.
#include <SoftwareSerial.h>

// Pair XBees talks to a factory-fresh XBee at 9600 first, so keep this at 9600
// to program it. After it is programmed to 38400, set 38400 and re-upload to
// use Read settings again.
const long SOFT_BAUD = 38400;

SoftwareSerial xbee(12, 13);  // RX, TX

void setup() {
  Serial.begin(SOFT_BAUD);  // the GUI opens the port at this baud for the working attempt
  xbee.begin(SOFT_BAUD);
}

void loop() {
  while (xbee.available()) Serial.write(xbee.read());
  while (Serial.available()) xbee.write(Serial.read());
}
