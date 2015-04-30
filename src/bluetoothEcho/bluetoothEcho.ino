
/*

Written by Dipl.-Ing. Roland Littwin (Repetier)
Licence: GPL V3

This little sketch is to help configuring the bluetooth adapter if you have no
success doing it form the remote side. How does it work:

1. Uploading sketch with BTSerial and BTBaudrate set for your adapter. Most come with factory
set to 9600, 38400 or 115200.

2. Connect with Arduino IDE serial console and 115200 baud, line ending LF

3. Send command line by line. Pause a second between commands.

Once your adapter is configured, you can upload the read firmware.

Typical commands for HC-06

AT
Just a function test
AT+NAMEnewname
Changes adapter name to newname
AT+BAUD8
Sets baudrate to 115200

*/

// On a due Serial1, Serial2 and Serial3 are possible.
#define BTSerial Serial1
// Use ansi baud rates: 9600, 19800, 38400,  57600, 115200, 230400
#define BTBaudrate 9600

void setup() {
  Serial.begin(115200);
  Serial1.begin(BTBaudrate);

}

char text[200];
int textPos = 0;
void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      Serial.print("Send command:");
      text[textPos] = 0;
      Serial.println(text);
      for (int i = 0; i < textPos; i++) {
        BTSerial.write(text[i]);
      }
      textPos = 0;
    } else {
      text[textPos++] = c;
    }
  }
  if (BTSerial.available()) {
    Serial.write(BTSerial.read());
  }
}
