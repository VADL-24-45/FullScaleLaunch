#include <Wire.h>
#include <LibAPRS.h>
#include <SoftwareSerial.h>
#include <LowPower.h>
#include <EEPROM.h>

#define RfPDPin     19
#define RfPwrHLPin  21
#define RfPttPin    20
#define PIN_DRA_RX  22
#define PIN_DRA_TX  23

#define ADC_REFERENCE REF_3V3
#define OPEN_SQUELCH false

#define RfON          digitalWrite(RfPDPin, HIGH)
#define RfOFF         digitalWrite(RfPDPin, LOW)
#define RfPwrHigh     pinMode(RfPwrHLPin, INPUT)
#define RfPwrLow      pinMode(RfPwrHLPin, OUTPUT);digitalWrite(RfPwrHLPin, LOW)
#define RfPttON       digitalWrite(RfPttPin, HIGH)
#define RfPttOFF      digitalWrite(RfPttPin, LOW)
#define AprsPinInput  pinMode(12, INPUT);pinMode(13, INPUT);pinMode(14, INPUT);pinMode(15, INPUT)
#define AprsPinOutput pinMode(12, OUTPUT);pinMode(13, OUTPUT);pinMode(14, OUTPUT);pinMode(15, OUTPUT)

char CallSign[7] = "KQ4VMR";
int CallNumber = 11;
char Symbol = 'O';
char StatusMessage[50] = "Vanderbilt VADL Test Message";
unsigned int BeaconWait = 10;
uint16_t TxCount = 1;

float receivedFloats[13];  // Updated buffer to store the 13 received floats
int floatCount = 0;        // Count of received floats
const float END_MARKER = -999.0;

void setup() {
  Wire.begin(0x8);  // Join I2C bus as slave with address 8
  Wire.onReceive(receiveEvent);  // Register receive event
  Serial.begin(9600);  // Start Serial for debugging
  Serial.println("Arduino ready and waiting for I2C data...");

  pinMode(RfPDPin, OUTPUT);
  pinMode(RfPwrHLPin, OUTPUT);
  pinMode(RfPttPin, OUTPUT);
  RfOFF;
  RfPwrLow;
  RfPttOFF;

  Serial.println(F("APRS minimalistic beacon started"));
  APRS_init(ADC_REFERENCE, OPEN_SQUELCH);
  APRS_setCallsign(CallSign, CallNumber);
  APRS_setPath1("WIDE1", 1);
  APRS_setPath2("WIDE2", 1);
  APRS_setSymbol(Symbol);
  APRS_setPreamble(350UL);
  Serial.println(F("APRS setup complete"));
}

void receiveEvent(int howMany) {
  while (howMany >= 5) {  // Expect 5 bytes: 1 command byte + 4 bytes for float
    byte buffer[4];  // Temporary buffer for float data
    Wire.read();  // Discard the first byte (command byte)

    // Read the next 4 bytes (float data)
    for (int i = 0; i < 4; i++) {
      buffer[i] = Wire.read();
    }

    // Convert the 4 bytes into a float and store it in receivedFloats
    float newFloat;
    memcpy(&newFloat, buffer, sizeof(newFloat));

    // Check if the received float is the end marker
    if (newFloat == END_MARKER) {
      Serial.println("All 13 floats received successfully:");
      for (int i = 0; i < 13; i++) {
        Serial.print("Float ");
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.println(receivedFloats[i], 6);
      }
      floatCount = 0;  // Reset count for next set of floats
    } else if (floatCount < 13) {
      receivedFloats[floatCount] = newFloat;
      floatCount++;  // Increment float count
    }

    howMany -= 5;  // Reduce howMany by 5, since each float transmission is 5 bytes
  }
}

void loop() {
  static int messageGroup = 0;  // Counter to track which message group to send

  Serial.print(F("Sending status message, TX Count: "));
  Serial.println(TxCount);

  Wire.onReceive(nullptr);
  sendStatus(messageGroup);
  Wire.onReceive(receiveEvent);

  Serial.println(F("Status message sent, entering delay..."));
  delay(BeaconWait * 1000);

  // Increment and wrap around message group using modulo
  messageGroup = (messageGroup + 1) % 2;
}

void sendStatus(int group) {
  char status_buff[200] = "";  // Properly initialize the buffer
  char floatStr[10];  // Temporary buffer to hold each float as a string

  switch (group) {
    case 0:
      // Construct the status buffer for the first set of floats (first 6)
      strcpy(status_buff, "T");
      for (int i = 0; i < 6; i++) {
        dtostrf(receivedFloats[i], 3, 2, floatStr);
        strcat(status_buff, " ");
        strcat(status_buff, floatStr);
      }
      break;

    case 1:
      // Construct the status buffer for the second set of floats (remaining 7)
      for (int i = 6; i < 13; i++) {
        strcat(status_buff, " ");
        dtostrf(receivedFloats[i], 3, 2, floatStr);
        strcat(status_buff, floatStr);
      }
      break;
  }

  Serial.print(F("Status buffer: "));
  Serial.println(status_buff);

  // RF transmission code remains unchanged
  AprsPinOutput;
  RfON;
  delay(2000);
  RfPttON;
  delay(1000);

  Serial.println(F("Transmitting status message..."));
  APRS_sendStatus(status_buff, strlen(status_buff));
  delay(50);
  while (digitalRead(1)) {;}
  
  delay(50);
  RfPttOFF;
  RfOFF;
  AprsPinInput;

  Serial.println(F("Transmission complete, RF off"));

  TxCount++;

  // Clear the transmitted floats after sending
  int startIndex = (group == 0) ? 0 : 6;
  int endIndex = (group == 0) ? 6 : 13;
  for (int i = startIndex; i < endIndex; i++) {
    receivedFloats[i] = 0.0;
  }
}

int aprs_msg_callback(AX25Msg*) {
  // Important function - placeholder for handling received APRS messages
}
