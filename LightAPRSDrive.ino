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

char CallSign[7] = "KQ4VKK";
int CallNumber = 11;
char Symbol = 'O';
char StatusMessage[50] = "Vanderbilt VADL Test Message";
unsigned int BeaconWait = 10;
uint16_t TxCount = 1;
int ENABLE = 0; // Enable

float receivedFloats[13];  // Buffer to store the 13 received floats
int floatCount = 0;       // Count of received floats
const float END_MARKER = -999.0;

void setup() {
  Wire.begin(0x8);  // Join I2C bus as slave with address 8
  Wire.onReceive(receiveEvent);  // Register receive event
  Serial.begin(9600);  // Start Serial for debugging
  Serial.println("Arduino ready and waiting for I2C data...");

  pinMode(RfPDPin, OUTPUT);
  pinMode(RfPwrHLPin, OUTPUT);
  pinMode(RfPttPin, OUTPUT);
  pinMode(7, INPUT); // Enable Pin, SCK
  RfOFF;
  RfPwrHigh;
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
  // ENABLE = digitalRead(7); // Check for ENABLE Signal
  ENABLE = HIGH;

  if (ENABLE == HIGH) {
    Serial.print(F("Sending status message, TX Count: "));
    Serial.println(TxCount);

    Wire.onReceive(nullptr);
    sendStatus(messageGroup);
    Wire.onReceive(receiveEvent);

    Serial.println(F("Status message sent, entering delay..."));
    delay(BeaconWait * 1000);

    // Increment and wrap around message group using modulo
    messageGroup = (messageGroup + 1) % 4;
  } else {
    Serial.println("RF Disabled");
  }
}

int aprs_msg_callback(AX25Msg*) {
  // Important function - placeholder for handling received APRS messages
}

void sendStatus(int group) {
  char status_buff[200] = "";  // Properly initialize the buffer
  char floatStr[10];  // Temporary buffer to hold each float as a string

  switch (group) {
    case 0:
      // Construct the status buffer for floats 1-4: "T: float1, A: float2, B: float3, S: float4"
      strcat(status_buff, "T:");
      // dtostrf(receivedFloats[0], 3, 2, floatStr);
      dtostrf(receivedFloats[0] * 9.0 / 5.0 + 32, 3, 2, floatStr);  // Convert °C to °F
      strcat(status_buff, floatStr);

      strcat(status_buff, " A:");
      // dtostrf(receivedFloats[1], 3, 2, floatStr);
      dtostrf(receivedFloats[1] * 3.28084, 3, 2, floatStr);  // Convert meters to feet
      strcat(status_buff, floatStr);

      strcat(status_buff, " B:");
      dtostrf(receivedFloats[2], 3, 2, floatStr);
      strcat(status_buff, floatStr);

      strcat(status_buff, " S:");
      dtostrf(receivedFloats[3], 3, 2, floatStr);
      strcat(status_buff, floatStr);
      break;

    case 1:
      // Construct the status buffer for floats 5-8: "Qw: float5, Qx: float6, Qy: float7, Qz: float8"
      strcat(status_buff, "Qw:");
      dtostrf(receivedFloats[4], 3, 2, floatStr);
      strcat(status_buff, floatStr);

      strcat(status_buff, " Qx:");
      dtostrf(receivedFloats[5], 3, 2, floatStr);
      strcat(status_buff, floatStr);

      strcat(status_buff, " Qy:");
      dtostrf(receivedFloats[6], 3, 2, floatStr);
      strcat(status_buff, floatStr);

      strcat(status_buff, " Qz:");
      dtostrf(receivedFloats[7], 3, 2, floatStr);
      strcat(status_buff, floatStr);
      break;

    case 2:
      // Construct the status buffer for floats 9-11: "LandingT: int(float9):int(float10):int(float11)"
      strcat(status_buff, "LandingT:");
      sprintf(floatStr, "%d", (int)receivedFloats[8]);  // Cast float to int and format as string
      strcat(status_buff, floatStr);

      strcat(status_buff, ":");
      sprintf(floatStr, "%d", (int)receivedFloats[9]);  // Cast float to int and format as string
      strcat(status_buff, floatStr);

      strcat(status_buff, ":");
      sprintf(floatStr, "%d", (int)receivedFloats[10]);  // Cast float to int and format as string
      strcat(status_buff, floatStr);
      break;


    case 3:
      // Construct the status buffer for floats 12-13: "MaxV: float12, LV: float13"
      strcat(status_buff, "MaxV:");
      // dtostrf(receivedFloats[11], 3, 2, floatStr);
      dtostrf(receivedFloats[11] * 3.28084, 3, 2, floatStr);  // Convert m/s to fps
      strcat(status_buff, floatStr);

      strcat(status_buff, " LV:");
      // dtostrf(receivedFloats[12], 3, 2, floatStr);
      dtostrf(receivedFloats[12] * 3.28084, 3, 2, floatStr);  // Convert m/s to fps
      strcat(status_buff, floatStr);
      break;
  }

  Serial.print(F("Status buffer: "));
  Serial.println(status_buff);

  // Proceed with APRS transmission
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
}
