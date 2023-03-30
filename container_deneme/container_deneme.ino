#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>
#include <TinyGPSPlus.h>


unsigned long timer = 0;
unsigned long lastMillis = 0;
byte pressureAddrL = 0;
byte pressureAddrH = 0;
int pressure = 0;
char  basincL = 0;
char basincH = 0;
int8_t buzzer = 4; // Buzzer'in + bacağının bağlı olduğu arduino pini
int8_t state = 0;
int8_t buzzerState = 0;
float prevAlti = 0;
float alti = 0;
String telemetry;
static const int RXPin = 5, TXPin = 6;
static const uint32_t GPSBaud = 9600;

TinyGPSPlus gps;
Adafruit_BMP280 bmp;
SoftwareSerial mySerial(2, 3); // RX, TX
SoftwareSerial ss(RXPin, TXPin);


void setup() {
  Serial.begin(9600);
  mySerial.begin(9600);
  if (bmp.begin(0x76));
    Serial.println("bmp connection succesful");
  ss.begin(GPSBaud);
  EEPROM.write(pressureAddrH, 0);
  EEPROM.write(pressureAddrL, 0);
  if (EEPROM.read(pressureAddrL) == 0 && EEPROM.read(pressureAddrH) == 0)
  {
    pressure = bmp.readPressure() * 10;
    basincL = pressure && 0xFF;
    basincH = pressure >> 8;
    EEPROM.write(pressureAddrL, basincL);
    EEPROM.write(pressureAddrH, basincH);
  }
  pinMode(buzzer,OUTPUT);
}

void loop() {
  prevAlti = alti;
  float basinc = bmp.readPressure();
  alti = bmp.readAltitude((float)(EEPROM.read(pressureAddrL) | (EEPROM.read(pressureAddrH) << 8)) / 10);
  int vSpeed = (alti - prevAlti) / ((millis() - timer) / 1000.0);
  if (vSpeed < -5 && state == 0)
    state = 1;
  else if (vSpeed < 5 && vSpeed > -5 && alti <= 10 && alti >= 10 && state == 1)
    state = 2;
  if (millis() - timer > 1000)
  {
    if (alti < 10 && state == 2 && buzzerState == 0)
    {
      buzzerState = 1;
      digitalWrite(buzzer,HIGH); //Buzzer'a güç göndererek ses çıkarmasını sağladık.
    }
    else if (alti < 10 && state == 2 && buzzerState == 1)
    {
      buzzerState = 0;
      digitalWrite(buzzer,LOW); //Buzzerın gücünü keserek susmasını sağladık sağladık.
    }
    telemetry = String(basinc) + ","
    + String(gps.location.lat()) + ","
    + String(gps.location.lng()) + ","
    + String(gps.altitude.meters()) + ",\n";
    if(alti < 450)
      mySerial.println(telemetry);
    if (millis() < lastMillis) {
      timer += ULONG_MAX - lastMillis + millis() + 1;
    } 
    else {
      timer += millis() - lastMillis;
    }
    lastMillis = millis();
    Serial.print(vSpeed);
    Serial.print(",");
    Serial.println(telemetry);
  }
}
