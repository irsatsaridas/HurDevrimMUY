#include "MPU6050_6Axis_MotionApps20.h"
#include <EEPROM.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <RTClib.h>
#include <Arduino.h>
#include <math.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <Servo.h>
//#include <SPI.h>
//#include <SD.h>
#define VIDEO_SIZE 500000

  int servoPos = 38;
  int8_t buzzer = 4;
  char basincL = 0;
  char basincH = 0;
  byte ayrildi = 0;
  byte aktarildi = 0;
  unsigned long packNumber = 0;
  byte packAddr = 0;
  byte packDecAddr = 1;
  byte pressureAddrLow = 2;
  byte pressureAddrHigh = 3;
  byte hourAddr = 4;
  byte minAddr = 5;
  byte secAddr = 6;
  byte state = 0;
  int8_t err[5] = {0};
  unsigned long elapsedTime = 0;
  String container;
  String conPressure;
  String conGPS1;
  String conGPS2;
  String conGPS3;
  float pitch = 0;
  float roll = 0;
  float yaw = 0;
  float voltage;
  int altitude;
  int pressure;
  unsigned long timer = 0;

  /* MPU6050 */
  bool dmpReady = false;  // set true if DMP init was successful
  uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
  uint16_t fifoCount;     // count of all bytes currently in FIFO
  uint8_t fifoBuffer[64]; // FIFO storage buffer
  Quaternion q;           // [w, x, y, z]         quaternion container
  VectorFloat gravity;    // [x, y, z]            gravity vector
  float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

MPU6050 mpu(0x69);
Adafruit_BMP280 bmp;
RTC_DS3231 rtc;
TinyGPSPlus gps;
SoftwareSerial ss(2, 3);
Servo servo;
//File teleSD;
//File vidSD;

void setup() {
  Wire.begin();
  Serial.begin(9600);
  Serial3.begin(9600); // RX3, TX3
  ss.begin(9600);
  rtc.begin();
  Serial.println("RTC connection successful");
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  
  mpu.initialize();
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  devStatus = mpu.dmpInitialize();
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateGyro(6);
    //mpu.PrintActiveOffsets();
    mpu.setDMPEnabled(true);
    dmpReady = true;
  } else {
     // ERROR!
     // 1 = initial memory load failed
     // 2 = DMP configuration updates failed
     // (if it's going to break, usually the code will be 1)
     Serial.print(F("DMP Initialization failed (code "));
     Serial.print(devStatus);
     Serial.println(F(")"));
  }
  bmp.begin(0x76);
  Serial.println("BMP280 connection successful");
  pinMode(23, INPUT);
  servo.attach(13);
  //if (!SD.begin()) {
  //  Serial.println("SD CARD initialization failed!");
  //  return;
  //}
  //Serial.println("SD CARD initialization done.");
  if (EEPROM.read(pressureAddrLow) == 0 && EEPROM.read(pressureAddrHigh) == 0)
  {
    pressure = bmp.readPressure() * 10;
    basincL = pressure && 0xFF;
    basincH = pressure >> 8;
    EEPROM.write(pressureAddrLow, basincL);
    EEPROM.write(pressureAddrHigh, basincH);
  }
}

void resetEEPROM(void);

void loop() {
  timer = millis();
  /* Taşıyıcıdan gelen basınç verisi*/
  if (Serial3.available()){
    container = ""; // Initialize an empty string to store the message
    while (Serial3.available()) { // Keep reading characters until there are no more
      char c = Serial3.read();
      container += c;
    }
    //Serial.println(container);
    if (container.charAt(0) == '/' && state < 4) // xbee konfigürsayonunu dğeiştir
    {
      if (container.equals("/servo"))
      { 
        if (servoPos > 0){
          for (servoPos; servoPos <= 38; servoPos += 1)
          {
            servo.write(servoPos);
            delay(10);
          }
        }
        else{
          for (servoPos; servoPos >= 3; servoPos -= 1)
          {
            servo.write(servoPos);
            delay(10);
          }
        }
      }
      else if (container.equals("/res"))
      {
        resetEEPROM();
      }
    }
    else
    {
      int i = container.indexOf(',');
      conPressure = container.substring(0, i);
      //Serial.println(conPressure);
    
      container = container.substring(i + 1);
      i = container.indexOf(',');
      conGPS1 = container.substring(0, i);
      //Serial.println(conGPS1);
    
      container = container.substring(i + 1);
      i = container.indexOf(',');
      conGPS2 = container.substring(0, i);
      //Serial.println(conGPS2);
    
      container = container.substring(i + 1);
      i = container.indexOf(',');
      conGPS3 = container.substring(0, i);
      //Serial.println(conGPS3);  
     }
  }
  float prevAltitude = altitude;                        //Yükseklik değişimi [m]
  float temp = bmp.readTemperature();                   //Sıcaklık [C]
  float pressure = bmp.readPressure();         //Basınç [hPa]
  altitude = (pow((((float)(EEPROM.read(pressureAddrLow) | (EEPROM.read(pressureAddrHigh) << 8)) / 10) / pressure), 1 / 5.257) - 1) * (temp + 273.15) / 0.0065;
  float conAltitude = (pow((((float)(EEPROM.read(pressureAddrLow) | (EEPROM.read(pressureAddrHigh) << 8)) / 10) / conPressure.toFloat()), 1 / 5.257) - 1) * (temp + 273.15) / 0.0065;

  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  }

  float vSpeed = prevAltitude - altitude;           // Görev Yükü Hız Tayini
  int analogV = analogRead(24);                        // Analog pinden okunan bölünmüş değer
  float voltage = 2 * (float)analogV * 5.0f / 1023.0f;        // Voltaj hesabı

  DateTime now = rtc.now(); // Read the time from the RTC module
  if (EEPROM.read(hourAddr) == 0 && EEPROM.read(minAddr) == 0 && EEPROM.read(secAddr) == 0)
  {
    EEPROM.write(hourAddr, now.hour());
    EEPROM.write(minAddr, now.minute());
    EEPROM.write(secAddr, now.second());
  }
  
  elapsedTime = 3600 * (now.hour() - EEPROM.read(hourAddr)) + 60 * (now.minute() - EEPROM.read(minAddr)) + (now.second() - EEPROM.read(secAddr)); // Geçen süreyi güncel saat - başlangıç saati ile hesaplar

  if (altitude < 470 && altitude > 430)
  {
    for (servoPos; servoPos >= 3; servoPos -= 1)
    {
      servo.write(servoPos);
      delay(10);
    }
    if (vSpeed > -10)
    ayrildi = 1;
  }

  /****************************************************/
  /*!!!               UYDU STATÜSÜ                 !!!*/
  /****************************************************/
  if (altitude > 10 && vSpeed > 3 && state == 0)
    state = 1;
  else if (vSpeed < 0 && ayrildi == 0 && altitude > 470 && state == 1)
    state = 2;
  else if (vSpeed < 0 && ayrildi == 0 && altitude < 470 && altitude > 430 && state == 2)
    state = 3;
  else if (vSpeed < 0 && ayrildi == 1 && state == 3)
    state = 4;
  else if (vSpeed < 2 && vSpeed > -2 && altitude < 10 && state == 4)
    state = 5;
  //vidSD = SD.open("video.avi", FILE_READ);
  //if (vidSD != 0 && vidSD.size() == VIDEO_SIZE && !aktarildi)
  //{
  //  state = 6;
  //  aktarildi = 1;
  //}
  //vidSD.close();

  if (altitude < 10 && state == 5)
  {
    digitalWrite(buzzer,HIGH); //Buzzer'a güç göndererek ses çıkarmasını sağladık.
    delay(500);                  // 1 saniye boyunca çalmasını söyledik.
    digitalWrite(buzzer,LOW); //Buzzerın gücünü keserek susmasını sağladık sağladık.
  }
  

  /****************************************************/
  /*!!!                 ARAS                       !!!*/
  /****************************************************/
  if ((vSpeed < 12 || vSpeed > 14) && altitude > 450 && ayrildi == 0)
    err[0] = 1;
  else
    err[0] = 0;
  if ((vSpeed < 6 || vSpeed > 8) && altitude < 450)
    err[1] = 1;
  else
    err[1] = 0;
  if (conGPS1.toInt() == 0 && conGPS2.toInt() == 0 && conGPS3.toInt() == 0)
    err[2] = 1;
  else
    err[2] = 0;
  if (!gps.location.isValid())
    err[3] = 1;
  else
    err[3] = 0;
  if (altitude < 470 && altitude > 430 && ayrildi == 0)
    err[4] = 1;
  else
    err[4] = 0;


  /****************************************************/
  /*!!!                 PAKET NUMARASI             !!!*/
  /****************************************************/

  //  EEPROM maksimum 255'e kadar depolayabilir. 
  //  Bu yüzden paket numarasını 255'e bölümünden kalan ve 255'e bölümü olarak iki parça halinde saklıyoruz.
  packNumber = EEPROM.read(packAddr) + 255 * EEPROM.read(packDecAddr); // her saniye veri yazma, voltaş düştüğünde tek sefer kaydet, yenileme hızını arttır
  EEPROM.write(packDecAddr, packNumber / 255);
  EEPROM.write(packAddr, packNumber % 255);

  
  String telemetry; //String formatında telemetri verileri

  telemetry = "<" + String((int)EEPROM.read(packAddr)) + ">, " 
  + "<" + String(state) + ">, "
  + "<" + String(err[0]) + String(err[1]) + String(err[2]) + String(err[3]) + String(err[4]) + ">, " // Aras hata kodları, taşıyıcı konumu hariç ('T')
  + "<" + String(now.day()) + "/" + String(now.month()) + "/" + String(now.year()) + "," + String(now.hour())
  + "/" + String(now.minute()) + "/" + String(now.second()) + ">, " 
  + "<" + String((int)pressure) + ">, "
  + "<" + conPressure + ">, "
  + "<" + String(altitude) + ">, "
  + "<" + String(conAltitude) + ">, "
  + "<" + String(altitude - conAltitude) + ">, "
  + "<" + String(vSpeed) + ">, "
  + "<" + String(temp) + ">, "
  + "<" + String(voltage) + ">, "
  + "<" + String(gps.location.lat()) + ">, "
  + "<" + String(gps.location.lng()) + ">, "
  + "<" + String(gps.altitude.meters()) + ">, "
  + "<" + conGPS1 + ">, "
  + "<" + conGPS2 + ">, "
  + "<" + conGPS3 + ">, "
  + "<" + String(ypr[0] * 180/M_PI) + ">, "
  + "<" + String(ypr[1] * 180/M_PI) + ">, "
  + "<" + String(ypr[2] * 180/M_PI) + ">, "
  + "<" + "249224" + ">, \n";

  // Toplanan telemetri verileri "SD/telemetry.txt" konumuna kaydedilir.
  Serial.println(telemetry);
  Serial3.print(telemetry);
  //teleSD = SD.open("telemetry.txt", FILE_WRITE);
  //if (teleSD) {
  //  teleSD.print(telemetry);
  //  teleSD.close();
  //}

  // Zamanlayıcı işlem zamanını hesaplar, geçen zamanı 1 saniyeye tamamlayarak işlem frekansını 1Hz olarak belirler.
  if (1000 - (millis() - timer) > 0)
    delay(1000 - (millis() - timer));
}

  // Cold Start işlemi için EEPROM'da kaydedilen verileri ve SD karta kayıtlı telemetri verilerini siler.
void resetEEPROM(void)
{
  EEPROM.write(packAddr, 0);
  EEPROM.write(pressureAddrLow,0);
  EEPROM.write(pressureAddrHigh,0);
  EEPROM.write(hourAddr, 0);
  EEPROM.write(minAddr, 0);
  EEPROM.write(secAddr, 0);
  remove("telemetry.txt");
  Serial.println("All EEPROM values are successfully set to 0");
}
