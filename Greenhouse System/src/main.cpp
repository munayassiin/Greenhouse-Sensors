#include <WiFi.h>
#include "ThingSpeak.h"
#include <Adafruit_Sensor.h>
#include <SPI.h>
#include "Adafruit_SHT31.h"
#include "Adafruit_LTR329_LTR303.h"

Adafruit_LTR329 ltr = Adafruit_LTR329();

 //Wifi

const char* ssid = "Telenor6950ovn";   
const char* password = "onicpjozepykk";   

WiFiClient  client;

//Thingspeak channel & API

unsigned long myChannelNumber = 2106350 ;  
const char * myWriteAPIKey = "YBIMKD1GRGUTE6H3";  

unsigned long lastTime = 0;
unsigned long timerDelay = 30000; 

Adafruit_SHT31 sht31 = Adafruit_SHT31();  

void initSHT31(){
  if (!sht31.begin(0x44)) {  
    Serial.println("Couldn't find SHT31");
    while (1) delay(1);
  }
}



void setup() {
  Serial.begin(115200);  
  initSHT31();  
  
  WiFi.mode(WIFI_STA);  
  
  ThingSpeak.begin(client);
    if ( ! ltr.begin() ) {
    Serial.println("Couldn't find LTR sensor!");
    while (1) delay(10);
  }

  // Setup LTR sensor (see advanced demo in library for all options!)
  Serial.println("Found LTR sensor!");
  ltr.setGain(LTR3XX_GAIN_4);
  ltr.setIntegrationTime(LTR3XX_INTEGTIME_50);
  ltr.setMeasurementRate(LTR3XX_MEASRATE_50);  // Initialize ThingSpeak
}

void loop() {

  //Light
    uint16_t visible_plus_ir, infrared;
  if (ltr.newDataAvailable()) {
    bool valid = ltr.readBothChannels(visible_plus_ir, infrared);
    if (valid) {
      Serial.print("CH0 Visible + IR: ");
      Serial.print(visible_plus_ir);
      Serial.print("\tCH1 Infrared: ");
      Serial.println(infrared);
    }
      delay(5000);
  }
  if ((millis() - lastTime) > timerDelay) {  
    if(WiFi.status() != WL_CONNECTED){  // Check if WiFi is not connected
      Serial.print("Attempting to connect");
      while(WiFi.status() != WL_CONNECTED){  // Try to connect to WiFi
        WiFi.begin(ssid, password); 
        delay(5000);     
      } 
      Serial.println("\nConnected.");
    }

    //Humidity & Tempetuar 

    float humidity = sht31.readHumidity();  
    float temperatureC = sht31.readTemperature();
    Serial.print("Temperature: ");
    Serial.println(temperatureC);
    Serial.print("Humidity: ");
    Serial.println(humidity);

    if (humidity < 60 ){
      Serial.print("Humidity procentage is too low!! ");
    }
     if (temperatureC < 8 ){
      Serial.print("The temperature is too low!! ");
    }
     
    //Display the graphs

        ThingSpeak.setField(1, temperatureC);
        ThingSpeak.setField(2, infrared);
        ThingSpeak.setField(3, visible_plus_ir);
        ThingSpeak.setField(4, humidity);

        int x = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);

     if(x == 200){  
            Serial.println("Channel update successful.");
        }
        else{
            Serial.println("Problem updating channel. HTTP error code " + String(x));
        }
   
    lastTime = millis();  
  }
  
}

