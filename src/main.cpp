/* This code works with MAX30102 + 128x32 OLED i2c + Buzzer and Arduino UNO
 * It's displays the Average BPM on the screen, with an animation and a buzzer sound
 * everytime a heart pulse is detected
 * It's a modified version of the HeartRate library example
 * Refer to www.surtrtech.com for more details or SurtrTech YouTube channel
 */

#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoWebsockets.h>
//#define SERVER_IP "10.0.1.7:9080" // PC address with emulation on host
#define SERVER_IP "192.168.5.171:1880"
using namespace websockets;
unsigned long previousMillis = 0; 

const long interval = 2000;  // Interval at which to blink (milliseconds)
WebsocketsClient client;
const char* websockets_server_host = "192.168.5.171"; 

#ifndef STASSID
#define STASSID "NOVA_8558"
#define STAPSK "juice9273"
#endif

#include <Adafruit_GFX.h>        //OLED libraries
#include <Adafruit_SSD1306.h>
#include <Wire.h>
#include "MAX30105.h"           //MAX3010x library
#include "heartRate.h"          //Heart rate calculating algorithm
#include <ArduinoJson.h>
     DynamicJsonDocument doc(1024);
MAX30105 particleSensor;
///////////////////////////////////////////////////////////////////////////////////
const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred
float beatsPerMinute;
int beatAvg;
long irValue = 0;


double oxi;


#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define OLED_RESET -1    // Reset pin # (or -1 if sharing Arduino reset pin)

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET); // Declaring the display name (display)

#define USEFIFO

static const unsigned char PROGMEM logo2_bmp[] =
{ 0x03, 0xC0, 0xF0, 0x06, 0x71, 0x8C, 0x0C, 0x1B, 0x06, 0x18, 0x0E, 0x02, 0x10, 0x0C, 0x03, 0x10,              //Logo2 and Logo3 are two bmp pictures that display on the OLED if called
0x04, 0x01, 0x10, 0x04, 0x01, 0x10, 0x40, 0x01, 0x10, 0x40, 0x01, 0x10, 0xC0, 0x03, 0x08, 0x88,
0x02, 0x08, 0xB8, 0x04, 0xFF, 0x37, 0x08, 0x01, 0x30, 0x18, 0x01, 0x90, 0x30, 0x00, 0xC0, 0x60,
0x00, 0x60, 0xC0, 0x00, 0x31, 0x80, 0x00, 0x1B, 0x00, 0x00, 0x0E, 0x00, 0x00, 0x04, 0x00,  };

static const unsigned char PROGMEM logo3_bmp[] =
    {0x01, 0xF0, 0x0F, 0x80, 0x06, 0x1C, 0x38, 0x60, 0x18, 0x06, 0x60, 0x18, 0x10, 0x01, 0x80, 0x08,
     0x20, 0x01, 0x80, 0x04, 0x40, 0x00, 0x00, 0x02, 0x40, 0x00, 0x00, 0x02, 0xC0, 0x00, 0x08, 0x03,
     0x80, 0x00, 0x08, 0x01, 0x80, 0x00, 0x18, 0x01, 0x80, 0x00, 0x1C, 0x01, 0x80, 0x00, 0x14, 0x00,
     0x80, 0x00, 0x14, 0x00, 0x80, 0x00, 0x14, 0x00, 0x40, 0x10, 0x12, 0x00, 0x40, 0x10, 0x12, 0x00,
     0x7E, 0x1F, 0x23, 0xFE, 0x03, 0x31, 0xA0, 0x04, 0x01, 0xA0, 0xA0, 0x0C, 0x00, 0xA0, 0xA0, 0x08,
     0x00, 0x60, 0xE0, 0x10, 0x00, 0x20, 0x60, 0x20, 0x06, 0x00, 0x40, 0x60, 0x03, 0x00, 0x40, 0xC0,
     0x01, 0x80, 0x01, 0x80, 0x00, 0xC0, 0x03, 0x00, 0x00, 0x60, 0x06, 0x00, 0x00, 0x30, 0x0C, 0x00,
     0x00, 0x08, 0x10, 0x00, 0x00, 0x06, 0x60, 0x00, 0x00, 0x03, 0xC0, 0x00, 0x00, 0x01, 0x80, 0x00};



void onMessageCallback(WebsocketsMessage message) {
    Serial.print("Got Message: ");
    Serial.println(message.data());
}

void onEventsCallback(WebsocketsEvent event, String data) {
    if(event == WebsocketsEvent::ConnectionOpened) {
        Serial.println("Connnection Opened");
    } else if(event == WebsocketsEvent::ConnectionClosed) {
        Serial.println("Connnection Closed");
    } else if(event == WebsocketsEvent::GotPing) {
        Serial.println("Got a Ping!");
    } else if(event == WebsocketsEvent::GotPong) {
        Serial.println("Got a Pong!");
    }
}

void setup()
{

  Serial.begin(115200);
  WiFi.begin(STASSID, STAPSK);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }


  Serial.println("");
  Serial.print("Connected! IP address: ");
  Serial.println(WiFi.localIP());


  
    // run callback when messages are received
    client.onMessage(onMessageCallback);
    
    // run callback when events are occuring
    client.onEvent(onEventsCallback);

    // Connect to server
    bool connected = client.connect(websockets_server_host, 1880, "/ws/simple2");
    if(connected) {
        Serial.println("Connecetd!");
        client.send("Hello Server");
    } else {
        Serial.println("Not Connected!");
    }
    // Send a ping
    client.ping();


  Serial.println("Initializing...");

 display.begin(SSD1306_SWITCHCAPVCC, 0x3C); //Start the OLED display
  display.display();

 
  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30102 was not found. Please check wiring/power/solder jumper at MH-ET LIVE MAX30102 board. ");
    while (1);
  }
 
  //Setup to sense a nice looking saw tooth on the plotter
  //byte ledBrightness = 0x7F; //Options: 0=Off to 255=50mA
  byte ledBrightness = 70; //Options: 0=Off to 255=50mA
  byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  //Options: 1 = IR only, 2 = Red + IR on MH-ET LIVE MAX30102 board
  int sampleRate = 200; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 4096; //Options: 2048, 4096, 8192, 16384
  // Set up the wanted parameters
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings

  display.clearDisplay();

  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 10);
  // Display static text
  display.println("TeraCare");
  display.display();
  delay(2000);

}

double avered = 0; double aveir = 0;
double sumirrms = 0;
double sumredrms = 0;
int i = 0;
int Num = 100;//calculate SpO2 by this sampling interval
 
double ESpO2 = 93.0;//initial value of estimated SpO2
double FSpO2 = 0.7; //filter factor for estimated SpO2
double frate = 0.95; //low pass filter for IR/red LED value to eliminate AC component
#define TIMETOBOOT 2000 // wait for this time(msec) to output SpO2
#define SCALE 88.0 //adjust to display heart beat and SpO2 in the same scale
#define SAMPLING 5 //if you want to see heart beat more precisely , set SAMPLING to 1
#define FINGER_ON 35000 // if red signal is lower than this , it indicates your finger is not on the sensor
#define MINIMUM_SPO2 0.0

void loop()
{
unsigned long currentMillis = millis();
  uint32_t ir, red , green;
  double fred, fir;
  double SpO2 = 0; //raw SpO2 before low pass filtered

 
#ifdef USEFIFO
  particleSensor.check(); //Check the sensor, read up to 3 samples
 
  while (particleSensor.available()) {//do we have new data
#ifdef MAX30105
   red = particleSensor.getFIFORed(); //Sparkfun's MAX30105
    ir = particleSensor.getFIFOIR();  //Sparkfun's MAX30105
#else
    red = particleSensor.getFIFOIR(); //why getFOFOIR output Red data by MAX30102 on MH-ET LIVE breakout board
    ir = particleSensor.getFIFORed(); //why getFIFORed output IR data by MAX30102 on MH-ET LIVE breakout board
#endif
    i++;
    fred = (double)red;
    fir = (double)ir;
    avered = avered * frate + (double)red * (1.0 - frate);//average red level by low pass filter
    aveir = aveir * frate + (double)ir * (1.0 - frate); //average IR level by low pass filter
    sumredrms += (fred - avered) * (fred - avered); //square sum of alternate component of red level
    sumirrms += (fir - aveir) * (fir - aveir);//square sum of alternate component of IR level    
    if ((i % SAMPLING) == 0) {//slow down graph plotting speed for arduino Serial plotter by thin out
      if ( millis() > TIMETOBOOT) {
        if (ir < FINGER_ON) ESpO2 = MINIMUM_SPO2; //indicator for finger detached
        if (ESpO2 <= -1)
        {
          ESpO2 = 0;
        }

        if (ESpO2 > 100)
        {
          ESpO2 = 100;
        }

        oxi = ESpO2;
        
        //Serial.print(" Oxygen % = ");
        //Serial.println(ESpO2); //low pass filtered SpO2

      }
    }
    if ((i % Num) == 0) {
      double R = (sqrt(sumredrms) / avered) / (sqrt(sumirrms) / aveir);
      // Serial.println(R);
      SpO2 = -23.3 * (R - 0.4) + 100; //http://ww1.microchip.com/downloads/jp/AppNotes/00001525B_JP.pdf
      ESpO2 = FSpO2 * ESpO2 + (1.0 - FSpO2) * SpO2;//low pass filter
      //  Serial.print(SpO2);Serial.print(",");Serial.println(ESpO2);
      //Serial.print(" Oxygen2 % = ");
      //Serial.println(ESpO2); //low pass filtered SpO2
      sumredrms = 0; sumirrms = 0; i = 0;
      break;
    }
    particleSensor.nextSample(); //We're finished with this sample so move to next sample
    //Serial.println(SpO2);

    
 long irValue = particleSensor.getIR();
  
  if (irValue > 7000)
  {                                                     // If a finger is detected
    display.clearDisplay();                             // Clear the display
    display.drawBitmap(5, 5, logo2_bmp, 24, 21, WHITE); // Draw the first bmp picture (little heart)
    display.setTextSize(1);                             // Near it display the average BPM you can display the BPM if you want
    display.setTextColor(WHITE);
    display.setCursor(40, 0);
    display.println("BPM");
       display.setTextSize(2);
    display.setCursor(100, 0);
    display.println(beatAvg);
  

     display.setTextSize(1);                                   //Near it display the average BPM you can display the BPM if you want
      display.setTextColor(WHITE);   
       display.setCursor(40, 18);   
      display.println("SpO2");
      display.setTextSize(2);
    display.setCursor(66, 18);                
      display.println(oxi,1); 
      display.setTextSize(1);
        display.setCursor(120, 18);
      display.print("%"); 
      display.display();

    if (checkForBeat(irValue) == true) // If a heart beat is detected
    {
         display.clearDisplay();                             // Clear the display
    display.drawBitmap(5, 5, logo2_bmp, 24, 21, WHITE); // Draw the first bmp picture (little heart)
    display.setTextSize(1);                             // Near it display the average BPM you can display the BPM if you want
    display.setTextColor(WHITE);
    display.setCursor(40, 0);
    display.println("BPM");
       display.setTextSize(2);
    display.setCursor(100, 0);
    display.println(beatAvg);


     display.setTextSize(1);                                   //Near it display the average BPM you can display the BPM if you want
      display.setTextColor(WHITE);   
       display.setCursor(40, 18);   
      display.println("SpO2");
      display.setTextSize(2);
    display.setCursor(66, 18);                
      display.println(oxi,1); 
      display.setTextSize(1);
        display.setCursor(120, 18);
      display.print("%"); 
      display.display();
      tone(3, 1000); // And tone the buzzer for a 100ms you can reduce it it will be better
      delay(100);
      noTone(3); // Deactivate the buzzer to have the effect of a "bip"
      // We sensed a beat!
      long delta = millis() - lastBeat; // Measure duration between two beats
      lastBeat = millis();

      beatsPerMinute = 60 / (delta / 1000.0); // Calculating the BPM

      if (beatsPerMinute < 255 && beatsPerMinute > 20) // To calculate the average we strore some values (4) then do some math to calculate the average
      {
        rates[rateSpot++] = (byte)beatsPerMinute; // Store this reading in the array
        rateSpot %= RATE_SIZE;                    // Wrap variable

        // Take average of readings
        beatAvg = 0;
        for (byte x = 0; x < RATE_SIZE; x++)
          beatAvg += rates[x];
        beatAvg /= RATE_SIZE;
      }
    }
       Serial.print("IR=");
    Serial.print(irValue);
    Serial.print(", BPM=");
    Serial.print(beatsPerMinute);
    Serial.print(", Avg BPM=");
    Serial.print(beatAvg);
    Serial.print(",SPO2= ");
    Serial.print(oxi,1);
    Serial.println();
 
  }

  if (irValue < 7000)
  { // If no finger is detected it inform the user and put the average BPM to 0 or it will be stored for the next measure
    beatAvg = 0;
    beatsPerMinute = 0;
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(30, 5);
    display.println("Please Place ");
    display.setCursor(30, 15);
    display.println("your finger ");
    display.display();
    noTone(3);
  }
  #endif

  if (beatAvg > 0)
  {
      if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;
         doc["beatAvg"] = beatAvg;
          doc["SpO2"] = oxi;
          String json;
          serializeJson(doc, json);
          client.send(json);
          doc.clear();
    }
  }
}}