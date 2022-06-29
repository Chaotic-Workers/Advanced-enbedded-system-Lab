#include <ArduinoMqttClient.h>
#include <WiFiNINA.h>
#include "arduino_secrets.h"
#include "DHT.h"

#define Echo_EingangsPin 7 // Echo input pin
#define Trigger_AusgangsPin 8 // Trigger output pin
#define DHTPIN 2     
 
// The temp and hum sensor is initialized
#define DHTTYPE DHT11 // DHT 11
DHT dht(DHTPIN, DHTTYPE);
///////My sensitive data of wifi is in the arduino_secrets.h
char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)

WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

const char broker[] = "192.168.213.65";
int        port     = 1883;
const char topic[]  = "project";
const char topic2[]  = "project1";
const char topic3[]  = "project2";
const char topic4[]  = "project3";


//set interval for sending messages (milliseconds)
const long interval = 8000;
unsigned long previousMillis = 0;

int count = 0;
int Analog_Input = A0; // Analog output of the Microphone sensor
int Digital_Input = 3; // Digital output of the Microphone sensor
// Required variables are defined
int maximumRange = 300; //Ultrasound sensor variables
int minimumRange = 2;
long distance;
long duration;

//Led variables
int Led_Red = 11;
int Led_Green = 10;
int Led_Blue = 12;
void setup() {
  //Initialize serial and wait for port to open:
  Serial.begin(9600);

  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // attempt to connect to Wifi network:
  Serial.print("Attempting to connect to WPA SSID: ");
  Serial.println(ssid);
  while (WiFi.begin(ssid, pass) != WL_CONNECTED) {
    // failed, retry
    Serial.print(".");
    delay(5000);
  }

  Serial.println("You're connected to the network");
  Serial.println();

  Serial.print("Attempting to connect to the MQTT broker: ");
  Serial.println(broker);

  if (!mqttClient.connect(broker, port)) {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());

    while (1);
  }

  Serial.println("You're connected to the MQTT broker!");
  Serial.println();
  //microphone setup
 pinMode (Analog_Input, INPUT);
  pinMode (Digital_Input, INPUT);
//Ultra sound setup
pinMode (Trigger_AusgangsPin, OUTPUT);
 pinMode (Echo_EingangsPin, INPUT);
 //Temp and humidity setup
 Serial.println("KY-015 test - temperature and humidity test:");
 
    // Initialize output pins for the LEDs
  pinMode (Led_Red, OUTPUT); 
  pinMode (Led_Green, OUTPUT);
  pinMode (Led_Blue, OUTPUT); 
   // Measurement is started

  dht.begin();
}

void loop() {
  // call poll() regularly to allow the library to send MQTT keep alive which
  // avoids being disconnected by the broker
  mqttClient.poll();

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    // save the last time a message was sent
    previousMillis = currentMillis;

    //record random value from A0, A1 and A2
    float Rvalue = Microphone();
    float Rvalue3 = Distances();
    float Rvalue2T= temper();
    float Rvalue2 = humidity();

    Serial.print("Sending message to topic: ");
    Serial.println(topic);
    
    Serial.println(Rvalue);

    Serial.print("Sending message to topic: ");
    Serial.println(topic2);
    Serial.println(Rvalue3);

    Serial.print("Sending message to topic: ");
    Serial.println(topic3);
    Serial.println(Rvalue2);

    // send message, the Print interface can be used to set the message contents
    mqttClient.beginMessage(topic);
   mqttClient.print("-------------------------The value comming from Microphone----------------------------");
   
   mqttClient.print(Rvalue);
   
    mqttClient.endMessage();

    mqttClient.beginMessage(topic2);
       mqttClient.print("---------Value of Distance from near by Object in cm------------");

       mqttClient.print(Rvalue3);

    mqttClient.endMessage();

    mqttClient.beginMessage(topic3);
    mqttClient.print("------------------Humidity value in % ---------------");
    mqttClient.print(Rvalue2);
    mqttClient.print("-----------Temp value in celcius-------------");
        mqttClient.print(Rvalue2T);

    mqttClient.endMessage();


if (temper()||humidity >= 23){
    digitalWrite (Led_Red, HIGH); // LED is switched on
      digitalWrite (Led_Blue, HIGH); // LED is switched on
    Serial.print("Dispensing perfume with Lower voltality");
mqttClient.beginMessage(topic4);
       mqttClient.print("---------Dispensing perfume with Lower voltality because low humidity and temperature------------");

       

    mqttClient.endMessage();

  }
if (Distances() <= 100){
   digitalWrite (Led_Red, LOW); // LED is switched on
      digitalWrite (Led_Blue, HIGH); // LED is switched on
      digitalWrite (Led_Green, HIGH); // LED is switched on
    Serial.print("Dispensing perfume with Lower voltality");
mqttClient.beginMessage(topic4);
       mqttClient.print("---------Dispensing perfume with Lower voltality because of smaller room------------");

       

    mqttClient.endMessage();
  }
  else if(Distances >= 101){
     digitalWrite (Led_Red, HIGH); // LED is switched on
      digitalWrite (Led_Blue, LOW); // LED is switched on
      digitalWrite (Led_Green, HIGH); // LED is switched on
    Serial.print("Dispensing perfume with HIgher voltality");
mqttClient.beginMessage(topic4);
       mqttClient.print("---------Dispensing perfume with HIGHER voltality because of Larger room------------");
       mqttClient.endMessage();

    };





    Serial.println();
  }
}

//New Functions for sensors
float Microphone (){
  float  Analogous;
  int Digital;
    
  //Current values are read out, converted to the voltage value...
  Analogous =  analogRead (Analog_Input)   *  (5.0 / 1023.0); 
  Digital = digitalRead (Digital_Input) ;
    
  //...  and issued at this point
    Serial.println  ( " ----------------------------Microphone value------------------------------------") ;
Serial.print  ("Analog voltage value:");  Serial.print (Analogous,  4) ;   Serial.print  ("V, ");
  Serial.print ("Limit value:") ;
  mqttClient.print(" ----------------------------Microphone value------------------------------------");
  mqttClient.print("Analog voltage value:");
  if  (Digital==1) 
  {
      Serial.println ("reached");
  }
  else
  {
      Serial.println (" not yet reached");
  }
  Serial.println  ( " ----------------------------------------------------------------") ;
  delay (200) ;
  return Analogous;
  };
float temper(){
  // temperature is measured
  float t = dht.readTemperature();
  return t;
  };
  float humidity(){
 delay(2000);
 
  // Humidity is measured
  float h = dht.readHumidity();
  // temperature is measured
  float t = dht.readTemperature();
   
  // Checking if the measurements have passed without errors
  // if an error is detected, a error message is displayed here
  if (isnan(h) || isnan(t)) {
    Serial.println("Error reading the sensor");
    return;
  }
 
  // Output to serial console
  Serial.println("-----------------------------------------------------------");
  Serial.print("Humidity: ");
  Serial.print(h);
  Serial.print(" %\t");
  Serial.print("Temperature: ");
  Serial.print(t);
  Serial.print(char(186)); //Output <°> symbol
  Serial.println("C ");
  Serial.println("-----------------------------------------------------------");
  Serial.println(" ");
    return h;
    };


    float Distances(){
      
 // Distance measurement is started by means of the 10us long trigger signal
 digitalWrite (Trigger_AusgangsPin, HIGH);
 delayMicroseconds (10);
 digitalWrite (Trigger_AusgangsPin, LOW);
  
 // Now we wait at the echo input until the signal has been activated
 // and then the time measured how long it remains activated
 duration = pulseIn (Echo_EingangsPin, HIGH);
  
 // Now the distance is calculated using the recorded time
 distance = duration / 58.2;
  
 // Check whether the measured value is within the permissible distance
 if (distance >= maximumRange || distance <= minimumRange)
 {
    // If not, an error message is output.
    
      Serial.println ("Distance outside the measuring range");
      Serial.println ("-----------------------------------");
 }
  
 else
 {
    // The calculated distance is output in the serial output
      Serial.println  ( " --------------------------------Distance of object value--------------------------------") ;

      Serial.print ("The distance is:");
      Serial.print (distance);
      Serial.println ("cm");
      Serial.println ("-----------------------------------");
 }
  // Pause between the individual measurements
 delay (500);
     return distance;
      };
