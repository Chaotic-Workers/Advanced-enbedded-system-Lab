// Adafruit_DHT library is inserted
#include "DHT.h"
 
// Here the respective input pin can be declared
#define DHTPIN 2     
 
 
 
 //NOTE: LED is represented as a motor and dispenser. If red light is turned on then high intensity scent is dispersed. purple represents intensity depending on the humidity. Green represents that user has
 //initialised the dispersion from the user 



 
// The sensor is initialized
#define DHTTYPE DHT11 // DHT 11
DHT dht(DHTPIN, DHTTYPE);
//(Part of publisher and subscriber)
#include <ArduinoMqttClient.h>
#include <WiFiNINA.h>

char ssid[] = "Hammad";        // my network SSID (name)
char pass[] = "123asd123";    // my network password (use for WPA, or use as key for WEP)

WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

const char broker[] = "192.168.157.65";
int        port     = 1883;
const char topic[]  = "project";
const char topic2[]  = "project1";
const char topic3[]  = "project2";

//set interval for sending messages (milliseconds)
const long interval = 8000;
unsigned long previousMillis = 0;

int count = 0;
// Declaration and initialization of input pins
int Analog_Input = A0; // Analog output of the sensor
int Digital_Input = 3; // Digital output of the sensor
  
void setup  ( )
{
    Serial.begin (9600) ;  //  Serial output with 9600 bps

  pinMode (Analog_Input, INPUT);
  pinMode (Digital_Input, INPUT);
        Serial.println("KY-015 Temp and hum values:");
 
  // Measurement is started
  dht.begin();
  pinMode (Trigger_outputpin, OUTPUT);
 pinMode (Echo_inputpin, INPUT);
 Serial.begin (9600);

//setting up MQTT response
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

  while (!mqttClient.connect(broker, port)) {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());

  }

  Serial.println("You're connected to the MQTT broker!");
  Serial.println();
}
  
//  The program reads the current values of the input pins
// and outputs it on the serial output
void loop  ( )
{
  if (Microphone()==1){
   humidity();
   if(Microphone()||Distance()==1){
    digitalWrite (Led_Red, HIGH);
    digitalWrite (Led_Blue, LOW);
    } 
    else {
      digitalWrite (Led_Red, LOW); 
    digitalWrite (Led_Blue, HIGH);  
    };
    }
    else
      Serial.print("noise not found\n");
      }



 // call poll() regularly to allow the library to send MQTT keep alive which
  // avoids being disconnected by the broker
  mqttClient.poll();

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    // save the last time a message was sent
    previousMillis = currentMillis;

    //record random value from A0, A1 and A2
    int Rvalue = Microphone();
    int Rvalue2 = Distance();
    int Rvalue3 = humidity();

    Serial.print("Sending message to topic: ");
    Serial.println(topic);
    Serial.println(Rvalue);

    Serial.print("Sending message to topic: ");
    Serial.println(topic2);
    Serial.println(Rvalue2);

    Serial.print("Sending message to topic: ");
    Serial.println(topic2);
    Serial.println(Rvalue3);

    // send message, the Print interface can be used to set the message contents
    mqttClient.beginMessage(topic);
    mqttClient.print(Rvalue);
    mqttClient.endMessage();

    mqttClient.beginMessage(topic2);
    mqttClient.print(Rvalue2);
    mqttClient.endMessage();

    mqttClient.beginMessage(topic3);
    mqttClient.print(Rvalue3);
    mqttClient.endMessage();

    Serial.println();
  }
}

//New Functions for sensors
int Microphone (){
  float  Analog;
  int Digital;
    
  //Current values are read out, converted to the voltage value...
  Analog =  analogRead (Analog_Input)   *  (5.0 / 1023.0); 
  Digital = digitalRead (Digital_Input) ;
    
  //...  and issued at this point
  //Serial.print  ("Analog voltage value:");  Serial.print (Analog,  4) ;   Serial.print  ("V, ");
  //Serial.print ("Limit value:") ;
  
  if  (Digital==1) 
  {
return 1;  }
  else
  {
return 0;  }
  Serial.println  ( " ----------------------------------------------------------------") ;
  delay (2000) ;
  };

  int humidity(){
    
  // Two seconds pause between measurements
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
    
    };


    int Distance(){
      
 // Distance measurement is started by means of the 10us long trigger signal
 digitalWrite (Trigger_outputpin, HIGH);
 delayMicroseconds (10);
 digitalWrite (Trigger_outputpin, LOW);
  
 // Now we wait at the echo input until the signal has been activated
 // and then the time measured how long it remains activated
 duration = pulseIn (Echo_inputpin, HIGH);
  
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
      Serial.print ("The distance is:");
      Serial.print (distance);
      Serial.println ("cm");
      Serial.println ("-----------------------------------");
 }
  // Pause between the individual measurements
 delay (500);
      };
