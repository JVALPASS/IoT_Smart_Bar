#include <ESP32Servo.h>
#include "WiFi.h"
#include "src/dependencies/WiFiClientSecure/WiFiClientSecure.h" //using older WiFiClientSecure
#include "PubSubClient.h"
#define HOSTNAME "pub1"
unsigned long now = millis();

#define timeSeconds_mqtt_connect 5
unsigned long lastTrigger_mqtt_connect = 0;
boolean startTimer_mqtt_connect = false;

#define timeSeconds_send_data 0.5
unsigned long lastTrigger_send_data = 0;
boolean startTimer_send_data = false;

#define timeSeconds_wifi 0.010
unsigned long lastTrigger_wifi = 0;
boolean startTimer_wifi = false;

//Variables
boolean sub = false;
const char ssid[] = "";
const char pass[] = "";
const char *MQTT_HOST = "";
const int MQTT_PORT = 8883;
const char *MQTT_USER = ""; // leave blank if no credentials used//errore qui
const char *MQTT_PASS = ""; // leave blank if no credentials used/
const char MQTT_PUB_TOPIC[] = "home/bathroom/in";
WiFiClientSecure net;
PubSubClient clientPubSub(net);

Servo myservo;  // create servo object to control a servo
// 16 servo objects can be created on the ESP32

int pos = 180;    // variable to store the servo position
// Recommended PWM GPIO pins on the ESP32 include 2,4,12-19,21-23,25-27,32-33 
const int servoPin = 12;
const int led_pin = 26;
const int sensor_pir_pin = 27;
const int photoResistor_pin = 33;
int photoRvalue;
#define time_seconds_stop_motion 3
boolean startTimer_DetectionMotion = false;
unsigned long stop_motion = millis();
unsigned long last_trigger_photoresistor = 0;

#define time_seconds_motion 5
unsigned long now_motion = millis();
unsigned long last_trigger_motion = 0;
boolean startTimer_motion = false;

#define timeSeconds_door 0.015
unsigned long lastTrigger_door = 0;
boolean startTimer_door = false;
unsigned long now_door = millis();

bool openDoor = true;
boolean statusDoorChanged = false;

const char* local_root_ca =
  "-----BEGIN CERTIFICATE-----\n" \
  "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\n" \
  "yzLg1Q05GHbhRhVlqWUCSuuBJRa9o1e3YA==\n" \
  "-----END CERTIFICATE-----";

void mqtt_connect(){
    startTimer_mqtt_connect = false;
    while (!clientPubSub.connected()) {
      now = millis();
      if((!startTimer_mqtt_connect) || ((now - lastTrigger_mqtt_connect) >= timeSeconds_mqtt_connect*1000)){
        Serial.print("MQTT connecting....");
        if (clientPubSub.connect(HOSTNAME, MQTT_USER, MQTT_PASS)) {
          Serial.println("connected");
          startTimer_mqtt_connect = false;
        } else {
          Serial.print("failed, status code =");
          Serial.print(clientPubSub.state());
          Serial.println("try again in 5 seconds");
          /* Wait 5 seconds before retrying */
          //delay(5000);
          startTimer_mqtt_connect = true;
          lastTrigger_mqtt_connect = now;
        }
      }
    }
}
void sendData(String pubData) {
  if (WiFi.status() != WL_CONNECTED){
    Serial.print("Checking wifi");
    startTimer_wifi = false;
    while (WiFi.waitForConnectResult() != WL_CONNECTED){
      now = millis();
      if((!startTimer_wifi) || (now - lastTrigger_wifi >= (timeSeconds_wifi*1000))) {
        WiFi.begin(ssid, pass);
        Serial.print(".");
        startTimer_wifi = true;
        lastTrigger_wifi = now;
        //delay(10);
      }
    }
    Serial.println("connected");
  }
  else{
    if (!clientPubSub.connected()){
      net.setCACert(local_root_ca);
      Serial.println("setting cert");
      clientPubSub.setServer(MQTT_HOST, MQTT_PORT);
      Serial.println("setting server");
      mqtt_connect();
    }
    else{
      clientPubSub.loop();
    }
  }
  now = millis();
  if((!startTimer_send_data) || (now - lastTrigger_send_data >= (timeSeconds_send_data*1000))) {
    Serial.println("published ");
    char ch[pubData.length()];
    for (int i = 0; i < pubData.length(); i++) {
      ch[i] = pubData.charAt(i);
    }
    ch[pubData.length()] = '\0';
    clientPubSub.publish(MQTT_PUB_TOPIC, ch, false);
    Serial.println("Dati inviati!");
    startTimer_send_data = true;
    lastTrigger_send_data = now;
  }
}

void open_door(){
  Serial.println("open door");
  while(pos < 180) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    now_door = millis();
    if(now_door - lastTrigger_door >= (timeSeconds_door*1000)) {
      //Serial.println(pos);
      //delay(15); 
      pos++;
      myservo.write(pos); 
      lastTrigger_door = now_door;
    }
  }
}
void close_door(){
  Serial.println("close door");
  while(pos > 90) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    now_door = millis();
    if(now_door - lastTrigger_door >= (timeSeconds_door*1000)) {
      //Serial.println(pos);
      //delay(15); 
      pos--;
      myservo.write(pos); 
      lastTrigger_door = now_door;
    }
  }
}
void setupServoMotor(){
  // Allow allocation of all timers
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  myservo.setPeriodHertz(50);    // standard 50 hz servo
  myservo.attach(servoPin, 500, 2800); // attaches the servo on pin 18 to the servo object
  // using default min/max of 1000us and 2000us
  // different servos may require different min/max settings
  // for an accurate 0 to 180 sweep
  // Serial port for debugging purposes
  myservo.write(180); 
}
void movement_detection() {
  stop_motion = millis();
  if((!startTimer_DetectionMotion) || (stop_motion - last_trigger_photoresistor >= time_seconds_stop_motion * 1000)){
    startTimer_DetectionMotion = false;
    if(digitalRead(sensor_pir_pin)){
      Serial.println("movmenet detected");
      if(openDoor == true){
        openDoor = false;
        digitalWrite(led_pin, HIGH);
        close_door();
      }
      startTimer_motion = true;
      last_trigger_motion = millis();
    }
  }
}
void stop_detection() {
  now_motion = millis();
  if((startTimer_motion && (now_motion - last_trigger_motion >= time_seconds_motion * 1000))) {
      Serial.println("Motion has stopped");
      if(openDoor == false){
        openDoor = true;
        digitalWrite(led_pin, LOW);
        open_door();
        startTimer_motion = false;
        last_trigger_motion = now_motion;
      }
  }
}
void detectPhotoResistor(){
  photoRvalue = analogRead(photoResistor_pin);
  if(photoRvalue < 1500){
    if(!openDoor){
      Serial.println("the door is closed");
      Serial.println("open the door");
      digitalWrite(led_pin, LOW);
      open_door();
      openDoor = true;
      last_trigger_photoresistor = millis();
      startTimer_DetectionMotion = true;
      startTimer_motion = false;
     }
     else{
      Serial.println("the door is opened");
      Serial.println("close the door");
      digitalWrite(led_pin, HIGH);
      close_door();
      openDoor = false;
    }
  }
}
void setup() {
  Serial.begin(115200);
  Serial.println("stampa topic");
  Serial.println(MQTT_PUB_TOPIC);
  Serial.print("Attempting to connect to SSID: ");
  Serial.println(ssid);
  WiFi.setHostname(HOSTNAME);
  WiFi.mode(WIFI_AP_STA);
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(1000);
  }
  Serial.println();
  Serial.print("Connected to ");
  Serial.println(ssid);
  
  setupServoMotor();
  pinMode(sensor_pir_pin, INPUT_PULLUP);
  pinMode(led_pin, OUTPUT);
  pinMode(photoResistor_pin, INPUT);
  digitalWrite(led_pin, LOW);
  myservo.write(pos); 
  delay(4000);
  Serial.println("start");
}

void loop() {
  movement_detection();
  stop_detection();
  detectPhotoResistor();
  sendData(String(openDoor));
}
