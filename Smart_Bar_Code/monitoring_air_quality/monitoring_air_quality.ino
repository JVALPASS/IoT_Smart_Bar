// Define Pins 
#include "DHT.h"
#include "analogWrite.h"
#include "MQUnifiedsensor.h"
#include "SPI.h"
#include "Adafruit_GFX.h"
#include "Adafruit_PCD8544.h"
#include "WiFi.h"
#include "src/dependencies/WiFiClientSecure/WiFiClientSecure.h" //using older WiFiClientSecure
#include "PubSubClient.h"
#define HOSTNAME "pub2"
#define Board ("ESP-32") // Wemos ESP-32 or other board, whatever have ESP32 core.
#define REDPIN (33) 
#define GREENPIN (22)
#define BLUEPIN (21)
#define DHTPIN (19)
#define DHTTYPE (DHT11)
#define ENABLE_MOTOR (14)
#define DIR_RIGHT (27)
#define DIR_LEFT (12)
#define Pin (36) //check the esp32-wroom-32d.jpg image on ESP32 folder 
#define Type ("MQ-2") //MQ2 or other MQ Sensor, if change this verify your a and b values.
#define Voltage_Resolution (3.3) // 3V3 <- IMPORTANT. Source: https://randomnerdtutorials.com/esp32-adc-analog-read-arduino-ide/
#define ADC_Bit_Resolution (12) // ESP-32 bit resolution. Source: https://randomnerdtutorials.com/esp32-adc-analog-read-arduino-ide/
#define RatioMQ2CleanAir (9.83) //RS / R0 = 9.83 ppm
/*setting timer */
unsigned long now = millis();

#define timeSeconds_take_data 2
unsigned long lastTrigger_take_data = 0;
boolean startTimer_take_data = false;

#define timeSeconds_mqtt_connect 5
unsigned long lastTrigger_mqtt_connect = 0;
boolean startTimer_mqtt_connect = false;

#define timeSeconds_send_data 0
unsigned long lastTrigger_send_data = 0;
boolean startTimer_send_data = false;

#define timeSeconds_wifi 0.010
unsigned long lastTrigger_wifi = 0;
boolean startTimer_wifi = false;
/*end setting timer */

/*****************************Globals***********************************************/
/* Declare LCD object for SPI
 Adafruit_PCD8544(CLK,DIN,D/C,CE,RST); */
Adafruit_PCD8544 display = Adafruit_PCD8544(18, 23, 4, 15, 2);
int contrastValue = 60; // Default Contrast Value

//Variables
const char ssid[] = "";
const char pass[] = "";
const char *MQTT_HOST = "";
const int MQTT_PORT = 8883;
const char *MQTT_USER = ""; // leave blank if no credentials used//errore qui
const char *MQTT_PASS = ""; // leave blank if no credentials used/
const char MQTT_PUB_TOPIC_TEMP[] = "home/temperature/in";
const char MQTT_PUB_TOPIC_HUM[] = "home/humidity/in";
const char MQTT_PUB_TOPIC_AIR[] = "home/CO/in";
WiFiClientSecure net;
PubSubClient clientPubSub(net);


MQUnifiedsensor MQ2(Board, Voltage_Resolution, ADC_Bit_Resolution, Pin, Type);
float CO;
float humidity;
float temperature;
DHT dht(DHTPIN, DHTTYPE);
int period;
boolean enableMotor = false;

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

void sendData(String temp, String hum, String textCO) {
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
    String pubData_temp = String(temp);
    String pubData_hum = String(hum);
    String pubData_textCO = String(textCO);
    char ch_temp[pubData_temp.length()];
    char ch_hum[pubData_hum.length()];
    char ch_textCO[pubData_textCO.length()];
    
    for (int i = 0; i < pubData_temp.length(); i++) {
      ch_temp[i] = pubData_temp.charAt(i);
    }
    for (int i = 0; i < pubData_hum.length(); i++) {
      ch_hum[i] = pubData_hum.charAt(i);
    }
    for (int i = 0; i < pubData_textCO.length(); i++) {
      ch_textCO[i] = pubData_textCO.charAt(i);
    }
    ch_temp[pubData_temp.length()] = '\0';
    ch_hum[pubData_hum.length()] = '\0';
    ch_textCO[pubData_textCO.length()] = '\0';
    clientPubSub.publish(MQTT_PUB_TOPIC_TEMP, ch_temp, false);
    clientPubSub.publish(MQTT_PUB_TOPIC_HUM, ch_hum, false);
    clientPubSub.publish(MQTT_PUB_TOPIC_AIR, ch_textCO, false);
    Serial.println("Dati inviati!");
    startTimer_send_data = true;
    lastTrigger_send_data = now;
  }
}
void receivedCallback(char* topic, byte* payload, unsigned int length) {
  String str = "";
  Serial.print("Received [");
  Serial.print(topic);
  Serial.print("]: ");
  for (int i = 0; i < length; i++) {
    str = str + (char)payload[i];
  }
  Serial.println(str);
  Serial.println("fine");
}

void setGreenLed(boolean val){
  digitalWrite(GREENPIN, val);
}
void setBlueLed(boolean val){
  digitalWrite(BLUEPIN, val);
}
void setRedLed(boolean val){
  digitalWrite(REDPIN, val);
}
void disableLed(){
  setBlueLed(LOW); 
  setGreenLed(LOW);
  setRedLed(LOW); 
}
void print_time(unsigned long time_millis){
  Serial.print("Time: ");
  Serial.print(time_millis/1000);
  Serial.println("s - ");
}
String takeTemp(){
  temperature = dht.readTemperature();
  if (isnan(humidity) || isnan(temperature)) { // is not a number
    Serial.println(F("Failed to read from DHT sensor!"));
    return "error";
  }
  //Serial.print(" Temp: ");
  //Serial.println(temperature);
  return String(temperature);
}
String takeHum(){
  humidity = dht.readHumidity();
  if (isnan(humidity) || isnan(temperature)) { // is not a number
    Serial.println(F("Failed to read from DHT sensor!"));
    return "error";
  }
  //Serial.print("Hum: ");
  //Serial.println(humidity);
  return String(humidity);
}
void disableMotor(){
  Serial.print("DISABLED ");
  analogWrite(ENABLE_MOTOR,0); //enable on
  disableLed();
  setRedLed(HIGH);
  //delay(2000);
}
void runMotor(){
  //analogWrite(ENABLE_MOTOR,180);
  //digitalWrite(DIR_RIGHT,HIGH); //one way
  //Serial.println("RIGHT");
  disableLed();
  setGreenLed(HIGH);
  analogWrite(ENABLE_MOTOR,225); //enable on
  digitalWrite(DIR_RIGHT,HIGH); //one way
}
void setup_Q2(){
  //Init the serial port communication - to debug the library
  Serial.begin(9600); //Init serial port
  delay(10);
   
  //Set math model to calculate the PPM concentration and the value of constants
  MQ2.setRegressionMethod(1); //_PPM = a*ratio^b
  MQ2.setA(36974); MQ2.setB(-3.109); // Configure the equation to to calculate H2 concentration
   
 /*
  Exponential regression:
  Gas | a | b
  H2 | 987.99 | -2.162
  LPG | 574.25 | -2.222
  CO | 36974 | -3.109
  Alcohol| 3616.1 | -2.675
  Propane| 658.71 | -2.168
  */
  MQ2.init(); 
  Serial.print("Calibrating please wait.");
  float calcR0 = 0;
  for(int i = 1; i<=10; i ++){
    MQ2.update(); // Update data, the arduino will read the voltage from the analog pin
    calcR0 += MQ2.calibrate(RatioMQ2CleanAir);
    Serial.print(".");
  }
  MQ2.setR0(calcR0/10);
  Serial.println(" done!.");
   
  if(isinf(calcR0)) {Serial.println("Warning: Conection issue, R0 is infinite (Open circuit detected) please check your wiring and supply"); while(1);}
  if(calcR0 == 0){Serial.println("Warning: Conection issue found, R0 is zero (Analog pin shorts to ground) please check your wiring and supply"); while(1);}
  delay(500);
   /***************************** MQ CAlibration ********************************************/ 
  //MQ2.serialDebug(true); uncomment if you want to print the table on the serial port
}
void setupDisplay_5110(){
  analogWrite(5,100);
  /* Initialize the Display*/
  display.begin();

  /* Change the contrast using the following API*/
  display.setContrast(contrastValue);

  /* Clear the buffer */
  display.clearDisplay();
  display.display();
  display.setTextColor(WHITE, BLACK);
}
void showDisplay(String text, int size, int x, int y){
  display.setCursor(x,y);
  display.setTextSize(size);
  display.println(text);
  display.display();
}
void setup() { 
  Serial.begin(9600);
  Serial.println("stampa topic");
  Serial.println(MQTT_PUB_TOPIC_TEMP);
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

  pinMode(REDPIN, OUTPUT); 
  pinMode(GREENPIN, OUTPUT); 
  pinMode(BLUEPIN, OUTPUT); 
  disableLed();
  Serial.print(F("Temperature and Humidity Monitoring Application\n\n"));
  dht.begin();
  pinMode(ENABLE_MOTOR,OUTPUT);
  pinMode(DIR_RIGHT,OUTPUT);
  pinMode(DIR_LEFT,OUTPUT);
  disableMotor();
  setup_Q2();

  setupDisplay_5110();
  /* Now let us display some text */
  Serial.begin(9600);
  delay(1000);
}
String takeCO(){
     MQ2.update(); // Update data, the arduino will read the voltage from the analog pin
    //MQ2.serialDebug(); // Will print the table on the serial port
    CO = MQ2.readSensor();
    String textCO = "";
    textCO = ( String(CO).equals("inf") || CO > 5000 ) ? "5000 PPM" :  ( String(CO)+ " PPM" );
    return textCO;
}
void loop() {
  now = millis();
  // Turn off the LED after the number of seconds defined in the timeSeconds variable
  if((!startTimer_take_data) || (now - lastTrigger_take_data >= (timeSeconds_take_data*1000))) {
    String temp = takeTemp();
    String hum = takeHum();
    String textCO = takeCO();
    display.clearDisplay();
    showDisplay(temp,1,1,1);
    showDisplay(hum,1,1,10);
    showDisplay(textCO,1,0,20);
    startTimer_take_data = true;
    lastTrigger_take_data = now;
    sendData(temp, hum, textCO);
  }
  if(CO > 100 || temperature > 30){
    enableMotor = true;
    runMotor();
  }
  else if(enableMotor){
    enableMotor = false;
    disableMotor();
  }
}
