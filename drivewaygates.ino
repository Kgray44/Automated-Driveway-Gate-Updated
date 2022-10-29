#define BLYNK_TEMPLATE_ID "************"
#define BLYNK_DEVICE_NAME "********"
#define BLYNK_AUTH_TOKEN "*******************************"
#define BLYNK_PRINT Serial

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <SoftwareSerial.h>

#define motorAspeed   16//D11
#define motorBspeed   17//D10
#define motorAdir     14//D6
#define motorBdir     13//D7
#define hall          26//D3
#define ultra1        22
#define ultra2        21
#define currentIn     A0
#define pinRed        D4
#define pinGreen      D3
#define pinBlue       4//D12
#define magnet        D9//D13
#define light         4//D12

/* Current Sensor (currently not working)
const int numReadings = 30;
float readings[numReadings];      // the readings from the analog input
int inde = 0;                  // the inde of the current reading
float total = 0;                  // the running total
float average = 0;                // the average
float currentValue = 0;
*/
int ppi = 300; //pulses per inch (for the PA-04 Linear Actuator).  Change this according to your specs.
int usedAct = 6; //inches of actuator used

char ssid[] = "************";//WiFi network name
char pass[] = "********";//WiFi network password

//Gate button value for Blynk
int button;

int timer;

//current sensor value
float voltage;

//ultrasonic sensor
unsigned char data[4] = {};
float distance;
float inches;
int car;
int carDist = 24;//alter this value! This value is distance in inches before something is activated

int tim = 10000;

long hallCount;

//gate status flag
boolean flag = false;

//LED strip starting values
int red = 255;
int green = 255;
int blue = 255;

SoftwareSerial ultraSerial;

//BLYNK_CONNECTED(){
//  Blynk.syncAll();
//}

BLYNK_WRITE(V0){
  button = param.asInt();
}

BLYNK_WRITE(V1){
  red = param.asInt();
}

BLYNK_WRITE(V2){
  green = param.asInt();
}

BLYNK_WRITE(V3){
  blue = param.asInt();
}

BLYNK_WRITE(V4){
  int spotlight = param.asInt();
  if (spotlight){
    digitalWrite(light, HIGH);
  }
  else {
    digitalWrite(light, LOW);
  }
}

String string;
BLYNK_WRITE(V10) {
  string = param.asStr();
  if (string == "restart"){
    Blynk.virtualWrite(V10, "Restarting...");
    Serial.println("Restarting...");
    delay(2000);
    ESP.restart();
  }
}

//gate status flag
boolean isOpen = false;

//PWM information
const int freq = 5000;
const int resolution = 8;
const int redChannel = 1;
const int greenChannel = 2;
const int blueChannel = 3;
const int motorA = 4;
const int motorB = 5;

void setup() {
  Serial.begin(115200);
  ultraSerial.begin(9600, SWSERIAL_8N1, ultra1, ultra2, false); // RX, TX

  if (!ultraSerial) { // If the object did not initialize, then its configuration is invalid
    Serial.println("Invalid SoftwareSerial pin configuration, check config"); 
  } 
  
  //set pinModes
  pinMode(motorAspeed, OUTPUT);
  pinMode(motorBspeed, OUTPUT);
  pinMode(motorAdir, OUTPUT);
  pinMode(motorBdir, OUTPUT);
  pinMode(pinRed, OUTPUT);
  pinMode(pinGreen, OUTPUT);
  pinMode(pinBlue, OUTPUT);
  pinMode(hall, INPUT_PULLUP);
  pinMode(currentIn, INPUT);
  pinMode(magnet, OUTPUT);
  pinMode(light, OUTPUT);
  
  //attach PWM channels to the correct pins
  ledcAttachPin(pinRed, redChannel);
  ledcAttachPin(pinGreen, greenChannel);
  ledcAttachPin(pinBlue, blueChannel);
  ledcAttachPin(motorAspeed, motorA);
  ledcAttachPin(motorBspeed, motorB);
  
  //setup the PWM channels
  ledcSetup(redChannel, freq, resolution);
  ledcSetup(greenChannel, freq, resolution);
  ledcSetup(blueChannel, freq, resolution);
  ledcSetup(motorA, freq, resolution);
  ledcSetup(motorB, freq, resolution);  
  
  //connect to the WiFi
  wificonnect();

  //Start Blynk
  Serial.println("Blynk Starting...");
  Blynk.config(BLYNK_AUTH_TOKEN);
  
  //Start OTA
  Serial.println("OTA Starting...");
  OTAStart();

  //open the gates all the way at the beginning to set the hall sensor back at 0
  Serial.println("Begin opening...");
  delay(1000);
  beginOpen();
  hallCount = ppi*usedAct;
  isOpen = true;
  Serial.println("Attaching interrupt...");
  attachInterrupt(digitalPinToInterrupt(hall), interruptName, FALLING);//Interrupt initialization
  //then close to the correct place, keeping track of the hall count
  Serial.println("Closing to correct place...");
  close();
  Serial.println("Done closing!");
  Serial.println("Ready");
  isOpen = false;
}

void loop() {
  /*current sensor reading (currently not working)
  currentVal();
  if (currentValue >= 20.00){
    ledcWrite(motorA, 0);
    ledcWrite(motorB, 0);
    digitalWrite(hall, HIGH);
    Serial.println("Current too high!");
  }
  current = (analogRead(currentIn)-510)*5/1024/0.04-0.04;//calculate current value
  Blynk.virtualWrite(V7,current);
  */
  
  //if WiFi was lost, reconnect
  if (WiFi.status() == WL_CONNECTION_LOST){
    Serial.println("Connection lost...");
    wificonnect();
  }
  
  //handle the OTA
  ArduinoOTA.handle();
  
  //ultrasonic sensor reading
  for (int i=0;i<4;i++){
    Serial.println("Checkdist();");
    ArduinoOTA.handle();
    checkDist();
  }
  Blynk.virtualWrite(V8,inches);
  if (inches <= carDist){
    Serial.println("Car");
    Blynk.virtualWrite(V9, HIGH);
  }
  else {
    Blynk.virtualWrite(V9, LOW);
  }
  
  //if Blynk button pressed, switch the flag
  if (button == HIGH){
    flag = !flag;
  }
  
  //check to see if the flag is true, then check if the gates are closed and open them
  if (flag == true && isOpen == false) {
    digitalWrite(magnet, LOW);
    delay(100);
    open();
    isOpen = true;
  }
  //else, check to see if the gates are open, and close them
  else if (flag == false && isOpen == true) {
    close();
    delay(100);
    digitalWrite(magnet, HIGH);
    isOpen = false;
  }
  
  //write the Blynk values to the LED strips
  ledcWrite(redChannel, red);
  ledcWrite(greenChannel, green);
  ledcWrite(blueChannel, blue);

  //run Blynk, yield() to prevent unwanted resets, and delay 1 millisecond to keep track of time
  Blynk.run();
  yield();
  delay(1);
}

void wificonnect(){
  Serial.println("WiFi Connecting...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED){
    Serial.print(".");
    delay(500);
    delay(500);
  }
  Serial.println("");
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }
}

void checkDist(){
  retry:
  do {
    for (int i = 0; i < 4; i++){
      data[i] = ultraSerial.read();
    }
  }
  while (ultraSerial.read() == 0xff);

  ultraSerial.flush();

  if (data[0] == 0xff)
  {
    int sum;
    sum = (data[0] + data[1] + data[2]) & 0x00FF;
    if (sum == data[3])
    {
      distance = (data[1] << 8) + data[2];
      if (distance > 20)
      {
        inches = distance / 25.4; //mm to inches
        Serial.print(distance / 10);
        Serial.println("in");
      } else
      {
        inches = 0;
        Serial.println("Below the lower limit");
      }
    } else goto retry;//Serial.println("ERROR");
  }
  delay(100);
}

void beginOpen(){
  bodo:
  digitalWrite(motorAdir, HIGH);
  digitalWrite(motorBdir, HIGH);
  ledcWrite(motorA, 255);
  ledcWrite(motorB, 255);
  yield();
  timer++;
  delay(1);
  while (timer < 24000){
    yield();
    goto bodo;
  }
  timer = 0;
  Serial.println("Done!");
  ledcWrite(motorA, 0);
  ledcWrite(motorB, 0);
}

void open(){
  int times=0;

  hallCount = 0;
  redo:
  digitalWrite(motorAdir, HIGH);
  digitalWrite(motorBdir, HIGH);
  ledcWrite(motorA, 255);
  ledcWrite(motorB, 255);

  //while (digitalRead(hall) == HIGH){yield();}
  
  while (hallCount < ppi*usedAct){
    yield();

    Serial.print("Hall count: ");
    Serial.println(hallCount);
    
    goto redo;
  }
  delay(500);
  tedo:
  digitalWrite(motorAdir, HIGH);
  digitalWrite(motorBdir, HIGH);
  ledcWrite(motorA, 255);
  ledcWrite(motorB, 255);
  times++;
  delay(1);
  yield();
  while (times < 7000){
    goto tedo;
  }
  times=0;
  yield();
  ledcWrite(motorA, 0);
  ledcWrite(motorB, 0);
}

void close(){
  hallCount = ppi*usedAct;
  redone:
  digitalWrite(motorAdir, LOW);
  digitalWrite(motorBdir, LOW);
  ledcWrite(motorA, 255);
  ledcWrite(motorB, 255);
  

  while (hallCount > 0){
    yield();

    Serial.print("Hall count: ");
    Serial.println(hallCount);
    
    goto redone; //goto endy;
  }
  yield();
  ledcWrite(motorA, 0);
  ledcWrite(motorB, 0);
  digitalWrite(motorAdir, HIGH);
  digitalWrite(motorBdir, HIGH);
  Serial.println("Gates closed");
}

void OTAStart(){
    // Port defaults to 8266
   ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
   ArduinoOTA.setHostname("DrivewayESP");

  // No authentication by default
   ArduinoOTA.setPassword("maker");
  
  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA.onStart([]() {
    //open();
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_FS
      type = "filesystem";
    }

    // NOTE: if updating FS this would be the place to unmount FS using FS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });
  ArduinoOTA.begin();
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}


ICACHE_RAM_ATTR void interruptName() {
  if (isOpen == false){
    hallCount++;
    
    if (hallCount >= ppi*usedAct){
      //ledcWrite(motorA, 0);
      //ledcWrite(motorB, 0);
      
      digitalWrite(hall, HIGH);
      hallCount = ppi*usedAct;
      //delay(1000);
    }
  }
  else if (isOpen == true){
    hallCount--;
    
    if (hallCount <= 0){
      ledcWrite(motorA, 0);
      ledcWrite(motorB, 0);

      digitalWrite(hall, HIGH);
      hallCount = 0;
      //delay(1000);
    }
  }
  //currentVal();
}

/*void currentVal() {
  //total= total - readings[inde];
  //readings[inde] = 
  currentValue = analogRead(currentIn); //Raw data reading
//Data processing:510-raw data from analogRead when the input is 0;
// 5-5v; the first 0.04-0.04V/A(sensitivity); the second 0.04-offset val;
  //readings[inde] = 
  currentValue = (currentValue-512)*3.30/4095/0.04-0.04;

  //total= total + readings[inde];
  //inde = inde + 1;
  //if (inde >= numReadings)
  //  inde = 0;
  //average = total/numReadings;   //Smoothing algorithm (http://www.arduino.cc/en/Tutorial/Smoothing)
  //currentValue= average;
  Serial.println(currentValue);
  //return(currentValue);
    //delay(1);
}*/
