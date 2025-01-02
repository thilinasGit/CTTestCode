#include <WiFiManager.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <Update.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <RFID.h>
#include <SPI.h>
#include <FastLED.h>
#include <FS.h>
#include <SD.h>
#include "SPIFFS.h"
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <MovingAverage.h>
MovingAverage<float> avrVpp(5, 0);

String version = "V2.9.P";
/**** Global variables ****/
float smv;
int smv_CT;
float smv_CAL;
int spi;
int target;
int workingHours;
int efficiencyLevel1;
int efficiencyLevel3;
String deviceId;
String sessionId_O;
String sessionId_S;
String sessionId_M;
String sessionId_P;
String encodedDeviceId;   // Encoded numeric part of Device ID
String additionalRandom;  // Additional six-character random string

String verifiedKey = "qdG1VCGs2ReHZLCBVOCiIKRGBjg1eIlMvUUHuq42giSjobzS";
String result;          // isloggedin status of OperatorRFID
String colour;          // Traffic-light colour
String MeEmployeeId;    //Mechanic Employee ID
String SuEmployeeId;    //Supervisor Employee ID
String productionLine;  //Production line name
String OPemployeeId;    //Operator Employee ID
String machineType;
String ownership;
String style;
String obbOperationId;
String mechanicName;
String qualityInsName;
String supervisorName;
String supervisorRFID;
String mechanicRFID;
String machineId;
String operationName;
String DHU_1;
float DHU;
String part;
String logStatus = "t";
int Lvalue;

unsigned long startTime;
unsigned long previousTime;
unsigned long getTime;
unsigned long endTime;
unsigned long Diff1;
unsigned long Diff2 = 0;
unsigned long Diff4 = 0;
unsigned long Diff5 = 0;
unsigned long Diff6 = 0;
unsigned long Diff3;

unsigned long mech_startTime;
unsigned long mech_endTime;
unsigned long mech_diff;
unsigned long sup_startTime;
unsigned long sup_endTime;
unsigned long sup_diff;

unsigned long operatorBreakStart;
unsigned long operatorBreakEnd;
unsigned long oper_diff;
int diffSecond;
int SDdiffSecond;
String SDTIME;
String timeStr1;
String timeStr2;
String timeStr3;

float smvCal;     /*Need to save-DONE*/
int AVG;          /*Need to save*/
int T;            /*Need to save*/
int delay1 = 10;  // Delay after one piece
int workingHr;    /*Need to save-DONE*/
int DAILY;        /*Need to save-DONE*/
int proTar = 1;   /*Need to save*/
int HrTar;        /*Need to save*/
int Tar;          /*Need to save*/
int N = 0;        /*Need to save-DONE//Acctual production*/
int SUM = 0;
int G;  /*Need to save//Total live piece count */
int LG; /*Need to save//Hourly live piece count*/
int RG;
int R;
int L;     /*Need to save-DONE*/
int J = 0; /*Need Change J = 1 when download-DONE*/
int sR;
int mR;
int SuMe = 0;
int SuMe_memo = 0;
int mechanicCheck;
int supervisorCheck;

//int CON = 0;         /*Need to save*/
int Rework_Time = 0; /*Need to save*/
int EFF_RGB_MEM = 0;

int X1;  //Operator break time timer
int Y1;
int Z1;

int BreakCheck;
int BreakOVER = 0;
int breakCount = 0;  /*Need to save*/
int bobbinCount = 0; /*Need to save*/

int stdCheck;

String Time1;
String dateTime1;
int AEM;
int Add; /*Need to save*/

int TotAdd;  /*Need to save*/
int spo_Int; /*Need to save*/
int spi_Int; /*Need to save*/
int ReadDone;
int t;            /*Need to save*/
int FirstPG = 0;  /*Change to FirstPG = 1 */
int pcsRe;        // Efficency time loop
int attendanceOp; /*Need to save*/
String avg = "average";
String act = "actual";
String TIME_FB = "time";
String NDT = "needelDownTime";
String currentPro = "currentProduction";
int tt;
int AEMM;
int errorS = 0;
int changeOP = 0;
int NO_Power = 0;
int sortRFID;
int eff_mem;
int sCallCount;
int mCallCount;
String TimeSU;
String TimeME;
String TimeFAB;
int FABNotAva;

int manualModeButton;
int autoModeButton;
int PWR_Srch = 0;
int re_pwr = 0;
int RoundTLS = 0;
String colourTLS;
int errorL;
int Hour_X = 1;
int Hour_Y = 0;
int Hour_Z = 0;
int Done45 = 0;

int Min_X = 1;
int Min_Y = 0;

int currentMinute;
int opChangeM;
int memStd;
int memLun;
int secCore = 0;
int secCore_Timer = 0;
int netConCheck;
int DHU_X = 1;
int DHU_Y;
unsigned long currentMillisDHU;
unsigned long previousMillisDHUSecondary = 0;
unsigned long previousMillisDHUMain = 0;
const long twelveMinuteInterval = 750000;  //12.30
const long oneMinuteInterval = 30000;      //.30
int opRFID_M;
String formattedCount;

/**** data receive check ****/
bool dataReceived_DHU;
bool dataReceived_initial_data;
volatile bool dataReceived_result;
bool dataReceived_OPemployeeId;
bool dataReceived_colour;
bool fnCall_initial_data = true;
bool fnCall_DHU;
bool fnCall_RFIDresult;
bool fnCall_employeeId;
bool fnCall_colour;

/**** CT variables ****/
const int adcPin = 35;
const int button = 4;
const int ledPin = 2;

const int vppBuffSize = 15;
int adcValues[vppBuffSize];
const int edgeBuffSize = 10;
float vppBuffer[edgeBuffSize];

int currentIndex = 0, bufferIndex;
bool accumulateAreaFlag = false;
double accumulatedArea = 0.0, trainedArea = 0.0, toleranceArea = 0.0;  //// should save to sd
float autoModeTolerance = 0.13;
float threshold;
float difference = 0.1;
float upperTrigger = 0.12;
float lowerTrigger = 0.05;
int c;
float idleVpp;  //// should save to sd
float idleVppAdj;

unsigned long lastEdgeMillis = 0, savedLastEdge = 0, lastPieceMillis = 0, lastBtnPressMillis = 0, autoSMV = 0;
unsigned long previousMillis = 0;  // Store the last time ctThread() was called
const long ctThreadInterval = 50;  // Interval for ctThread() in milliseconds
uint8_t restartCountDown=10;  /// has a random factor!!
const long timeUpdateInterval = 3600000;
MovingAverage<int> avrAutoSMV(5, 0);
int trainedSMV = 0;
int avrSMVBuff[10];
int avrSMVBuffIndex = 0;
int areaResetTime = 3;  // require sd save

bool awaitStop = false, awaitButtonPress = 0, hmiButtonFlag = 0, newPieceFlag = 1, waitASec = 0;  /// should save to SD
int pieceCount = 0;                                                                               //// should save to sd
unsigned int areaReseterCounter = 0;
int lastEdge = 0;
bool autoCountMode = 1;     /////// Auto manual mode selection   (to eeprom)
int buttonPressCount = 0;   /// If buttonpresscount is zero bak to traning mode   (//// should save to sd)
int trainingCountDown = 2;  /////// should save to sd
uint8_t trainingNum = 2;    /// set no of traning steps
bool awaitRisingEdge = false;

/**** MQTT Constants ****/
const char *mqtt_broker = "wb4664bb.ala.us-east-1.emqxsl.com";
const char *send_topic = "EmmanuelsLanka/commands_control-45Yb_zN$8_ELIoT_Pro_Send";  // Topic for sending data
String topic;
const char *recv_topic;
TaskHandle_t taskBHandle;
volatile bool awaitReply = 0;

const char *mqtt_username = "eliot2024";   // For the usage of server
const char *mqtt_password = "ELIoT@2024";  // For the usage of server
const int mqtt_port = 8883;
volatile int lineNo = 0;
volatile int totalLines = 0;
volatile bool spiffReady = 0;
SemaphoreHandle_t fileAccessSemaphore;

// Root CA certificate for the MQTT broker
const char *root_ca =
  "-----BEGIN CERTIFICATE-----\n"
  "MIIDrzCCApegAwIBAgIQCDvgVpBCRrGhdWrJWZHHSjANBgkqhkiG9w0BAQUFADBh\n"
  "MQswCQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3\n"
  "d3cuZGlnaWNlcnQuY29tMSAwHgYDVQQDExdEaWdpQ2VydCBHbG9iYWwgUm9vdCBD\n"
  "QTAeFw0wNjExMTAwMDAwMDBaFw0zMTExMTAwMDAwMDBaMGExCzAJBgNVBAYTAlVT\n"
  "MRUwEwYDVQQKEwxEaWdpQ2VydCBJbmMxGTAXBgNVBAsTEHd3dy5kaWdpY2VydC5j\n"
  "b20xIDAeBgNVBAMTF0RpZ2lDZXJ0IEdsb2JhbCBSb290IENBMIIBIjANBgkqhkiG\n"
  "9w0BAQEFAAOCAQ8AMIIBCgKCAQEA4jvhEXLeqKTTo1eqUKKPC3eQyaKl7hLOllsB\n"
  "CSDMAZOnTjC3U/dDxGkAV53ijSLdhwZAAIEJzs4bg7/fzTtxRuLWZscFs3YnFo97\n"
  "nh6Vfe63SKMI2tavegw5BmV/Sl0fvBf4q77uKNd0f3p4mVmFaG5cIzJLv07A6Fpt\n"
  "43C/dxC//AH2hdmoRBBYMql1GNXRor5H4idq9Joz+EkIYIvUX7Q6hL+hqkpMfT7P\n"
  "T19sdl6gSzeRntwi5m3OFBqOasv+zbMUZBfHWymeMr/y7vrTC0LUq7dBMtoM1O/4\n"
  "gdW7jVg/tRvoSSiicNoxBN33shbyTApOB6jtSj1etX+jkMOvJwIDAQABo2MwYTAO\n"
  "BgNVHQ8BAf8EBAMCAYYwDwYDVR0TAQH/BAUwAwEB/zAdBgNVHQ4EFgQUA95QNVbR\n"
  "TLtm8KPiGxvDl7I90VUwHwYDVR0jBBgwFoAUA95QNVbRTLtm8KPiGxvDl7I90VUw\n"
  "DQYJKoZIhvcNAQEFBQADggEBAMucN6pIExIK+t1EnE9SsPTfrgT1eXkIoyQY/Esr\n"
  "hMAtudXH/vTBH1jLuG2cenTnmCmrEbXjcKChzUyImZOMkXDiqw8cvpOp/2PV5Adg\n"
  "06O/nVsJ8dWO41P0jmP6P6fbtGbfYmbW0W5BjfIttep3Sp+dWOIrWcBAI+0tKIJF\n"
  "PnlUkiaY4IBIqDfv8NZ5YBberOgOzW6sRBc4L0na4UU+Krk2U886UAb3LujEV0ls\n"
  "YSEY1QSteDwsOoBrp+uvFRTp2InBuThs4pFsiv9kuXclVzDAGySj4dzp30d8tbQk\n"
  "CAUw7C29C79Fv1C5qfPrmAESrciIxpg0X40KPMbp1ZWVbd4=\n"
  "-----END CERTIFICATE-----\n";

/**** Initialize MQTT client ****/
WiFiClientSecure espClient;  // Secure WiFi client for TLS/SSL
PubSubClient client(espClient);

/**** wifi password setup and code upload wirelessly ****/
char command;
WiFiManager wm;

/**** RFID card pin configuration ****/
#define SS_PIN 21
#define RST_PIN 22

unsigned char serNum[5];
unsigned char status;
unsigned char blockAddr;
String dataa = "";

String OperatorID;  /*Need to save*/
int countBreak = 0; /*Need to save*/

String supervisorID; /*Need to save*/
String mechanicID;   /*Need to save*/
String qcID;         /*Need to save*/
int qc_count = 0;    /*Need to save*/

int S_mem = 0; /*Need to save*/
int M_mem = 0; /*Need to save*/

int wifiData = 2;

unsigned char sectorKeyA[16][16] = {
  { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF },
  { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF },
  { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF },
  { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF },
  { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF },
  { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF },
  { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF },
  { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF },
  { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF },
  { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF },
  { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF },
  { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF },
  { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF },
  { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF },
  { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF },
  { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF },
};

RFID rfid(SS_PIN, RST_PIN);
unsigned char str[MAX_LEN];  //MAX_LEN is 16: size of the array


/**** SD card pin configuration ****/
#define SS_SD 15
String rdline;


/**** DWIN page find data types ****/
String buttonNo;
String PAGE;
String pageNo;
String RFIDpageNo;
int M = 0;

char output[50];


/**** RGB LED PANEL ****/
#define LED_PIN 13
#define NUM_LEDS 18

CRGB leds[NUM_LEDS];

int E_RGB_LOW;
int E_RGB_HIGH;


/**** Sever time setup ****/
WiFiUDP ntpUDP;
//const long utcOffsetInSeconds = 19800; //(UTC+5:30)
const long utcOffsetInSeconds = 21600;  //(UTC+6)
NTPClient timeClient(ntpUDP, "pool.ntp.org", utcOffsetInSeconds);
time_t localEpochTime = 0;


/**** Time String Convert ****/
// Function to convert a time string in "HH:MM:SS" format to seconds since midnight
time_t timeStringToSeconds(const String &timeStr) {
  int hours, minutes, seconds;
  if (sscanf(timeStr.c_str(), "%d:%d:%d", &hours, &minutes, &seconds) == 3) {
    return hours * 3600 + minutes * 60 + seconds;
  }
  return 0;  // Return 0 if the string is not in the expected format
}

// Function to convert seconds since midnight to a time string in "HH:MM:SS" format
String secondsToTimeFormat(time_t seconds) {
  int hours = seconds / 3600;
  int minutes = (seconds % 3600) / 60;
  int sec = seconds % 60;
  char buffer[9];
  snprintf(buffer, sizeof(buffer), "%02d:%02d:%02d", hours, minutes, sec);
  return String(buffer);
}


/**** 2nd  CORE Accessing ****/
#include <esp_task_wdt.h>
TaskHandle_t Task1;

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

void sendSerialData(uint8_t *data, size_t size) {
  for (size_t i = 0; i < size; i++) {
    Serial2.write(data[i]);
  }
}

void blinkLED(CRGB color1, CRGB color2, int delayTime) {
  leds[0] = color1;
  leds[1] = color1;
  FastLED.show();
  vTaskDelay(delayTime / portTICK_PERIOD_MS);
  leds[0] = color2;
  leds[1] = color2;
  FastLED.show();
  vTaskDelay(delayTime / portTICK_PERIOD_MS);
}

void codeForTask1(void *parameter) {
  uint8_t data1[] = { 0x5A, 0xA5, 0x05, 0x82, 0x02, 0x00, 0x00, 0x01 };
  uint8_t data2[] = { 0x5A, 0xA5, 0x05, 0x82, 0x02, 0x00, 0x00, 0x00 };

  while (1) {
    if (secCore_Timer == 1) {
      onTheHour();  //Efficiency meter
    }

    checkInternetConnectivity();

    if (netConCheck == 1) {

      wifiData = 2;
      vTaskDelay(1000 / portTICK_PERIOD_MS);  // Non-blocking delay
      sendSerialData(data2, sizeof(data2));   // Send data2 three times
      sendSerialData(data2, sizeof(data2));
      sendSerialData(data2, sizeof(data2));
    }

    if (netConCheck == 2) {
      wifiData = 1;
      vTaskDelay(1000 / portTICK_PERIOD_MS);  // Non-blocking delay
      sendSerialData(data1, sizeof(data1));   // Send data1 three times
      sendSerialData(data1, sizeof(data1));
      sendSerialData(data1, sizeof(data1));

      // Blink LED in red
      blinkLED(CRGB(255, 0, 0), CRGB(0, 0, 0), 300);
      blinkLED(CRGB(255, 0, 0), CRGB(0, 0, 0), 300);

      DHUfunction();
    }

    if (WiFi.status() != WL_CONNECTED) {
      wifiData = 1;                           // When Wifi disconnects
      vTaskDelay(1000 / portTICK_PERIOD_MS);  // Non-blocking delay
      sendSerialData(data1, sizeof(data1));   // Send data1 three times
      sendSerialData(data1, sizeof(data1));
      sendSerialData(data1, sizeof(data1));

      reconnectWiFi();  // Attempt to reconnect to Wi-Fi
      wifiData = 2;     // When Wi-Fi reconnects

      DHUfunction();
      vTaskDelay(1000 / portTICK_PERIOD_MS);  // Non-blocking delay
      sendSerialData(data2, sizeof(data2));   // Send data2 three times
      sendSerialData(data2, sizeof(data2));
      sendSerialData(data2, sizeof(data2));
    }

    // Supervisor and mechanic LED blinking
    if ((S_mem == 1) && (M_mem == 1)) {
      blinkLED(CRGB(255, 0, 255), CRGB(0, 0, 0), 1000);  // Magenta
      blinkLED(CRGB(0, 255, 255), CRGB(0, 0, 0), 1000);  // Cyan
    }

    // Supervisor LED blinking function
    else if (S_mem == 1) {
      blinkLED(CRGB(255, 0, 255), CRGB(0, 0, 0), 1000);  // Magenta
    }

    // Mechanic LED blinking function
    else if (M_mem == 1) {
      blinkLED(CRGB(0, 255, 255), CRGB(0, 0, 0), 1000);  // Cyan
    }
  }
}


void setup() {

  Serial.begin(115200);

  /**** CT sensor ****/
  pinMode(ledPin, OUTPUT);
  pinMode(button, INPUT);
  pinMode(adcPin, INPUT);
  pinMode(21, OUTPUT);
  pinMode(15, OUTPUT);
  digitalWrite(15, HIGH);
  digitalWrite(ledPin, HIGH);
  //setTrainingMode(trainingNum);  /// call this function to start training, parameter num of training

  /**** RFID function ****/
  digitalWrite(21, HIGH);
  if (!SD.begin(SS_SD)) {
    Serial.println("Card Mount Failed");
    return;
  }

  if (!SPIFFS.begin(true)) {  // 'true' will format SPIFFS if not formatted already
    Serial.println("An error occurred while mounting SPIFFS");
    return;
  }
  Serial.println("SPIFFS mounted successfully");


  fileAccessSemaphore = xSemaphoreCreateMutex();
  /**** SD card function setup ****/

  uint8_t cardType = SD.cardType();

  if (cardType == CARD_NONE) {
    Serial.println("No SD card attached");
    return;
  }

  readFile(SD, "/82.txt"); /*SD Read*/
  deviceId = rdline;
  delay(200);

  readFile(SD, "/46.txt"); /*SD Read*/
  J = rdline.toInt();
  if (ReadDone == 0) {
    J = 0;
  }

  startTrafficLight();
  //delay(2500);
  Serial2.begin(115200);

  delay(300);
  send_text19(deviceId);
  send_text19(deviceId);
  send_text19(deviceId);

  topic = "EmmanuelsLanka/commands_control-45Yb_zN$8_ELIoT_Pro_Receive/" + deviceId;  // Topic for receiving data
  recv_topic = topic.c_str();

  /**** WIFI connect function ****/
  bool res;
  randomSeed(analogRead(adcPin));
   delay(200*random(20));   /// random delay to avoid simultanious connection
  //res = wm.autoConnect(deviceId.c_str(), "eliot@2024");  // anonymous ap
  res = wm.autoConnect(deviceId.c_str(), "1111111111");  // anonymous ap
  if (!res) {
    Serial.println("Failed to connect");
    WiFi.reconnect();
    //reconnectWiFi();
    //ESP.restart();
  } else {
    //if you get here you have connected to the WiFi
    Serial.println("connected...yeey :)");
  }

  /**** MQTT connect function ****/
  MQTT_Connect();

  /**** Second core accessing ****/
  xTaskCreatePinnedToCore(
    codeForTask1,
    "led1Task",
    5000,
    NULL,
    1,
    &Task1,
    0);
  esp_task_wdt_init(36000, false);

  /**** RGB Led function setup ****/
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);

  leds[0] = CRGB(0, 0, 0);
  FastLED.show();

  leds[1] = CRGB(0, 0, 0);
  FastLED.show();

  leds[2] = CRGB(0, 0, 0);
  FastLED.show();

  leds[3] = CRGB(0, 0, 0);
  FastLED.show();


  readFile(SD, "/22.txt"); /*SD Read*/
  attendanceOp = rdline.toInt();
  delay(300);

  readFile(SD, "/21.txt"); /*SD Read*/
  changeOP = rdline.toInt();
  if (ReadDone == 0) {
    changeOP = 0;
  }
  delay(300);

  readFile(SD, "/35.txt"); /*SD Read*/
  BreakCheck = rdline.toInt();
  delay(300);

  readFile(SD, "/71.txt"); /*SD Read*/
  opChangeM = rdline.toInt();
  delay(300);

  readFile(SD, "/72.txt"); /*SD Read*/
  SuMe = rdline.toInt();
  delay(300);

  readFile(SD, "/86.txt"); /*SD Read*/
  stdCheck = rdline.toInt();

  readSPIFFS("/111.txt"); /*SD Read*/
  lineNo = rdline.toInt();

  readSPIFFS("/112.txt"); /*SD Read*/
  totalLines = rdline.toInt();


  Serial.print("BreakCheck = ");
  Serial.println(BreakCheck);
  Serial.println(SuMe);
  Serial.println(deviceId);
  /**** New Id genersting ****/
  randomSeed(analogRead(0));
  encodedDeviceId = encodeDeviceId(deviceId);  // Encode the numeric part of the device ID

  String autoM = "A";
  send_text6(autoM);
  send_text6(autoM);

  send_text24(version);
  send_text24(version);

  updateTime();

  spiffReady = 1;
  xTaskCreatePinnedToCore(
    mqttThread,    // Task function
    "mqttThread",  // Name of the task
    5000,          // Stack size
    NULL,          // Parameter
    2,             // Priority (higher)
    &taskBHandle,  // Task handle
    0              // Pin to Core 0
  );
}

void updateTime() {
  Serial.println("Running time sync loop..");
  while (timeClient.getEpochTime() < 1721184478) {
    timeClient.update();
    localEpochTime = timeClient.getEpochTime();
    delay(100);  // Wait before retrying
  }
}


/**** RFID Access Function ****/
void checkAccess(String dataa) {  //Function to check if an identified tag is registered to allow access

  String strings[] = { dataa };
  Serial.println(dataa);
  int n = sizeof(strings) / sizeof(strings[0]);
  for (int i = 0; i < n; i++) {
    getPrefixNumber(strings[i]);
  }

  /**** Operator ****/
  if (sortRFID == 1) {

    /**** Send OperatorRFID to NODE RED ****/
    if (fnCall_RFIDresult) {
      send_OperatorRFID(dataa, deviceId);
    }
    Serial.println("Await reply");
    opRFID_M = 1;

    for (int d = 0; d < 50; d++) {
      delay(100);
      if (dataReceived_result) d = 50;
    }

    if (dataReceived_result) {

      if ((result == "1" || result == "0") && ((pageNo == "1") || (pageNo == "9") || (RFIDpageNo == "1") || (RFIDpageNo == "9"))) {

        if ((result == "0") && ((pageNo == "1") || (RFIDpageNo == "1")))  //If result =0 -- login
        {
          opRFID_M = 0;
          M = 0;
          RFIDpageNo = "";
          errorS = 1;

          time_t epochTime = timeClient.getEpochTime();
          struct tm *ptm = gmtime((time_t *)&epochTime);

          char formattedTime[20];
          strftime(formattedTime, sizeof(formattedTime), "%Y%m%d%H%M%S", ptm);
          String dateTimee = String(formattedTime);

          int monthDay = ptm->tm_mday;
          int currentMonth = ptm->tm_mon + 1;
          int currentYear = ptm->tm_year + 1900;

          // Ensure month and day are two digits
          String formattedMonth = (currentMonth < 10) ? "0" + String(currentMonth) : String(currentMonth);
          String formattedDay = (monthDay < 10) ? "0" + String(monthDay) : String(monthDay);

          String currentDate = String(currentYear) + "-" + formattedMonth + "-" + formattedDay;
          String Time = String(timeClient.getFormattedTime());
          String dateTime = currentDate + " " + Time;


          writeFile(SD, "/23.txt", Time.c_str()); /*SD Save*/
          //DD = String currentDate;

          attendanceOp = 1;
          String attenOp = String(attendanceOp);
          writeFile(SD, "/22.txt", attenOp.c_str()); /*SD Save*/

          changeOP = 0;
          String cOperaor = String(changeOP);
          writeFile(SD, "/21.txt", cOperaor.c_str()); /*SD Save*/

          OperatorID = dataa;

          writeFile(SD, "/24.txt", OperatorID.c_str()); /*SD Save*/

          additionalRandom = generateRandomString(4);                                          // Generate additional characters random string
          sessionId_O = encodedDeviceId + additionalRandom + "-" + encodeDateTime(dateTimee);  // Construct full ID

          Serial.print("dateTimee");
          Serial.println(dateTimee);

          String sessionIdO = String(sessionId_O);
          writeFile(SD, "/63.txt", sessionIdO.c_str()); /*SD Save*/


          /* NODE RED */
          send_loginTS(OperatorID, dateTime);  // send operator login timestamp
          set_operatorSession(sessionId_O, OperatorID, obbOperationId, "t", dateTime);
          append_operator_t(OperatorID);

          pageShift1();  //Page 02 (Main page)
          send_text1(dataa);

        } else if ((result == "1") && ((pageNo == "9") || (RFIDpageNo == "9")) && ((OperatorID == dataa) || (dataa == "OP-00000"))) {  //If result =1 -- logout

          opRFID_M = 0;
          M = 0;
          RFIDpageNo = "";
          FirstPG = 0;
          secCore_Timer = 0;

          time_t epochTime = timeClient.getEpochTime();
          struct tm *ptm = gmtime((time_t *)&epochTime);

          char formattedTime[20];
          strftime(formattedTime, sizeof(formattedTime), "%Y%m%d%H%M%S", ptm);
          //String dateTime = String(formattedTime);

          int monthDay = ptm->tm_mday;
          int currentMonth = ptm->tm_mon + 1;
          int currentYear = ptm->tm_year + 1900;

          // Ensure month and day are two digits
          String formattedMonth = (currentMonth < 10) ? "0" + String(currentMonth) : String(currentMonth);
          String formattedDay = (monthDay < 10) ? "0" + String(monthDay) : String(monthDay);

          String currentDate = String(currentYear) + "-" + formattedMonth + "-" + formattedDay;
          String Time = String(timeClient.getFormattedTime());
          String dateTime = currentDate + " " + Time;

          String NA = "N/A";
          attendanceOp = 0;
          String attenOp = String(attendanceOp);
          writeFile(SD, "/22.txt", attenOp.c_str()); /*SD Save*/

          /* NODE RED */
          send_logoutTS(OperatorID, dateTime);  //send operator logout timestamp
          append_operatorSession(sessionId_O, dateTime);
          append_operator_f(OperatorID);

          pageShift18();  // Page 21 (Page before Network setting up page)
        } else if ((dataa.length() > 0) || (wifiData == 1)) {
          if (M == 0) {
            RFIDpageNo = pageNo;
            M = 1;
          }
          pageShift6();  //Page 20 (Tryagain Page)
        }

      } else if ((result == "2") || (pageNo == "7") || (pageNo == "8") || (pageNo == "10") || (pageNo == "5") || (pageNo == "6") || (wifiData == 1) || (opRFID_M = 1)) {

        if (M == 0) {
          RFIDpageNo = pageNo;
          M = 1;
        }
        pageShift6();  //Page 20 (Tryagain Page)
      }
      dataReceived_result = false;
    } else if ((wifiData == 1) || (opRFID_M = 1)) {
      if (M == 0) {
        RFIDpageNo = pageNo;
        M = 1;
      }
      pageShift6();  //Page 20 (Tryagain Page)
    }
  }


  /**** Supervisor ****/
  else if (sortRFID == 2) {

    if ((dataa == supervisorRFID) && ((pageNo == "7") || (pageNo == "8") || (RFIDpageNo == "7") || (RFIDpageNo == "8"))) {

      if ((supervisorCheck == 0) && ((pageNo == "7") || (RFIDpageNo == "7"))) {

        M = 0;
        sR = 0;
        String sRR = String(sR);
        writeFile(SD, "/25.txt", sRR.c_str()); /*SD Save*/

        SuMe = 1;
        String SuMe_M = String(SuMe);
        writeFile(SD, "/72.txt", SuMe_M.c_str()); /*SD Save*/

        SuMe_memo = 1;
        String SuMe_memo_M = String(SuMe_memo);
        writeFile(SD, "/73.txt", SuMe_memo_M.c_str()); /*SD Save*/

        RFIDpageNo = "";
        supervisorCheck = 1;

        sup_endTime = millis();
        sup_diff = sup_endTime - sup_startTime;
        int timeDelay_sup = sup_diff / 1000;

        time_t epochTime = timeClient.getEpochTime();
        struct tm *ptm = gmtime((time_t *)&epochTime);

        char formattedTime[20];
        strftime(formattedTime, sizeof(formattedTime), "%Y%m%d%H%M%S", ptm);
        String dateTimee = String(formattedTime);

        int monthDay = ptm->tm_mday;
        int currentMonth = ptm->tm_mon + 1;
        int currentYear = ptm->tm_year + 1900;

        // Ensure month and day are two digits
        String formattedMonth = (currentMonth < 10) ? "0" + String(currentMonth) : String(currentMonth);
        String formattedDay = (monthDay < 10) ? "0" + String(monthDay) : String(monthDay);

        String currentDate = String(currentYear) + "-" + formattedMonth + "-" + formattedDay;
        String Time = String(timeClient.getFormattedTime());
        String dateTime = currentDate + " " + Time;


        S_mem = 0;
        String smem = String(S_mem);
        writeFile(SD, "/26.txt", smem.c_str()); /*SD Save*/

        delay(2000);

        DHUfunction();


        /* NODE RED */
        send_SUBreakStartTS(OperatorID, dateTime);  // Send supervisor break start timestamp
        set_staffSession(sessionId_S, SuEmployeeId, obbOperationId, "t", dateTime);
        set_LogStatus(verifiedKey, machineId, SuEmployeeId, "login");


        uint8_t data1[] = { 0x5A, 0xA5, 0x05, 0x82, 0x17, 0x00, 0x00, 0x00 };
        for (size_t i = 0; i < 8; i++) {
          Serial2.write(data1[i]);
        }

        uint8_t data2[] = { 0x5A, 0xA5, 0x05, 0x82, 0x17, 0x00, 0x00, 0x00 };
        for (size_t i = 0; i < 8; i++) {
          Serial2.write(data2[i]);
        }

        pageShift3();  // Page 16 (Issue in progress)

      } else if ((supervisorCheck == 1) && ((pageNo == "8") || (RFIDpageNo == "8"))) {
        M = 0;
        RFIDpageNo = "";
        supervisorCheck = 0;

        SuMe = 0;
        String SuMe_M = String(SuMe);
        writeFile(SD, "/72.txt", SuMe_M.c_str()); /*SD Save*/

        SuMe_memo = 0;
        String SuMe_memo_M = String(SuMe_memo);
        writeFile(SD, "/73.txt", SuMe_memo_M.c_str()); /*SD Save*/

        time_t epochTime = timeClient.getEpochTime();
        struct tm *ptm = gmtime((time_t *)&epochTime);

        char formattedTime[20];
        strftime(formattedTime, sizeof(formattedTime), "%Y%m%d%H%M%S", ptm);

        int monthDay = ptm->tm_mday;
        int currentMonth = ptm->tm_mon + 1;
        int currentYear = ptm->tm_year + 1900;

        // Ensure month and day are two digits
        String formattedMonth = (currentMonth < 10) ? "0" + String(currentMonth) : String(currentMonth);
        String formattedDay = (monthDay < 10) ? "0" + String(monthDay) : String(monthDay);

        String currentDate = String(currentYear) + "-" + formattedMonth + "-" + formattedDay;
        String Time = String(timeClient.getFormattedTime());
        String dateTime = currentDate + " " + Time;


        /* NODE RED */
        send_SUBreakEndTS(OperatorID, dateTime);  // Send supervisor break end timestamp
        append_staffSession(sessionId_S, dateTime);
        set_LogStatus(verifiedKey, machineId, SuEmployeeId, "logout");

        pageShift1();  // Page 02 (Main page)
      }
    } else if ((dataa == supervisorRFID) && ((pageNo == "10") || (RFIDpageNo == "10"))) {

      if (autoModeButton == 1) {

        autoCountMode = 1;
        autoModeButton = 0;
        String autoM = "A";

        send_text6(autoM);
        send_text6(autoM);

        String autoCM = String(autoCountMode);
        writeFile(SD, "/27.txt", autoCM.c_str()); /*SD Save*/

      } else if (manualModeButton == 1) {

        autoCountMode = 0;
        manualModeButton = 0;
        String manualM = "M";
        send_text6(manualM);
        send_text6(manualM);

        String autoCM = String(autoCountMode);
        writeFile(SD, "/27.txt", autoCM.c_str()); /*SD Save*/
      }

      pageShift1();  // Page 02 (Main page)

    } else if ((pageNo == "5") || (pageNo == "6") || (pageNo == "1") || (pageNo == "9")) {
      if (M == 0) {
        RFIDpageNo = pageNo;
        M = 1;
      }
      pageShift6();  //Page 20 (Tryagain Page)
    } else {

      if (M == 0) {
        RFIDpageNo = pageNo;
        M = 1;
      }
      pageShift6();  //Page 20 (Tryagain Page)
    }

  }


  /**** Mechanic ****/
  else if (sortRFID == 3) {
    Serial.println("Mechanic ");
    Serial.println(mechanicRFID);

    if ((dataa == mechanicRFID) && ((pageNo == "5") || (pageNo == "6") || (RFIDpageNo == "5") || (RFIDpageNo == "6"))) {

      if ((mechanicCheck == 0) && ((pageNo == "5") || (RFIDpageNo == "5"))) {
        M = 0;
        mR = 0;
        String mRR = String(mR);
        writeFile(SD, "/28.txt", mRR.c_str()); /*SD Save*/

        SuMe = 1;
        String SuMe_M = String(SuMe);
        writeFile(SD, "/72.txt", SuMe_M.c_str()); /*SD Save*/

        SuMe_memo = 2;
        String SuMe_memo_M = String(SuMe_memo);
        writeFile(SD, "/73.txt", SuMe_memo_M.c_str()); /*SD Save*/

        RFIDpageNo = "";

        mechanicCheck = 1;
        String mechanicCheck_M = String(mechanicCheck);
        writeFile(SD, "/74.txt", mechanicCheck_M.c_str()); /*SD Save*/

        mech_endTime = millis();
        mech_diff = mech_endTime - mech_startTime;
        int timeDelay_mech = mech_diff / 1000;

        time_t epochTime = timeClient.getEpochTime();
        struct tm *ptm = gmtime((time_t *)&epochTime);

        char formattedTime[20];
        strftime(formattedTime, sizeof(formattedTime), "%Y%m%d%H%M%S", ptm);
        String dateTimee = String(formattedTime);

        int monthDay = ptm->tm_mday;
        int currentMonth = ptm->tm_mon + 1;
        int currentYear = ptm->tm_year + 1900;

        // Ensure month and day are two digits
        String formattedMonth = (currentMonth < 10) ? "0" + String(currentMonth) : String(currentMonth);
        String formattedDay = (monthDay < 10) ? "0" + String(monthDay) : String(monthDay);

        String currentDate = String(currentYear) + "-" + formattedMonth + "-" + formattedDay;
        String Time = String(timeClient.getFormattedTime());
        String dateTime = currentDate + " " + Time;

        M_mem = 0;
        String mmem = String(M_mem);
        writeFile(SD, "/29.txt", mmem.c_str()); /*SD Save*/

        delay(2000);

        DHUfunction();


        /* NODE RED */
        send_MEBreakStartTS(OperatorID, dateTime);  // Send mechanic break start timestamp
        set_staffSession(sessionId_M, MeEmployeeId, obbOperationId, "t", dateTime);
        set_LogStatus(verifiedKey, machineId, MeEmployeeId, "login");

        uint8_t data1[] = { 0x5A, 0xA5, 0x05, 0x82, 0x20, 0x00, 0x00, 0x00 };
        for (size_t i = 0; i < 8; i++) {
          Serial2.write(data1[i]);
        }

        uint8_t data2[] = { 0x5A, 0xA5, 0x05, 0x82, 0x20, 0x00, 0x00, 0x00 };
        for (size_t i = 0; i < 8; i++) {
          Serial2.write(data2[i]);
        }
        pageShift4();  // Page 17 (Repair in progress)

      }

      else if ((mechanicCheck == 1) && ((pageNo == "6") || (RFIDpageNo == "6"))) {
        M = 0;
        RFIDpageNo = "";
        mechanicCheck = 0;

        SuMe = 0;
        String SuMe_M = String(SuMe);
        writeFile(SD, "/72.txt", SuMe_M.c_str()); /*SD Save*/

        SuMe_memo = 0;
        String SuMe_memo_M = String(SuMe_memo);
        writeFile(SD, "/73.txt", SuMe_memo_M.c_str()); /*SD Save*/

        time_t epochTime = timeClient.getEpochTime();
        struct tm *ptm = gmtime((time_t *)&epochTime);

        char formattedTime[20];
        strftime(formattedTime, sizeof(formattedTime), "%Y%m%d%H%M%S", ptm);

        int monthDay = ptm->tm_mday;
        int currentMonth = ptm->tm_mon + 1;
        int currentYear = ptm->tm_year + 1900;

        // Ensure month and day are two digits
        String formattedMonth = (currentMonth < 10) ? "0" + String(currentMonth) : String(currentMonth);
        String formattedDay = (monthDay < 10) ? "0" + String(monthDay) : String(monthDay);

        String currentDate = String(currentYear) + "-" + formattedMonth + "-" + formattedDay;
        String Time = String(timeClient.getFormattedTime());
        String dateTime = currentDate + " " + Time;


        /* NODE RED */
        send_MEBreakEndTS(OperatorID, dateTime);  // Send mechanic break end timestamp
        append_staffSession(sessionId_M, dateTime);
        set_LogStatus(verifiedKey, machineId, MeEmployeeId, "logout");

        pageShift1();  // Page 02 (Main page)
      }
    } else if ((pageNo == "7") || (pageNo == "8") || (pageNo == "10") || (pageNo == "1") || (pageNo == "9")) {
      if (M == 0) {
        RFIDpageNo = pageNo;
        M = 1;
      }
      pageShift6();  //Page 20 (Tryagain Page)
    } else {

      if (M == 0) {
        RFIDpageNo = pageNo;
        M = 1;
      }
      pageShift6();  //Page 20 (Tryagain Page)
    }
  }
}


void loop() {
  //timeClient.update();
  checker();  //Page and button function checker
  displayDWIN();
}

/**** RFID Card Reading Function ****/
void displayDWIN() {

  /**** Main page functions ****/
  if (pageNo == "2") {  //Main Page function
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= ctThreadInterval) {
      previousMillis = currentMillis;
      ctThread();
      secCore = 1;
    }

    errorS = 1;
    memStd = 1;
    memLun = 1;
    secCore_Timer = 1;

    time_t epochTime = timeClient.getEpochTime();
    struct tm *ptm = gmtime(&localEpochTime);
    int monthDay = ptm->tm_mday;
    int currentMonth = ptm->tm_mon + 1;
    int currentYear = ptm->tm_year + 1900;
    String currentDate = String(currentYear) + "-" + String(currentMonth) + "-" + String(monthDay);
    Time1 = String(timeClient.getFormattedTime());

    dateTime1 = String(currentDate) + " " + String(Time1);
    send_text2(Time1);

    return;
  }

  /**** RFID Check function ****/
  else if ((pageNo == "1") || (pageNo == "5") || (pageNo == "6") || (pageNo == "7") || (pageNo == "8") || (pageNo == "9") || (pageNo == "10")) {
    errorS = 1;

    digitalWrite(SS_SD, HIGH);
    digitalWrite(SS_PIN, LOW);
    SPI.begin();
    rfid.init();
    rfid.findCard(PICC_REQIDL, str);

    if (rfid.anticoll(str) == MI_OK) {
      Serial.print("The card's number is  : ");

      for (int i = 0; i < 4; i++) {
        Serial.print(0x0F & (str[i] >> 4), HEX);
        Serial.print(0x0F & str[i], HEX);
      }
      memcpy(rfid.serNum, str, 5);
    }
    rfid.selectTag(rfid.serNum);
    readCard(2);
    rfid.halt();

    digitalWrite(SS_PIN, HIGH);
  }

  /*** Training function ***/
  else if (pageNo == "27") {
    errorS = 1;
    hmiButtonFlag = true;
    delay(300);
    pageShift1();
  }

  /*** Reset function ***/
  else if (pageNo == "28") {
    errorS = 1;
    setTrainingMode(trainingNum);
    delay(350);
    pageShift1();
  }

  /**** Mechanic call function ****/
  else if ((pageNo == "18") && (mR == 0)) {
    errorS = 1;
    mR = 1;
    mCallCount++;
    String mCall = String(mCallCount);
    String mRR = String(mR);
    writeFile(SD, "/30.txt", mCall.c_str()); /*SD Save*/
    writeFile(SD, "/28.txt", mRR.c_str());   /*SD Save*/

    time_t epochTime = timeClient.getEpochTime();
    struct tm *ptm = gmtime((time_t *)&epochTime);

    char formattedTime[20];
    strftime(formattedTime, sizeof(formattedTime), "%Y%m%d%H%M%S", ptm);
    String dateTimee = String(formattedTime);

    int monthDay = ptm->tm_mday;
    int currentMonth = ptm->tm_mon + 1;
    int currentYear = ptm->tm_year + 1900;

    // Ensure month and day are two digits
    String formattedMonth = (currentMonth < 10) ? "0" + String(currentMonth) : String(currentMonth);
    String formattedDay = (monthDay < 10) ? "0" + String(monthDay) : String(monthDay);

    String currentDate = String(currentYear) + "-" + formattedMonth + "-" + formattedDay;
    String Time = String(timeClient.getFormattedTime());
    String dateTime = currentDate + " " + Time;
    String Date = String(currentDate);
    TimeME = String(currentDate) + " " + String(Time);


    additionalRandom = generateRandomString(4);                                          // Generate additional characters random string
    sessionId_M = encodedDeviceId + additionalRandom + "-" + encodeDateTime(dateTimee);  // Construct full ID

    Serial.println(sessionId_M);
    String sessionIdM = String(sessionId_M);
    writeFile(SD, "/65.txt", sessionIdM.c_str()); /*SD Save*/

    writeFile(SD, "/31.txt", TimeME.c_str()); /*SD Save*/


    /*Node RED*/
    set_AlertLog(verifiedKey, machineId, OperatorID, MeEmployeeId, "Mechanics");
    set_SessionId(sessionId_M);

    Serial.println("sessionId_M");
    mech_startTime = millis();

    uint8_t data1[] = { 0x5A, 0xA5, 0x05, 0x82, 0x20, 0x00, 0x00, 0x01 };  // Mechanic icon change to green color
    for (size_t i = 0; i < 8; i++) {
      Serial2.write(data1[i]);
    }
    uint8_t data2[] = { 0x5A, 0xA5, 0x05, 0x82, 0x20, 0x00, 0x00, 0x01 };  // Mechanic icon change to green color
    for (size_t i = 0; i < 8; i++) {
      Serial2.write(data2[i]);
    }
    uint8_t data3[] = { 0x5A, 0xA5, 0x05, 0x82, 0x20, 0x00, 0x00, 0x01 };  // Mechanic icon change to green color
    for (size_t i = 0; i < 8; i++) {
      Serial2.write(data2[i]);
    }

    M_mem = 1;
    String mmem = String(M_mem);
    writeFile(SD, "/29.txt", mmem.c_str()); /*SD Save*/

    pageShift1();
    return;
  }

  else if ((pageNo == "18") && (mR != 0)) {
    errorS = 1;
    pageShift1();
  }

  /**** supervisor call function ****/
  else if ((pageNo == "19") && (sR == 0)) {
    errorS = 1;
    sR = 1;
    sCallCount++;

    String sCall = String(sCallCount);
    String sRR = String(sR);
    writeFile(SD, "/32.txt", sCall.c_str()); /*SD Save*/
    writeFile(SD, "/25.txt", sRR.c_str());   /*SD Save*/

    time_t epochTime = timeClient.getEpochTime();
    struct tm *ptm = gmtime((time_t *)&epochTime);

    char formattedTime[20];
    strftime(formattedTime, sizeof(formattedTime), "%Y%m%d%H%M%S", ptm);
    String dateTimee = String(formattedTime);

    int monthDay = ptm->tm_mday;
    int currentMonth = ptm->tm_mon + 1;
    int currentYear = ptm->tm_year + 1900;

    // Ensure month and day are two digits
    String formattedMonth = (currentMonth < 10) ? "0" + String(currentMonth) : String(currentMonth);
    String formattedDay = (monthDay < 10) ? "0" + String(monthDay) : String(monthDay);

    String currentDate = String(currentYear) + "-" + formattedMonth + "-" + formattedDay;
    String Time = String(timeClient.getFormattedTime());
    String dateTime = currentDate + " " + Time;
    TimeSU = String(currentDate) + " " + String(Time);

    additionalRandom = generateRandomString(4);                                          // Generate additional characters random string
    sessionId_S = encodedDeviceId + additionalRandom + "-" + encodeDateTime(dateTimee);  // Construct full ID

    String sessionIdS = String(sessionId_S);
    Serial.println(sessionId_S);
    writeFile(SD, "/64.txt", sessionIdS.c_str()); /*SD Save*/

    writeFile(SD, "/33.txt", TimeSU.c_str()); /*SD Save*/


    /* NODE RED */
    set_AlertLog(verifiedKey, machineId, OperatorID, SuEmployeeId, "Supervisors");
    set_SessionId(sessionId_S);

    Serial.println("sessionId_S");
    sup_startTime = millis();

    uint8_t data1[] = { 0x5A, 0xA5, 0x05, 0x82, 0x17, 0x00, 0x00, 0x01 };  // Supervisor icon change to green color
    for (size_t i = 0; i < 8; i++) {
      Serial2.write(data1[i]);
    }

    uint8_t data2[] = { 0x5A, 0xA5, 0x05, 0x82, 0x17, 0x00, 0x00, 0x01 };  // Supervisor icon change to green color
    for (size_t i = 0; i < 8; i++) {
      Serial2.write(data2[i]);
    }

    uint8_t data3[] = { 0x5A, 0xA5, 0x05, 0x82, 0x17, 0x00, 0x00, 0x01 };  // Supervisor icon change to green color
    for (size_t i = 0; i < 8; i++) {
      Serial2.write(data2[i]);
    }

    S_mem = 1;
    String smem = String(S_mem);
    writeFile(SD, "/26.txt", smem.c_str()); /*SD Save*/

    pageShift1();
    return;
  }

  else if ((pageNo == "19") && (sR != 0)) {
    errorS = 1;
    pageShift1();
  }

  /***Standard time starting function***/
  else if (pageNo == "20") {

    errorS = 1;

    if (memStd == 1) {
      errorS = 1;
      stdCheck = 1;
      String SDstdCheck = String(stdCheck);
      writeFile(SD, "/86.txt", SDstdCheck.c_str()); /*SD Save*/


      time_t epochTime = timeClient.getEpochTime();
      struct tm *ptm = gmtime((time_t *)&epochTime);
      int monthDay = ptm->tm_mday;
      int currentMonth = ptm->tm_mon + 1;
      int currentYear = ptm->tm_year + 1900;
      String currentDate = String(currentYear) + "-" + String(currentMonth) + "-" + String(monthDay);
      String Time = String(timeClient.getFormattedTime());
      String dateTime = String(currentDate) + " " + String(Time);

      send_OffstandStartTS(OperatorID, dateTime);  // Send offstand start timestamp

      memStd = 0;
    }
  }

  /***Standard time stoping function***/
  else if (pageNo == "31") {
    errorS = 1;
    delay(500);

    errorS = 1;
    stdCheck = 0;
    String SDstdCheck = String(stdCheck);
    writeFile(SD, "/86.txt", SDstdCheck.c_str()); /*SD Save*/


    time_t epochTime = timeClient.getEpochTime();
    struct tm *ptm = gmtime((time_t *)&epochTime);
    int monthDay = ptm->tm_mday;
    int currentMonth = ptm->tm_mon + 1;
    int currentYear = ptm->tm_year + 1900;
    String currentDate = String(currentYear) + "-" + String(currentMonth) + "-" + String(monthDay);
    String Time = String(timeClient.getFormattedTime());
    String dateTime = String(currentDate) + " " + String(Time);

    send_OffstandEndTS(OperatorID, dateTime);  // Send offstand end timestamp

    pageShift1();
    pageShift1();
  }

  /**** Help function page ****/
  else if (pageNo == "3") {
    errorS = 1;
    X1 = 0;
    Y1 = 0;
    Z1 = 0;
  }

  /**** Auto Mode ****/
  else if (pageNo == "41") {
    errorS = 1;
    delay(200);

    errorS = 1;
    autoModeButton = 1;
    Serial.println("autoModeButton = ");
    Serial.println(autoModeButton);

    pageShift14();
  }

  /**** Manual Mode ****/
  else if (pageNo == "42") {
    errorS = 1;
    delay(200);

    errorS = 1;
    manualModeButton = 1;
    Serial.println("manualModeButton = ");
    Serial.println(manualModeButton);

    pageShift14();
  }


  /****  Start lunch break time ****/
  else if (pageNo == "15") {

    errorS = 1;

    if (memLun == 1) {
      errorS = 1;
      breakCount++;
      String SDbreak = String(breakCount);
      writeFile(SD, "/34.txt", SDbreak.c_str()); /*SD Save*/

      BreakCheck = 1;
      String SDbreakCheck = String(BreakCheck);
      writeFile(SD, "/35.txt", SDbreakCheck.c_str()); /*SD Save*/
      Serial.print("break check = ");
      Serial.println(BreakCheck);


      time_t epochTime = timeClient.getEpochTime();
      struct tm *ptm = gmtime((time_t *)&epochTime);
      int monthDay = ptm->tm_mday;
      int currentMonth = ptm->tm_mon + 1;
      int currentYear = ptm->tm_year + 1900;
      String currentDate = String(currentYear) + "-" + String(currentMonth) + "-" + String(monthDay);
      String Time = String(timeClient.getFormattedTime());
      String dateTime = String(currentDate) + " " + String(Time);

      send_LunchBreakStartTS(OperatorID, dateTime);  // Send lunch break start timestamp

      writeFile(SD, "/36.txt", Time.c_str()); /*SD Save*/

      String bStartTime = "breakStartTime";

      operatorBreakStart = 0;
      operatorBreakStart = millis();

      memLun = 0;
    }
  }

  /****  End lunch break time ****/
  else if (pageNo == "30") {
    errorS = 1;
    delay(500);

    errorS = 1;
    BreakCheck = 0;
    String SDbreakCheck = String(BreakCheck);
    writeFile(SD, "/35.txt", SDbreakCheck.c_str()); /*SD Save*/
    Serial.print("break check = ");
    Serial.println(BreakCheck);


    time_t epochTime = timeClient.getEpochTime();
    struct tm *ptm = gmtime((time_t *)&epochTime);
    int monthDay = ptm->tm_mday;
    int currentMonth = ptm->tm_mon + 1;
    int currentYear = ptm->tm_year + 1900;
    String currentDate = String(currentYear) + "-" + String(currentMonth) + "-" + String(monthDay);
    String Time = String(timeClient.getFormattedTime());
    String dateTime = String(currentDate) + " " + String(Time);

    send_LunchBreakEndTS(OperatorID, dateTime);  // Send lunch break end timestamp

    String bStopTime = "breakStopTime";

    operatorBreakEnd = 0;
    operatorBreakEnd = millis();

    oper_diff = operatorBreakEnd - operatorBreakStart;
    Diff2 = oper_diff;

    if (BreakOVER == 1) {
      errorS = 1;
      time_t TIME2 = timeStringToSeconds(timeStr3);
      oper_diff = operatorBreakEnd - TIME2;
      Diff2 = oper_diff;
      BreakOVER = 0;
    }
    pageShift1();
    pageShift1();
  }

  /**** Tryagain cancel button ****/
  else if (pageNo == "37") {
    if (RFIDpageNo == "1") {  // Operator login
      pageShift7();           // Page 01 (Operaor login page)
      RFIDpageNo = "";
      M = 0;
    }

    else if (RFIDpageNo == "9") {  // Operator logout
      pageShift12();               // Page 13 (Operator Logout)
      RFIDpageNo = "";
      M = 0;
    }

    else if (RFIDpageNo == "5") {  // Mechanic login
      pageShift10();               // Page 9 (Mechanic Login)
      RFIDpageNo = "";
      M = 0;
    }

    else if (RFIDpageNo == "6") {  //  Mechanic logout
      pageShift20();               // Page 06 (Mechanic logout)
      RFIDpageNo = "";
      M = 0;
    }

    else if (RFIDpageNo == "7") {  // Supervisor login
      pageShift11();               // Page 11 (Supervisor Login)
      RFIDpageNo = "";
      M = 0;
    }

    else if (RFIDpageNo == "8") {  // Supervisor logout
      pageShift19();               // Page 08 (Supervisor logout)
      RFIDpageNo = "";
      M = 0;
    }

    else if (RFIDpageNo == "10") {  // Supervisor mode confirm
      pageShift14();                // Page 10 (MODE confirm page)
      RFIDpageNo = "";
      M = 0;
    }

  }

  /**** Change operation ****/
  else if (pageNo == "38") {

    M_mem = 0;
    S_mem = 0;

    deleteFile(SD, "/1.txt");
    deleteFile(SD, "/2.txt");
    deleteFile(SD, "/3.txt");
    deleteFile(SD, "/4.txt");
    deleteFile(SD, "/5.txt");
    deleteFile(SD, "/6.txt");
    deleteFile(SD, "/7.txt");
    deleteFile(SD, "/8.txt");
    deleteFile(SD, "/9.txt");
    deleteFile(SD, "/10.txt");
    deleteFile(SD, "/11.txt");
    deleteFile(SD, "/12.txt");
    deleteFile(SD, "/13.txt");
    deleteFile(SD, "/14.txt");
    deleteFile(SD, "/15.txt");
    deleteFile(SD, "/16.txt");
    deleteFile(SD, "/17.txt");
    deleteFile(SD, "/18.txt");
    deleteFile(SD, "/19.txt");
    deleteFile(SD, "/20.txt");
    deleteFile(SD, "/35.txt");
    deleteFile(SD, "/36.txt");
    deleteFile(SD, "/37.txt");
    deleteFile(SD, "/40.txt");
    //deleteFile(SD, "/41.txt");
    deleteFile(SD, "/42.txt");
    deleteFile(SD, "/43.txt");
    deleteFile(SD, "/44.txt");
    deleteFile(SD, "/45.txt");
    deleteFile(SD, "/46.txt");
    deleteFile(SD, "/48.txt");
    deleteFile(SD, "/49.txt");
    deleteFile(SD, "/50.txt");
    deleteFile(SD, "/51.txt");
    //deleteFile(SD, "/52.txt");
    deleteFile(SD, "/53.txt");
    deleteFile(SD, "/54.txt");
    deleteFile(SD, "/55.txt");
    deleteFile(SD, "/56.txt");
    deleteFile(SD, "/57.txt");
    deleteFile(SD, "/58.txt");
    deleteFile(SD, "/59.txt");
    deleteFile(SD, "/60.txt");
    deleteFile(SD, "/61.txt");
    deleteFile(SD, "/62.txt");
    deleteFile(SD, "/67.txt");
    deleteFile(SD, "/68.txt");
    deleteFile(SD, "/68.txt");
    deleteFile(SD, "/69.txt");
    deleteFile(SD, "/70.txt");
    deleteFile(SD, "/72.txt");
    deleteFile(SD, "/73.txt");
    deleteFile(SD, "/74.txt");
    //deleteFile(SD, "/75.txt");
    //deleteFile(SD, "/76.txt");
    //deleteFile(SD, "/77.txt");
    //deleteFile(SD, "/78.txt");
    //deleteFile(SD, "/79.txt");
    //deleteFile(SD, "/80.txt");
    //deleteFile(SD, "/81.txt");
    //deleteFile(SD, "/83.txt");
    //deleteFile(SD, "/84.txt");
    deleteFile(SD, "/86.txt");

    opChangeM = 1;
    String opChange_M = String(opChangeM);
    writeFile(SD, "/71.txt", opChange_M.c_str()); /*SD Save*/

    leds[0] = CRGB(0, 0, 0);
    FastLED.show();

    leds[1] = CRGB(0, 0, 0);
    FastLED.show();

    leds[2] = CRGB(0, 0, 0);
    FastLED.show();

    leds[3] = CRGB(0, 0, 0);
    FastLED.show();



    uint8_t data1[] = { 0x5A, 0xA5, 0x05, 0x82, 0x17, 0x00, 0x00, 0x00 };  // Supervisor green color icon change
    for (size_t i = 0; i < 8; i++) {
      Serial2.write(data1[i]);
    }

    uint8_t data2[] = { 0x5A, 0xA5, 0x05, 0x82, 0x20, 0x00, 0x00, 0x00 };  // Mechanic green color icon change
    for (size_t i = 0; i < 8; i++) {
      Serial2.write(data2[i]);
    }

    uint8_t data3[] = { 0x5A, 0xA5, 0x05, 0x82, 0x18, 0x00, 0x00, 0x00 };  // Efficiency meter
    for (size_t i = 0; i < 8; i++) {
      Serial2.write(data3[i]);
    }
    String NEW_Val = "0";
    send_text4(NEW_Val);

    pageShift2();
    pageShift2();

    ESP.restart();
  }


  /**** Change operator ****/
  else if (pageNo == "40") {
    errorS = 1;
    changeOP = 1;
    String cOperaor = String(changeOP);
    writeFile(SD, "/21.txt", cOperaor.c_str()); /*SD Save*/

    DHUfunction();

    deleteFile(SD, "/23.txt");
    deleteFile(SD, "/22.txt");
    deleteFile(SD, "/24.txt");
    deleteFile(SD, "/63.txt");

    pageShift7();
    pageShift7();
  }


  /**** shutdown function ****/
  else if (pageNo == "39") {

    leds[0] = CRGB(0, 0, 0);
    FastLED.show();

    leds[1] = CRGB(0, 0, 0);
    FastLED.show();

    leds[2] = CRGB(0, 0, 0);
    FastLED.show();

    leds[3] = CRGB(0, 0, 0);
    FastLED.show();

    if (xSemaphoreTake(fileAccessSemaphore, portMAX_DELAY)) {  // Wait for the lock
      if (SPIFFS.exists("/mqttLog.txt")) {                     // Check if the file exists
        SPIFFS.remove("/mqttLog.txt");
        Serial.println("All lines published. mqttLog.txt deleted.");
      }
      if (SPIFFS.exists("/111.txt")) {  // Check if the file exists
        SPIFFS.remove("/111.txt");
      }
      if (SPIFFS.exists("/112.txt")) {  // Check if the file exists
        SPIFFS.remove("/112.txt");
      }
      xSemaphoreGive(fileAccessSemaphore);  // Release the lock
    }

    deleteFile(SD, "/1.txt");
    deleteFile(SD, "/2.txt");
    deleteFile(SD, "/3.txt");
    deleteFile(SD, "/4.txt");
    deleteFile(SD, "/5.txt");
    deleteFile(SD, "/6.txt");
    deleteFile(SD, "/7.txt");
    deleteFile(SD, "/8.txt");
    deleteFile(SD, "/9.txt");
    deleteFile(SD, "/10.txt");
    deleteFile(SD, "/11.txt");
    deleteFile(SD, "/12.txt");
    deleteFile(SD, "/13.txt");
    deleteFile(SD, "/14.txt");
    deleteFile(SD, "/15.txt");
    deleteFile(SD, "/16.txt");
    deleteFile(SD, "/17.txt");
    deleteFile(SD, "/18.txt");
    deleteFile(SD, "/19.txt");
    deleteFile(SD, "/20.txt");
    deleteFile(SD, "/21.txt");
    deleteFile(SD, "/23.txt");
    deleteFile(SD, "/24.txt");
    deleteFile(SD, "/25.txt");
    deleteFile(SD, "/26.txt");
    deleteFile(SD, "/27.txt");
    deleteFile(SD, "/28.txt");
    deleteFile(SD, "/29.txt");
    deleteFile(SD, "/30.txt");
    deleteFile(SD, "/31.txt");
    deleteFile(SD, "/32.txt");
    deleteFile(SD, "/33.txt");
    deleteFile(SD, "/34.txt");
    deleteFile(SD, "/35.txt");
    deleteFile(SD, "/36.txt");
    deleteFile(SD, "/37.txt");
    deleteFile(SD, "/38.txt");
    deleteFile(SD, "/39.txt");
    deleteFile(SD, "/41.txt");
    deleteFile(SD, "/43.txt");
    deleteFile(SD, "/45.txt");
    deleteFile(SD, "/46.txt");
    deleteFile(SD, "/47.txt");
    deleteFile(SD, "/48.txt");
    deleteFile(SD, "/49.txt");
    deleteFile(SD, "/50.txt");
    deleteFile(SD, "/51.txt");
    deleteFile(SD, "/52.txt");
    deleteFile(SD, "/53.txt");
    deleteFile(SD, "/54.txt");
    deleteFile(SD, "/55.txt");
    deleteFile(SD, "/56.txt");
    deleteFile(SD, "/57.txt");
    deleteFile(SD, "/58.txt");
    deleteFile(SD, "/59.txt");
    deleteFile(SD, "/60.txt");
    deleteFile(SD, "/61.txt");
    deleteFile(SD, "/62.txt");
    deleteFile(SD, "/63.txt");
    deleteFile(SD, "/64.txt");
    deleteFile(SD, "/65.txt");
    deleteFile(SD, "/66.txt");
    deleteFile(SD, "/67.txt");
    deleteFile(SD, "/68.txt");
    deleteFile(SD, "/70.txt");
    deleteFile(SD, "/71.txt");
    deleteFile(SD, "/72.txt");
    deleteFile(SD, "/73.txt");
    deleteFile(SD, "/74.txt");
    deleteFile(SD, "/75.txt");
    deleteFile(SD, "/77.txt");
    deleteFile(SD, "/78.txt");
    deleteFile(SD, "/79.txt");
    deleteFile(SD, "/80.txt");
    deleteFile(SD, "/81.txt");
    deleteFile(SD, "/83.txt");
    deleteFile(SD, "/84.txt");
    deleteFile(SD, "/86.txt");

    leds[0] = CRGB(0, 0, 0);
    FastLED.show();

    leds[1] = CRGB(0, 0, 0);
    FastLED.show();

    leds[2] = CRGB(0, 0, 0);
    FastLED.show();

    leds[3] = CRGB(0, 0, 0);
    FastLED.show();

    M_mem = 0;
    S_mem = 0;

    pageShift16();
    pageShift16();
  }


  else if (errorS == 0) {

    /**** Network settingup page when operator change ****/
    if ((pageNo == "0") && (changeOP == 1)) {
      Serial.println("/**** Change operator start ****/");


      readFile(SD, "/1.txt"); /*SD Read*/
      smv = rdline.toFloat();
      String smv_SD = String(smv);
      send_text9(smv_SD);
      send_text9(smv_SD);

      readFile(SD, "/2.txt"); /*SD Read*/
      spi = rdline.toInt();
      String spi_SD = String(spi);
      send_text10(spi_SD);
      send_text10(spi_SD);

      readFile(SD, "/3.txt"); /*SD Read*/
      workingHr = rdline.toInt();

      readFile(SD, "/51.txt"); /*SD Read*/
      DAILY = rdline.toInt();

      readFile(SD, "/4.txt"); /*SD Read*/
      E_RGB_LOW = rdline.toInt();

      readFile(SD, "/5.txt"); /*SD Read*/
      E_RGB_HIGH = rdline.toInt();

      readFile(SD, "/6.txt"); /*SD Read*/
      target = rdline.toInt();
      Tar = target;
      send_text3(rdline);
      send_text3(rdline);

      readFile(SD, "/7.txt"); /*SD Read*/
      machineId = rdline;
      send_text20(machineId);

      readFile(SD, "/8.txt"); /*SD Read*/
      machineType = rdline;
      send_text7(machineType);
      send_text7(machineType);

      readFile(SD, "/9.txt"); /*SD Read*/
      ownership = rdline;
      send_text8(ownership);
      send_text8(ownership);

      readFile(SD, "/10.txt"); /*SD Read*/
      obbOperationId = rdline;

      readFile(SD, "/11.txt"); /*SD Read*/
      style = rdline;
      send_text14(style);
      send_text14(style);

      readFile(SD, "/12.txt"); /*SD Read*/
      supervisorName = rdline;
      send_text17(supervisorName);
      send_text17(supervisorName);

      readFile(SD, "/13.txt"); /*SD Read*/
      supervisorRFID = rdline;

      readFile(SD, "/14.txt"); /*SD Read*/
      mechanicName = rdline;
      send_text16(mechanicName);
      send_text16(mechanicName);

      readFile(SD, "/15.txt"); /*SD Read*/
      mechanicRFID = rdline;

      readFile(SD, "/16.txt"); /*SD Read*/
      MeEmployeeId = rdline;

      readFile(SD, "/17.txt"); /*SD Read*/
      SuEmployeeId = rdline;

      readFile(SD, "/18.txt"); /*SD Read*/
      qualityInsName = rdline;
      send_text18(qualityInsName);
      send_text18(qualityInsName);

      readFile(SD, "/19.txt"); /*SD Read*/
      operationName = rdline;
      send_text15(operationName);

      readFile(SD, "/20.txt"); /*SD Read*/
      productionLine = rdline;
      send_text13(productionLine);

      send_text19(deviceId);
      send_text19(deviceId);

      readFile(SD, "/54.txt"); /*SD Read*/
      t = rdline.toInt();

      readFile(SD, "/55.txt"); /*SD Read*/
      AEM = rdline.toInt();

      readFile(SD, "/52.txt");
      G = rdline.toInt();

      formattedCount = String(G);
      while (formattedCount.length() < 6) {
        formattedCount = "0" + formattedCount;  // Prepend zeros until the length is 4
      }
      send_text27(formattedCount);
      send_text27(formattedCount);

      readFile(SD, "/53.txt");
      LG = rdline.toInt();

      readFile(SD, "/68.txt");
      RG = rdline.toInt();

      readFile(SD, "/49.txt");
      N = rdline.toInt();
      send_text4(rdline);
      send_text4(rdline);
      if (N == 0) {
        String Nstr = "0";
        send_text4(Nstr);
        send_text4(Nstr);
      }

      //readFile(SD, "/45.txt");
      //CON = rdline.toInt();

      readFile(SD, "/46.txt");
      J = rdline.toInt();

      readFile(SD, "/47.txt");
      L = rdline.toInt();

      delay(50);
      Lvalue = (L + 1);

      String Ldata = String(Lvalue);
      send_text23(Ldata);  //L+1
      send_text23(Ldata);
      send_text23(Ldata);


      readFile(SD, "/48.txt");
      proTar = rdline.toInt();

      readFile(SD, "/42.txt"); /*SD Read*/
      trainingCountDown = rdline.toInt();

      readFile(SD, "/60.txt");
      EFF_RGB_MEM = rdline.toInt();

      if (ReadDone == 0) {
        EFF_RGB_MEM = 0;
      }

      if (EFF_RGB_MEM == 1) {
        readFile(SD, "/54.txt");
        t = rdline.toInt();
        meter2();
      }

      if (EFF_RGB_MEM == 0) {
        meter2();
      }

      readFile(SD, "/34.txt");
      breakCount = rdline.toInt();
      if (ReadDone == 0) {
        breakCount = 0;
      }

      readFile(SD, "/30.txt");
      mCallCount = rdline.toInt();
      if (ReadDone == 0) {
        mCallCount = 0;
      }

      readFile(SD, "/28.txt");
      mR = rdline.toInt();
      if (ReadDone == 0) {
        mR = 0;
      }

      readFile(SD, "/32.txt");
      sCallCount = rdline.toInt();
      if (ReadDone == 0) {
        sCallCount = 0;
      }

      readFile(SD, "/25.txt");
      sR = rdline.toInt();
      if (ReadDone == 0) {
        sR = 0;
      }

      readFile(SD, "/26.txt");
      S_mem = rdline.toInt();
      if (ReadDone == 0) {
        S_mem = 0;
      }

      readFile(SD, "/29.txt");
      M_mem = rdline.toInt();
      if (ReadDone == 0) {
        M_mem = 0;
      }

      if (sR == 0) {
        uint8_t data1[] = { 0x5A, 0xA5, 0x05, 0x82, 0x17, 0x00, 0x00, 0x00 };
        for (size_t i = 0; i < 8; i++) {
          Serial2.write(data1[i]);
        }
      }

      if (sR == 1) {
        uint8_t data1[] = { 0x5A, 0xA5, 0x05, 0x82, 0x17, 0x00, 0x00, 0x01 };  // Supervisor icon change to green color
        for (size_t i = 0; i < 8; i++) {
          Serial2.write(data1[i]);
        }
      }

      if (mR == 0) {
        uint8_t data1[] = { 0x5A, 0xA5, 0x05, 0x82, 0x20, 0x00, 0x00, 0x00 };
        for (size_t i = 0; i < 8; i++) {
          Serial2.write(data1[i]);
        }
      }

      if (mR == 1) {
        uint8_t data1[] = { 0x5A, 0xA5, 0x05, 0x82, 0x20, 0x00, 0x00, 0x01 };
        for (size_t i = 0; i < 8; i++) {
          Serial2.write(data1[i]);
        }
      }

      readFile(SD, "/62.txt");
      tt = rdline.toInt();

      readFile(SD, "/61.txt");
      AEMM = rdline.toInt();


      readFile(SD, "/31.txt"); /*SD Read*/
      TimeME = rdline;

      readFile(SD, "/33.txt"); /*SD Read*/
      TimeSU = rdline;

      readFile(SD, "/38.txt"); /*SD Read*/
      RoundTLS = rdline.toInt();

      readFile(SD, "/40.txt"); /*SD Read*/
      buttonPressCount = rdline.toInt();
      if ((ReadDone == 0) || (buttonPressCount == 1)) {
        buttonPressCount = 0;
      }

      readFile(SD, "/41.txt"); /*SD Read*/
      pieceCount = rdline.toInt();

      readFile(SD, "/27.txt"); /*SD Read*/
      autoCountMode = rdline.toInt() != 0;

      if (autoCountMode == 1) {
        String autoM = "A";
        send_text6(autoM);
        send_text6(autoM);
      }

      if (autoCountMode == 0) {
        String manualM = "M";
        send_text6(manualM);
        send_text6(manualM);
      }

      // readFile(SD, "/43.txt"); /*SD Read*/
      // accumulatedArea = rdline.toDouble();

      readFile(SD, "/44.txt"); /*SD Read*/
      trainedArea = rdline.toFloat();

      readFile(SD, "/69.txt");
      idleVpp = rdline.toFloat();

      readFile(SD, "/70.txt");
      Hour_Z = rdline.toInt();

      readFile(SD, "/77.txt");
      Hour_X = rdline.toInt();

      readFile(SD, "/78.txt");
      Hour_Y = rdline.toInt();

      readFile(SD, "/79.txt");
      Done45 = rdline.toInt();

      readFile(SD, "/80.txt");
      Min_X = rdline.toInt();

      readFile(SD, "/81.txt");
      Min_Y = rdline.toInt();

      readFile(SD, "/75.txt");
      smv_CAL = rdline.toInt();

      readFile(SD, "/76.txt");
      trainedSMV = rdline.toInt();

      readFile(SD, "/83.txt");
      part = rdline;
      Serial.print("part :");
      Serial.println(part);

      readFile(SD, "/84.txt");
      DHU = rdline.toFloat();

      if (ReadDone == 0) {
        DHU = 0.00;
      }
      Serial.print("DHU :");
      Serial.println(DHU);

      DHUfunction();

      readFile(SD, "/85.txt");
      areaResetTime = rdline.toInt();

      Serial.println("/**** Change operator end ****/");

      pageShift7();
      pageShift7();
    }


    else if ((pageNo == "0") && (attendanceOp == 1) && (errorS == 0) && (opChangeM == 1)) {

      Serial.println("/**** Operation change start ****/");
      unsigned long currentMillis=millis();
      if (currentMillis - previousMillis >= 3000+random(100)*10) {
      previousMillis = currentMillis;
      /** Initial get NODE RED **/
      if (fnCall_initial_data) {
        awaitReply = 1;
        restartCountDown--;
        if(restartCountDown==0)ESP.restart();
        get_initial_data(deviceId);
        
      }
    }

      if (dataReceived_initial_data) {

        J = 2;
        opChangeM = 0;
        String opChange_M = String(opChangeM);
        writeFile(SD, "/71.txt", opChange_M.c_str()); /*SD Save*/

        Serial.println("/**** Operation change start ****/");

        // To check whether the variables have assigned properly
        Serial.println("");

        leds[2] = CRGB(0, 0, 255);
        FastLED.show();

        leds[3] = CRGB(0, 0, 255);
        FastLED.show();

        opChangeM = 0;
        String opChangeMem = String(opChangeM);
        writeFile(SD, "/71.txt", opChangeMem.c_str()); /*SD Save*/

        Serial.print("obbOperationId = ");
        Serial.println(obbOperationId);
        writeFile(SD, "/10.txt", obbOperationId.c_str()); /*SD Save*/

        Serial.print("machineId = ");
        Serial.println(machineId);
        writeFile(SD, "/7.txt", machineId.c_str()); /*SD Save*/
        send_text20(machineId);

        Serial.print("machineType = ");
        Serial.println(machineType);
        writeFile(SD, "/8.txt", machineType.c_str()); /*SD Save*/
        send_text7(machineType);
        send_text7(machineType);

        Serial.print("ownership = ");
        Serial.println(ownership);
        writeFile(SD, "/9.txt", ownership.c_str()); /*SD Save*/
        send_text8(ownership);
        send_text8(ownership);

        Serial.print("smv = ");
        Serial.println(smv);
        String smv_SD = String(smv);
        writeFile(SD, "/1.txt", smv_SD.c_str()); /*SD Save*/
        send_text9(smv_SD);
        send_text9(smv_SD);

        Serial.print("spi = ");
        Serial.println(spi);
        String spi_SD = String(spi);
        writeFile(SD, "/2.txt", spi_SD.c_str()); /*SD Save*/
        send_text10(spi_SD);
        send_text10(spi_SD);

        Serial.print("target = ");
        Serial.println(target);
        Tar = target;
        HrTar = Tar;
        String target_SD = String(target);
        writeFile(SD, "/6.txt", target_SD.c_str());  /*SD Save*/
        writeFile(SD, "/50.txt", target_SD.c_str()); /*SD Save*/
        send_text3(target_SD);
        send_text3(target_SD);

        Serial.print("workingHours = ");
        Serial.println(workingHours);
        String workingHours_SD = String(workingHours);
        writeFile(SD, "/3.txt", workingHours_SD.c_str()); /*SD Save*/
        workingHr = workingHours;

        DAILY = workingHr * Tar;
        String DAILY_SD = String(DAILY);
        writeFile(SD, "/51.txt", DAILY_SD.c_str()); /*SD Save*/


        Serial.print("efficiencyLevel1 = ");
        Serial.println(efficiencyLevel1);
        E_RGB_LOW = efficiencyLevel1;
        String efficiencyLevel1_SD = String(efficiencyLevel1);
        writeFile(SD, "/4.txt", efficiencyLevel1_SD.c_str()); /*SD Save*/

        Serial.print("efficiencyLevel3 = ");
        Serial.println(efficiencyLevel3);
        E_RGB_HIGH = efficiencyLevel3;
        String efficiencyLevel3_SD = String(efficiencyLevel3);
        writeFile(SD, "/5.txt", efficiencyLevel3_SD.c_str()); /*SD Save*/

        Serial.print("style = ");
        Serial.println(style);
        writeFile(SD, "/11.txt", style.c_str()); /*SD Save*/
        send_text14(style);
        send_text14(style);

        Serial.print("operationName = ");
        Serial.println(operationName);
        writeFile(SD, "/19.txt", operationName.c_str()); /*SD Save*/
        send_text15(operationName);

        Serial.print("SuEmployeeId = ");
        Serial.println(SuEmployeeId);
        writeFile(SD, "/17.txt", SuEmployeeId.c_str()); /*SD Save*/

        Serial.print("supervisorRFID = ");
        Serial.println(supervisorRFID);
        writeFile(SD, "/13.txt", supervisorRFID.c_str()); /*SD Save*/

        Serial.print("supervisorName = ");
        Serial.println(supervisorName);
        writeFile(SD, "/12.txt", supervisorName.c_str()); /*SD Save*/
        send_text17(supervisorName);
        send_text17(supervisorName);

        Serial.print("mechanicName = ");
        Serial.println(mechanicName);
        writeFile(SD, "/14.txt", mechanicName.c_str()); /*SD Save*/
        send_text16(mechanicName);
        send_text16(mechanicName);

        Serial.print("MeEmployeeId = ");
        Serial.println(MeEmployeeId);
        writeFile(SD, "/16.txt", MeEmployeeId.c_str()); /*SD Save*/

        Serial.print("mechanicRFID = ");
        Serial.println(mechanicRFID);
        writeFile(SD, "/15.txt", mechanicRFID.c_str()); /*SD Save*/

        Serial.print("qualityInsName = ");
        Serial.println(qualityInsName);
        writeFile(SD, "/18.txt", qualityInsName.c_str()); /*SD Save*/
        send_text18(qualityInsName);
        send_text18(qualityInsName);

        Serial.print("productionLine = ");
        Serial.println(productionLine);
        writeFile(SD, "/20.txt", productionLine.c_str()); /*SD Save*/
        send_text13(productionLine);

        readFile(SD, "/24.txt"); /*SD Read*/
        OperatorID = rdline;
        send_text1(OperatorID);
        send_text1(OperatorID);

        send_text19(deviceId);
        send_text19(deviceId);


        readFile(SD, "/47.txt");
        L = rdline.toInt();

        delay(50);
        Lvalue = (L + 1);
        String Ldata = String(Lvalue);
        send_text23(Ldata);  //L+1
        send_text23(Ldata);
        send_text23(Ldata);


        readFile(SD, "/34.txt");
        breakCount = rdline.toInt();
        if (ReadDone == 0) {
          breakCount = 0;
        }

        readFile(SD, "/30.txt");
        mCallCount = rdline.toInt();
        if (ReadDone == 0) {
          mCallCount = 0;
        }

        readFile(SD, "/28.txt");
        mR = rdline.toInt();
        if (ReadDone == 0) {
          mR = 0;
        }

        readFile(SD, "/32.txt");
        sCallCount = rdline.toInt();
        if (ReadDone == 0) {
          sCallCount = 0;
        }

        readFile(SD, "/25.txt");
        sR = rdline.toInt();
        if (ReadDone == 0) {
          sR = 0;
        }

        readFile(SD, "/26.txt");
        S_mem = rdline.toInt();
        if (ReadDone == 0) {
          S_mem = 0;
        }

        readFile(SD, "/29.txt");
        M_mem = rdline.toInt();
        if (ReadDone == 0) {
          M_mem = 0;
        }

        if (sR == 0) {
          uint8_t data1[] = { 0x5A, 0xA5, 0x05, 0x82, 0x17, 0x00, 0x00, 0x00 };
          for (size_t i = 0; i < 8; i++) {
            Serial2.write(data1[i]);
          }
        }

        if (sR == 1) {
          uint8_t data1[] = { 0x5A, 0xA5, 0x05, 0x82, 0x17, 0x00, 0x00, 0x01 };  // Supervisor icon change to green color
          for (size_t i = 0; i < 8; i++) {
            Serial2.write(data1[i]);
          }
        }

        if (mR == 0) {
          uint8_t data1[] = { 0x5A, 0xA5, 0x05, 0x82, 0x20, 0x00, 0x00, 0x00 };
          for (size_t i = 0; i < 8; i++) {
            Serial2.write(data1[i]);
          }
        }

        if (mR == 1) {
          uint8_t data1[] = { 0x5A, 0xA5, 0x05, 0x82, 0x20, 0x00, 0x00, 0x01 };
          for (size_t i = 0; i < 8; i++) {
            Serial2.write(data1[i]);
          }
        }

        readFile(SD, "/31.txt"); /*SD Read*/
        TimeME = rdline;

        readFile(SD, "/33.txt"); /*SD Read*/
        TimeSU = rdline;

        readFile(SD, "/66.txt"); /*SD Read*/
        errorL = rdline.toInt();

        readFile(SD, "/38.txt"); /*SD Read*/
        RoundTLS = rdline.toInt();

        if (errorL == 1) {
          RoundTLS = RoundTLS - 1;
        }

        readFile(SD, "/23.txt");  // Login time
        timeStr1 = rdline;

        readFile(SD, "/63.txt");
        sessionId_O = rdline;

        readFile(SD, "/64.txt");
        sessionId_S = rdline;

        readFile(SD, "/65.txt");
        sessionId_M = rdline;

        readFile(SD, "/27.txt"); /*SD Read*/
        autoCountMode = rdline.toInt() != 0;

        if (autoCountMode == 1) {
          String autoM = "A";
          send_text6(autoM);
          send_text6(autoM);
        }

        if (autoCountMode == 0) {
          String manualM = "M";
          send_text6(manualM);
          send_text6(manualM);
        }

        readFile(SD, "/70.txt");
        Hour_Z = rdline.toInt();

        readFile(SD, "/77.txt");
        Hour_X = rdline.toInt();

        readFile(SD, "/78.txt");
        Hour_Y = rdline.toInt();

        readFile(SD, "/79.txt");
        Done45 = rdline.toInt();

        readFile(SD, "/80.txt");
        Min_X = rdline.toInt();

        readFile(SD, "/81.txt");
        Min_Y = rdline.toInt();

        readFile(SD, "/75.txt");
        smv_CAL = rdline.toInt();

        readFile(SD, "/76.txt");
        trainedSMV = rdline.toInt();

        readFile(SD, "/52.txt");
        G = rdline.toInt();

        readFile(SD, "/41.txt"); /*SD Read*/
        pieceCount = rdline.toInt();

        readFile(SD, "/83.txt");
        part = rdline;
        Serial.print("part :");
        Serial.println(part);

        readFile(SD, "/84.txt");
        DHU = rdline.toFloat();
        Serial.print("DHU :");
        Serial.println(DHU);

        if (ReadDone == 0) {
          DHU = 0.00;
        }

        DHUfunction();

        pageShift1();
        pageShift1();

        dataReceived_initial_data = false;
      }
    }


    else if ((pageNo == "0") && (attendanceOp == 1) && (errorS == 0) && (stdCheck == 1)) {
      Serial.println("/**** Operator after off standard start ****/");



      delay(300);
      NO_Power = 1;
      delay(300);
      memStd = 0;

      readFile(SD, "/1.txt"); /*SD Read*/
      smv = rdline.toFloat();
      String smv_SD = String(smv);
      send_text9(smv_SD);
      send_text9(smv_SD);

      readFile(SD, "/2.txt"); /*SD Read*/
      spi = rdline.toInt();
      String spi_SD = String(spi);
      send_text10(spi_SD);
      send_text10(spi_SD);

      readFile(SD, "/3.txt"); /*SD Read*/
      workingHr = rdline.toInt();

      readFile(SD, "/51.txt"); /*SD Read*/
      DAILY = rdline.toInt();

      readFile(SD, "/4.txt"); /*SD Read*/
      E_RGB_LOW = rdline.toInt();

      readFile(SD, "/5.txt"); /*SD Read*/
      E_RGB_HIGH = rdline.toInt();

      readFile(SD, "/6.txt"); /*SD Read*/
      target = rdline.toInt();
      Tar = target;
      send_text3(rdline);
      send_text3(rdline);

      readFile(SD, "/7.txt"); /*SD Read*/
      machineId = rdline;
      send_text20(machineId);

      readFile(SD, "/8.txt"); /*SD Read*/
      machineType = rdline;
      send_text7(machineType);
      send_text7(machineType);

      readFile(SD, "/9.txt"); /*SD Read*/
      ownership = rdline;
      send_text8(ownership);
      send_text8(ownership);

      readFile(SD, "/10.txt"); /*SD Read*/
      obbOperationId = rdline;

      readFile(SD, "/11.txt"); /*SD Read*/
      style = rdline;
      send_text14(style);
      send_text14(style);

      readFile(SD, "/12.txt"); /*SD Read*/
      supervisorName = rdline;
      send_text17(supervisorName);
      send_text17(supervisorName);

      readFile(SD, "/13.txt"); /*SD Read*/
      supervisorRFID = rdline;

      readFile(SD, "/14.txt"); /*SD Read*/
      mechanicName = rdline;
      send_text16(mechanicName);
      send_text16(mechanicName);

      readFile(SD, "/15.txt"); /*SD Read*/
      mechanicRFID = rdline;

      readFile(SD, "/16.txt"); /*SD Read*/
      MeEmployeeId = rdline;

      readFile(SD, "/17.txt"); /*SD Read*/
      SuEmployeeId = rdline;

      readFile(SD, "/18.txt"); /*SD Read*/
      qualityInsName = rdline;
      send_text18(qualityInsName);
      send_text18(qualityInsName);

      readFile(SD, "/19.txt"); /*SD Read*/
      operationName = rdline;
      send_text15(operationName);

      readFile(SD, "/20.txt"); /*SD Read*/
      productionLine = rdline;
      send_text13(productionLine);
      send_text13(productionLine);

      readFile(SD, "/24.txt"); /*SD Read*/
      OperatorID = rdline;
      send_text1(OperatorID);
      send_text1(OperatorID);

      send_text19(deviceId);
      send_text19(deviceId);

      readFile(SD, "/54.txt"); /*SD Read*/
      t = rdline.toInt();

      readFile(SD, "/55.txt"); /*SD Read*/
      AEM = rdline.toInt();

      readFile(SD, "/52.txt");
      G = rdline.toInt();

      formattedCount = String(G);
      while (formattedCount.length() < 6) {
        formattedCount = "0" + formattedCount;  // Prepend zeros until the length is 4
      }
      send_text27(formattedCount);
      send_text27(formattedCount);

      readFile(SD, "/53.txt");
      LG = rdline.toInt();

      readFile(SD, "/68.txt");
      RG = rdline.toInt();

      readFile(SD, "/49.txt");
      N = rdline.toInt();
      send_text4(rdline);
      send_text4(rdline);
      if (N == 0) {
        String Nstr = "0";
        send_text4(Nstr);
        send_text4(Nstr);
      }

      //readFile(SD, "/45.txt");
      //CON = rdline.toInt();
      //
      readFile(SD, "/46.txt");
      J = rdline.toInt();

      readFile(SD, "/47.txt");
      L = rdline.toInt();

      delay(50);
      Lvalue = (L + 1);
      String Ldata = String(Lvalue);
      send_text23(Ldata);  //L+1
      send_text23(Ldata);
      send_text23(Ldata);

      readFile(SD, "/48.txt");
      proTar = rdline.toInt();

      //readFile(SD, "/50.txt");
      //HrTar = rdline.toInt();
      //send_text3(rdline);
      //send_text3(rdline);

      readFile(SD, "/42.txt"); /*SD Read*/
      trainingCountDown = rdline.toInt();

      readFile(SD, "/60.txt");
      EFF_RGB_MEM = rdline.toInt();

      if (ReadDone == 0) {
        EFF_RGB_MEM = 0;
      }

      if (EFF_RGB_MEM == 1) {
        readFile(SD, "/54.txt");
        t = rdline.toInt();
        meter2();
      }

      if (EFF_RGB_MEM == 0) {
        meter2();
      }

      readFile(SD, "/34.txt");
      breakCount = rdline.toInt();
      if (ReadDone == 0) {
        breakCount = 0;
      }

      readFile(SD, "/30.txt");
      mCallCount = rdline.toInt();
      if (ReadDone == 0) {
        mCallCount = 0;
      }

      readFile(SD, "/28.txt");
      mR = rdline.toInt();
      if (ReadDone == 0) {
        mR = 0;
      }

      readFile(SD, "/32.txt");
      sCallCount = rdline.toInt();
      if (ReadDone == 0) {
        sCallCount = 0;
      }

      readFile(SD, "/25.txt");
      sR = rdline.toInt();
      if (ReadDone == 0) {
        sR = 0;
      }

      readFile(SD, "/26.txt");
      S_mem = rdline.toInt();
      if (ReadDone == 0) {
        S_mem = 0;
      }

      readFile(SD, "/29.txt");
      M_mem = rdline.toInt();
      if (ReadDone == 0) {
        M_mem = 0;
      }

      if (sR == 0) {
        uint8_t data1[] = { 0x5A, 0xA5, 0x05, 0x82, 0x17, 0x00, 0x00, 0x00 };
        for (size_t i = 0; i < 8; i++) {
          Serial2.write(data1[i]);
        }
      }

      if (sR == 1) {
        uint8_t data1[] = { 0x5A, 0xA5, 0x05, 0x82, 0x17, 0x00, 0x00, 0x01 };  // Supervisor icon change to green color
        for (size_t i = 0; i < 8; i++) {
          Serial2.write(data1[i]);
        }
      }

      if (mR == 0) {
        uint8_t data1[] = { 0x5A, 0xA5, 0x05, 0x82, 0x20, 0x00, 0x00, 0x00 };
        for (size_t i = 0; i < 8; i++) {
          Serial2.write(data1[i]);
        }
      }

      if (mR == 1) {
        uint8_t data1[] = { 0x5A, 0xA5, 0x05, 0x82, 0x20, 0x00, 0x00, 0x01 };
        for (size_t i = 0; i < 8; i++) {
          Serial2.write(data1[i]);
        }
      }

      readFile(SD, "/62.txt");
      tt = rdline.toInt();

      readFile(SD, "/61.txt");
      AEMM = rdline.toInt();

      readFile(SD, "/31.txt"); /*SD Read*/
      TimeME = rdline;

      readFile(SD, "/33.txt"); /*SD Read*/
      TimeSU = rdline;

      readFile(SD, "/38.txt"); /*SD Read*/
      RoundTLS = rdline.toInt();

      readFile(SD, "/23.txt");  // Login time
      timeStr1 = rdline;

      readFile(SD, "/59.txt");  // Efficiency time
      timeStr2 = rdline;

      readFile(SD, "/36.txt");  // Break start time
      timeStr3 = rdline;

      if (L == 0) {
        time_t TIME1 = timeStringToSeconds(timeStr1);  // Login time

        time_t TIMEst = timeStringToSeconds(timeStr3);  // Break start time

        time_t timeDifference = (TIMEst - TIME1);
        SDdiffSecond = timeDifference;
        Serial.println("SDdiffSecond1");
        Serial.print(SDdiffSecond);
      }

      if (L >= 1) {
        time_t TIME3 = timeStringToSeconds(timeStr2);  // Efficiency time

        time_t TIMEst = timeStringToSeconds(timeStr3);  // Break start time

        time_t timeDifference = (TIMEst - TIME3);
        SDdiffSecond = timeDifference;
        Serial.println("SDdiffSecond2");
        Serial.println(SDdiffSecond);
      }

      readFile(SD, "/63.txt");
      sessionId_O = rdline;

      readFile(SD, "/64.txt");
      sessionId_S = rdline;

      readFile(SD, "/65.txt");
      sessionId_M = rdline;

      readFile(SD, "/40.txt"); /*SD Read*/
      buttonPressCount = rdline.toInt();
      if ((ReadDone == 0) || (buttonPressCount == 1)) {
        buttonPressCount = 0;
      }

      readFile(SD, "/41.txt"); /*SD Read*/
      pieceCount = rdline.toInt();

      readFile(SD, "/27.txt"); /*SD Read*/
      autoCountMode = rdline.toInt() != 0;

      if (autoCountMode == 1) {
        String autoM = "A";
        send_text6(autoM);
        send_text6(autoM);
      }

      if (autoCountMode == 0) {
        String manualM = "M";
        send_text6(manualM);
        send_text6(manualM);
      }


      // readFile(SD, "/43.txt"); /*SD Read*/
      // accumulatedArea = rdline.toDouble();

      readFile(SD, "/44.txt"); /*SD Read*/
      trainedArea = rdline.toFloat();

      readFile(SD, "/69.txt");
      idleVpp = rdline.toFloat();

      readFile(SD, "/70.txt");
      Hour_Z = rdline.toInt();

      readFile(SD, "/77.txt");
      Hour_X = rdline.toInt();

      readFile(SD, "/78.txt");
      Hour_Y = rdline.toInt();

      readFile(SD, "/79.txt");
      Done45 = rdline.toInt();

      readFile(SD, "/80.txt");
      Min_X = rdline.toInt();

      readFile(SD, "/81.txt");
      Min_Y = rdline.toInt();

      readFile(SD, "/75.txt");
      smv_CAL = rdline.toInt();

      readFile(SD, "/76.txt");
      trainedSMV = rdline.toInt();

      readFile(SD, "/83.txt");
      part = rdline;
      Serial.print("part :");
      Serial.println(part);

      readFile(SD, "/84.txt");
      DHU = rdline.toFloat();
      Serial.print("DHU :");
      Serial.println(DHU);

      if (ReadDone == 0) {
        DHU = 0.00;
      }

      DHUfunction();

      readFile(SD, "/85.txt");
      areaResetTime = rdline.toInt();

      Serial.println("/**** Operator after off standard start ****/");

      pageShift23();
      pageShift23();
    }


    /**** Network settingup page when operator lunch break ****/
    else if ((pageNo == "0") && (attendanceOp == 1) && (errorS == 0) && (BreakCheck == 1)) {
      Serial.println("/**** Operator after lunch start ****/");



      delay(300);
      BreakOVER = 1;
      delay(300);
      NO_Power = 1;
      delay(300);
      memLun = 0;



      readFile(SD, "/1.txt"); /*SD Read*/
      smv = rdline.toFloat();
      String smv_SD = String(smv);
      send_text9(smv_SD);
      send_text9(smv_SD);

      readFile(SD, "/2.txt"); /*SD Read*/
      spi = rdline.toInt();
      String spi_SD = String(spi);
      send_text10(spi_SD);
      send_text10(spi_SD);

      readFile(SD, "/3.txt"); /*SD Read*/
      workingHr = rdline.toInt();

      readFile(SD, "/51.txt"); /*SD Read*/
      DAILY = rdline.toInt();

      readFile(SD, "/4.txt"); /*SD Read*/
      E_RGB_LOW = rdline.toInt();

      readFile(SD, "/5.txt"); /*SD Read*/
      E_RGB_HIGH = rdline.toInt();

      readFile(SD, "/6.txt"); /*SD Read*/
      target = rdline.toInt();
      Tar = target;
      send_text3(rdline);
      send_text3(rdline);

      readFile(SD, "/7.txt"); /*SD Read*/
      machineId = rdline;
      send_text20(machineId);

      readFile(SD, "/8.txt"); /*SD Read*/
      machineType = rdline;
      send_text7(machineType);
      send_text7(machineType);

      readFile(SD, "/9.txt"); /*SD Read*/
      ownership = rdline;
      send_text8(ownership);
      send_text8(ownership);

      readFile(SD, "/10.txt"); /*SD Read*/
      obbOperationId = rdline;

      readFile(SD, "/11.txt"); /*SD Read*/
      style = rdline;
      send_text14(style);
      send_text14(style);

      readFile(SD, "/12.txt"); /*SD Read*/
      supervisorName = rdline;
      send_text17(supervisorName);
      send_text17(supervisorName);

      readFile(SD, "/13.txt"); /*SD Read*/
      supervisorRFID = rdline;

      readFile(SD, "/14.txt"); /*SD Read*/
      mechanicName = rdline;
      send_text16(mechanicName);
      send_text16(mechanicName);

      readFile(SD, "/15.txt"); /*SD Read*/
      mechanicRFID = rdline;

      readFile(SD, "/16.txt"); /*SD Read*/
      MeEmployeeId = rdline;

      readFile(SD, "/17.txt"); /*SD Read*/
      SuEmployeeId = rdline;

      readFile(SD, "/18.txt"); /*SD Read*/
      qualityInsName = rdline;
      send_text18(qualityInsName);
      send_text18(qualityInsName);

      readFile(SD, "/19.txt"); /*SD Read*/
      operationName = rdline;
      send_text15(operationName);

      readFile(SD, "/20.txt"); /*SD Read*/
      productionLine = rdline;
      send_text13(productionLine);
      send_text13(productionLine);

      readFile(SD, "/24.txt"); /*SD Read*/
      OperatorID = rdline;
      send_text1(OperatorID);
      send_text1(OperatorID);

      send_text19(deviceId);
      send_text19(deviceId);

      readFile(SD, "/54.txt"); /*SD Read*/
      t = rdline.toInt();

      readFile(SD, "/55.txt"); /*SD Read*/
      AEM = rdline.toInt();

      readFile(SD, "/52.txt");
      G = rdline.toInt();

      formattedCount = String(G);
      while (formattedCount.length() < 6) {
        formattedCount = "0" + formattedCount;  // Prepend zeros until the length is 4
      }
      send_text27(formattedCount);
      send_text27(formattedCount);

      readFile(SD, "/53.txt");
      LG = rdline.toInt();

      readFile(SD, "/68.txt");
      RG = rdline.toInt();

      readFile(SD, "/49.txt");
      N = rdline.toInt();
      send_text4(rdline);
      send_text4(rdline);
      if (N == 0) {
        String Nstr = "0";
        send_text4(Nstr);
        send_text4(Nstr);
      }

      //readFile(SD, "/45.txt");
      //CON = rdline.toInt();

      readFile(SD, "/46.txt");
      J = rdline.toInt();

      readFile(SD, "/47.txt");
      L = rdline.toInt();

      delay(50);
      Lvalue = (L + 1);
      String Ldata = String(Lvalue);
      send_text23(Ldata);  //L+1
      send_text23(Ldata);
      send_text23(Ldata);

      readFile(SD, "/48.txt");
      proTar = rdline.toInt();

      //readFile(SD, "/50.txt");
      //HrTar = rdline.toInt();
      //send_text3(rdline);
      //send_text3(rdline);

      readFile(SD, "/42.txt"); /*SD Read*/
      trainingCountDown = rdline.toInt();

      readFile(SD, "/60.txt");
      EFF_RGB_MEM = rdline.toInt();

      if (ReadDone == 0) {
        EFF_RGB_MEM = 0;
      }

      if (EFF_RGB_MEM == 1) {
        readFile(SD, "/54.txt");
        t = rdline.toInt();
        meter2();
      }

      if (EFF_RGB_MEM == 0) {
        meter2();
      }

      readFile(SD, "/34.txt");
      breakCount = rdline.toInt();
      if (ReadDone == 0) {
        breakCount = 0;
      }

      readFile(SD, "/30.txt");
      mCallCount = rdline.toInt();
      if (ReadDone == 0) {
        mCallCount = 0;
      }

      readFile(SD, "/28.txt");
      mR = rdline.toInt();
      if (ReadDone == 0) {
        mR = 0;
      }

      readFile(SD, "/32.txt");
      sCallCount = rdline.toInt();
      if (ReadDone == 0) {
        sCallCount = 0;
      }

      readFile(SD, "/25.txt");
      sR = rdline.toInt();
      if (ReadDone == 0) {
        sR = 0;
      }

      readFile(SD, "/26.txt");
      S_mem = rdline.toInt();
      if (ReadDone == 0) {
        S_mem = 0;
      }

      readFile(SD, "/29.txt");
      M_mem = rdline.toInt();
      if (ReadDone == 0) {
        M_mem = 0;
      }

      if (sR == 0) {
        uint8_t data1[] = { 0x5A, 0xA5, 0x05, 0x82, 0x17, 0x00, 0x00, 0x00 };
        for (size_t i = 0; i < 8; i++) {
          Serial2.write(data1[i]);
        }
      }

      if (sR == 1) {
        uint8_t data1[] = { 0x5A, 0xA5, 0x05, 0x82, 0x17, 0x00, 0x00, 0x01 };  // Supervisor icon change to green color
        for (size_t i = 0; i < 8; i++) {
          Serial2.write(data1[i]);
        }
      }

      if (mR == 0) {
        uint8_t data1[] = { 0x5A, 0xA5, 0x05, 0x82, 0x20, 0x00, 0x00, 0x00 };
        for (size_t i = 0; i < 8; i++) {
          Serial2.write(data1[i]);
        }
      }

      if (mR == 1) {
        uint8_t data1[] = { 0x5A, 0xA5, 0x05, 0x82, 0x20, 0x00, 0x00, 0x01 };
        for (size_t i = 0; i < 8; i++) {
          Serial2.write(data1[i]);
        }
      }

      readFile(SD, "/62.txt");
      tt = rdline.toInt();

      readFile(SD, "/61.txt");
      AEMM = rdline.toInt();

      readFile(SD, "/31.txt"); /*SD Read*/
      TimeME = rdline;

      readFile(SD, "/33.txt"); /*SD Read*/
      TimeSU = rdline;

      readFile(SD, "/38.txt"); /*SD Read*/
      RoundTLS = rdline.toInt();

      readFile(SD, "/23.txt");  // Login time
      timeStr1 = rdline;

      readFile(SD, "/59.txt");  // Efficiency time
      timeStr2 = rdline;

      readFile(SD, "/36.txt");  // Break start time
      timeStr3 = rdline;

      if (L == 0) {
        time_t TIME1 = timeStringToSeconds(timeStr1);  // Login time

        time_t TIMEst = timeStringToSeconds(timeStr3);  // Break start time

        time_t timeDifference = (TIMEst - TIME1);
        SDdiffSecond = timeDifference;
        Serial.println("SDdiffSecond1");
        Serial.print(SDdiffSecond);
      }

      if (L >= 1) {
        time_t TIME3 = timeStringToSeconds(timeStr2);  // Efficiency time

        time_t TIMEst = timeStringToSeconds(timeStr3);  // Break start time

        time_t timeDifference = (TIMEst - TIME3);
        SDdiffSecond = timeDifference;
        Serial.println("SDdiffSecond2");
        Serial.println(SDdiffSecond);
      }

      readFile(SD, "/63.txt");
      sessionId_O = rdline;

      readFile(SD, "/64.txt");
      sessionId_S = rdline;

      readFile(SD, "/65.txt");
      sessionId_M = rdline;

      readFile(SD, "/40.txt"); /*SD Read*/
      buttonPressCount = rdline.toInt();
      if ((ReadDone == 0) || (buttonPressCount == 1)) {
        buttonPressCount = 0;
      }

      readFile(SD, "/41.txt"); /*SD Read*/
      pieceCount = rdline.toInt();

      readFile(SD, "/27.txt"); /*SD Read*/
      autoCountMode = rdline.toInt() != 0;

      if (autoCountMode == 1) {
        String autoM = "A";
        send_text6(autoM);
        send_text6(autoM);
      }

      if (autoCountMode == 0) {
        String manualM = "M";
        send_text6(manualM);
        send_text6(manualM);
      }


      // readFile(SD, "/43.txt"); /*SD Read*/
      // accumulatedArea = rdline.toDouble();

      readFile(SD, "/44.txt"); /*SD Read*/
      trainedArea = rdline.toFloat();

      readFile(SD, "/69.txt");
      idleVpp = rdline.toFloat();

      readFile(SD, "/70.txt");
      Hour_Z = rdline.toInt();

      readFile(SD, "/77.txt");
      Hour_X = rdline.toInt();

      readFile(SD, "/78.txt");
      Hour_Y = rdline.toInt();

      readFile(SD, "/79.txt");
      Done45 = rdline.toInt();

      readFile(SD, "/80.txt");
      Min_X = rdline.toInt();

      readFile(SD, "/81.txt");
      Min_Y = rdline.toInt();

      readFile(SD, "/75.txt");
      smv_CAL = rdline.toInt();

      readFile(SD, "/76.txt");
      trainedSMV = rdline.toInt();

      readFile(SD, "/83.txt");
      part = rdline;
      Serial.print("part :");
      Serial.println(part);

      readFile(SD, "/84.txt");
      DHU = rdline.toFloat();
      Serial.print("DHU :");
      Serial.println(DHU);

      if (ReadDone == 0) {
        DHU = 0.00;
      }

      DHUfunction();

      readFile(SD, "/85.txt");
      areaResetTime = rdline.toInt();

      Serial.println("/**** Operator after lunch end ****/");

      pageShift17();
      pageShift17();
    }


    /**** Network settingup page when Supervisor and mechnic power on ****/
    else if ((pageNo == "0") && (attendanceOp == 1) && (errorS == 0) && (SuMe == 1)) {
      Serial.println("/**** Supervisor and mechnic power on start ****/");



      delay(300);
      NO_Power = 1;

      readFile(SD, "/1.txt"); /*SD Read*/
      smv = rdline.toFloat();
      String smv_SD = String(smv);
      send_text9(smv_SD);
      send_text9(smv_SD);

      readFile(SD, "/2.txt"); /*SD Read*/
      spi = rdline.toInt();
      String spi_SD = String(spi);
      send_text10(spi_SD);
      send_text10(spi_SD);

      readFile(SD, "/3.txt"); /*SD Read*/
      workingHr = rdline.toInt();

      readFile(SD, "/51.txt"); /*SD Read*/
      DAILY = rdline.toInt();

      readFile(SD, "/4.txt"); /*SD Read*/
      E_RGB_LOW = rdline.toInt();

      readFile(SD, "/5.txt"); /*SD Read*/
      E_RGB_HIGH = rdline.toInt();

      readFile(SD, "/6.txt"); /*SD Read*/
      target = rdline.toInt();
      Tar = target;
      send_text3(rdline);
      send_text3(rdline);


      readFile(SD, "/7.txt"); /*SD Read*/
      machineId = rdline;
      send_text20(machineId);

      readFile(SD, "/8.txt"); /*SD Read*/
      machineType = rdline;
      send_text7(machineType);
      send_text7(machineType);

      readFile(SD, "/9.txt"); /*SD Read*/
      ownership = rdline;
      send_text8(ownership);
      send_text8(ownership);

      readFile(SD, "/10.txt"); /*SD Read*/
      obbOperationId = rdline;

      readFile(SD, "/11.txt"); /*SD Read*/
      style = rdline;
      send_text14(style);
      send_text14(style);

      readFile(SD, "/12.txt"); /*SD Read*/
      supervisorName = rdline;
      send_text17(supervisorName);
      send_text17(supervisorName);

      readFile(SD, "/13.txt"); /*SD Read*/
      supervisorRFID = rdline;

      readFile(SD, "/14.txt"); /*SD Read*/
      mechanicName = rdline;
      send_text16(mechanicName);
      send_text16(mechanicName);

      readFile(SD, "/15.txt"); /*SD Read*/
      mechanicRFID = rdline;

      readFile(SD, "/16.txt"); /*SD Read*/
      MeEmployeeId = rdline;

      readFile(SD, "/17.txt"); /*SD Read*/
      SuEmployeeId = rdline;

      readFile(SD, "/18.txt"); /*SD Read*/
      qualityInsName = rdline;
      send_text18(qualityInsName);
      send_text18(qualityInsName);

      readFile(SD, "/19.txt"); /*SD Read*/
      operationName = rdline;
      send_text15(operationName);

      readFile(SD, "/20.txt"); /*SD Read*/
      productionLine = rdline;
      send_text13(productionLine);
      send_text13(productionLine);

      readFile(SD, "/24.txt"); /*SD Read*/
      OperatorID = rdline;
      send_text1(OperatorID);
      send_text1(OperatorID);

      send_text19(deviceId);
      send_text19(deviceId);

      readFile(SD, "/54.txt"); /*SD Read*/
      t = rdline.toInt();

      readFile(SD, "/55.txt"); /*SD Read*/
      AEM = rdline.toInt();

      readFile(SD, "/52.txt");
      G = rdline.toInt();

      formattedCount = String(G);
      while (formattedCount.length() < 6) {
        formattedCount = "0" + formattedCount;  // Prepend zeros until the length is 4
      }
      send_text27(formattedCount);
      send_text27(formattedCount);

      readFile(SD, "/53.txt");
      LG = rdline.toInt();

      readFile(SD, "/68.txt");
      RG = rdline.toInt();

      readFile(SD, "/49.txt");
      N = rdline.toInt();
      send_text4(rdline);
      send_text4(rdline);
      if (N == 0) {
        String Nstr = "0";
        send_text4(Nstr);
        send_text4(Nstr);
      }

      //readFile(SD, "/45.txt");
      //CON = rdline.toInt();

      readFile(SD, "/46.txt");
      J = rdline.toInt();

      readFile(SD, "/47.txt");
      L = rdline.toInt();

      delay(50);
      Lvalue = (L + 1);
      String Ldata = String(Lvalue);
      send_text23(Ldata);  //L+1
      send_text23(Ldata);
      send_text23(Ldata);

      readFile(SD, "/48.txt");
      proTar = rdline.toInt();

      //readFile(SD, "/50.txt");
      //HrTar = rdline.toInt();
      //send_text3(rdline);
      //send_text3(rdline);

      readFile(SD, "/42.txt"); /*SD Read*/
      trainingCountDown = rdline.toInt();

      readFile(SD, "/60.txt");
      EFF_RGB_MEM = rdline.toInt();

      if (ReadDone == 0) {
        EFF_RGB_MEM = 0;
      }

      if (EFF_RGB_MEM == 1) {
        readFile(SD, "/54.txt");
        t = rdline.toInt();
        meter2();
      }

      if (EFF_RGB_MEM == 0) {
        meter2();
      }

      readFile(SD, "/34.txt");
      breakCount = rdline.toInt();
      if (ReadDone == 0) {
        breakCount = 0;
      }

      readFile(SD, "/30.txt");
      mCallCount = rdline.toInt();
      if (ReadDone == 0) {
        mCallCount = 0;
      }

      readFile(SD, "/28.txt");
      mR = rdline.toInt();
      if (ReadDone == 0) {
        mR = 0;
      }

      readFile(SD, "/32.txt");
      sCallCount = rdline.toInt();
      if (ReadDone == 0) {
        sCallCount = 0;
      }

      readFile(SD, "/25.txt");
      sR = rdline.toInt();
      if (ReadDone == 0) {
        sR = 0;
      }

      readFile(SD, "/26.txt");
      S_mem = rdline.toInt();
      if (ReadDone == 0) {
        S_mem = 0;
      }

      readFile(SD, "/29.txt");
      M_mem = rdline.toInt();
      if (ReadDone == 0) {
        M_mem = 0;
      }

      if (sR == 0) {
        uint8_t data1[] = { 0x5A, 0xA5, 0x05, 0x82, 0x17, 0x00, 0x00, 0x00 };
        for (size_t i = 0; i < 8; i++) {
          Serial2.write(data1[i]);
        }
      }

      if (sR == 1) {
        uint8_t data1[] = { 0x5A, 0xA5, 0x05, 0x82, 0x17, 0x00, 0x00, 0x01 };  // Supervisor icon change to green color
        for (size_t i = 0; i < 8; i++) {
          Serial2.write(data1[i]);
        }
      }

      if (mR == 0) {
        uint8_t data1[] = { 0x5A, 0xA5, 0x05, 0x82, 0x20, 0x00, 0x00, 0x00 };
        for (size_t i = 0; i < 8; i++) {
          Serial2.write(data1[i]);
        }
      }

      if (mR == 1) {
        uint8_t data1[] = { 0x5A, 0xA5, 0x05, 0x82, 0x20, 0x00, 0x00, 0x01 };
        for (size_t i = 0; i < 8; i++) {
          Serial2.write(data1[i]);
        }
      }

      readFile(SD, "/62.txt");
      tt = rdline.toInt();

      readFile(SD, "/61.txt");
      AEMM = rdline.toInt();

      readFile(SD, "/31.txt"); /*SD Read*/
      TimeME = rdline;

      readFile(SD, "/33.txt"); /*SD Read*/
      TimeSU = rdline;

      readFile(SD, "/38.txt"); /*SD Read*/
      RoundTLS = rdline.toInt();

      readFile(SD, "/23.txt");  // Login time
      timeStr1 = rdline;

      readFile(SD, "/59.txt");  // Efficiency time
      timeStr2 = rdline;

      readFile(SD, "/36.txt");  // Break start time
      timeStr3 = rdline;

      if (L == 0) {
        time_t TIME1 = timeStringToSeconds(timeStr1);  // Login time

        time_t TIMEst = timeStringToSeconds(timeStr3);  // Break start time

        time_t timeDifference = (TIMEst - TIME1);
        SDdiffSecond = timeDifference;
        Serial.println("SDdiffSecond1");
        Serial.print(SDdiffSecond);
      }

      if (L >= 1) {
        time_t TIME3 = timeStringToSeconds(timeStr2);  // Efficiency time

        time_t TIMEst = timeStringToSeconds(timeStr3);  // Break start time

        time_t timeDifference = (TIMEst - TIME3);
        SDdiffSecond = timeDifference;
        Serial.println("SDdiffSecond2");
        Serial.println(SDdiffSecond);
      }

      readFile(SD, "/63.txt");
      sessionId_O = rdline;

      readFile(SD, "/64.txt");
      sessionId_S = rdline;

      readFile(SD, "/65.txt");
      sessionId_M = rdline;

      readFile(SD, "/40.txt"); /*SD Read*/
      buttonPressCount = rdline.toInt();
      if ((ReadDone == 0) || (buttonPressCount == 1)) {
        buttonPressCount = 0;
      }

      readFile(SD, "/41.txt"); /*SD Read*/
      pieceCount = rdline.toInt();

      readFile(SD, "/27.txt"); /*SD Read*/
      autoCountMode = rdline.toInt() != 0;

      if (autoCountMode == 1) {
        String autoM = "A";
        send_text6(autoM);
        send_text6(autoM);
      }

      if (autoCountMode == 0) {
        String manualM = "M";
        send_text6(manualM);
        send_text6(manualM);
      }

      // readFile(SD, "/43.txt"); /*SD Read*/
      // accumulatedArea = rdline.toDouble();

      readFile(SD, "/44.txt"); /*SD Read*/
      trainedArea = rdline.toFloat();

      readFile(SD, "/69.txt");
      idleVpp = rdline.toFloat();

      readFile(SD, "/70.txt");
      Hour_Z = rdline.toInt();

      readFile(SD, "/77.txt");
      Hour_X = rdline.toInt();

      readFile(SD, "/78.txt");
      Hour_Y = rdline.toInt();

      readFile(SD, "/79.txt");
      Done45 = rdline.toInt();

      readFile(SD, "/80.txt");
      Min_X = rdline.toInt();

      readFile(SD, "/81.txt");
      Min_Y = rdline.toInt();

      readFile(SD, "/73.txt");
      SuMe_memo = rdline.toInt();

      readFile(SD, "/75.txt");
      smv_CAL = rdline.toInt();

      readFile(SD, "/76.txt");
      trainedSMV = rdline.toInt();

      readFile(SD, "/83.txt");
      part = rdline;
      Serial.print("part :");
      Serial.println(part);

      readFile(SD, "/84.txt");
      DHU = rdline.toFloat();

      if (ReadDone == 0) {
        DHU = 0.00;
      }
      Serial.print("DHU :");
      Serial.println(DHU);

      DHUfunction();

      readFile(SD, "/85.txt");
      areaResetTime = rdline.toInt();

      Serial.println("/**** Supervisor and mechanic ****/");

      if (SuMe_memo == 1) {
        supervisorCheck = 1;
        pageShift3();  // Page 16 (Issue in progress)
        pageShift3();  // Page 16 (Issue in progress)

      } else if (SuMe_memo == 2) {
        mechanicCheck = 1;
        pageShift4();  // Page 17 (Repair in progress)
        pageShift4();  // Page 17 (Repair in progress)
      }
    }


    /**** Network settingup page when device power interrupt happend ****/
    else if ((pageNo == "0") && (attendanceOp == 1) && (errorS == 0) && (BreakCheck == 0) && (opChangeM == 0) && (stdCheck == 0) && (SuMe == 0)) {
      Serial.println("/**** Device power interrupt start ****/");



      delay(300);
      NO_Power = 1;

      readFile(SD, "/1.txt"); /*SD Read*/
      smv = rdline.toFloat();
      String smv_SD = String(smv);
      send_text9(smv_SD);
      send_text9(smv_SD);

      readFile(SD, "/2.txt"); /*SD Read*/
      spi = rdline.toInt();
      String spi_SD = String(spi);
      send_text10(spi_SD);
      send_text10(spi_SD);

      readFile(SD, "/3.txt"); /*SD Read*/
      workingHr = rdline.toInt();

      readFile(SD, "/51.txt"); /*SD Read*/
      DAILY = rdline.toInt();

      readFile(SD, "/4.txt"); /*SD Read*/
      E_RGB_LOW = rdline.toInt();

      readFile(SD, "/5.txt"); /*SD Read*/
      E_RGB_HIGH = rdline.toInt();

      readFile(SD, "/6.txt"); /*SD Read*/
      target = rdline.toInt();
      Tar = target;
      send_text3(rdline);
      send_text3(rdline);

      readFile(SD, "/7.txt"); /*SD Read*/
      machineId = rdline;
      send_text20(machineId);

      readFile(SD, "/8.txt"); /*SD Read*/
      machineType = rdline;
      send_text7(machineType);
      send_text7(machineType);

      readFile(SD, "/9.txt"); /*SD Read*/
      ownership = rdline;
      send_text8(ownership);
      send_text8(ownership);

      readFile(SD, "/10.txt"); /*SD Read*/
      obbOperationId = rdline;

      readFile(SD, "/11.txt"); /*SD Read*/
      style = rdline;
      send_text14(style);
      send_text14(style);

      readFile(SD, "/12.txt"); /*SD Read*/
      supervisorName = rdline;
      send_text17(supervisorName);
      send_text17(supervisorName);

      readFile(SD, "/13.txt"); /*SD Read*/
      supervisorRFID = rdline;

      readFile(SD, "/14.txt"); /*SD Read*/
      mechanicName = rdline;
      send_text16(mechanicName);
      send_text16(mechanicName);

      readFile(SD, "/15.txt"); /*SD Read*/
      mechanicRFID = rdline;

      readFile(SD, "/16.txt"); /*SD Read*/
      MeEmployeeId = rdline;

      readFile(SD, "/17.txt"); /*SD Read*/
      SuEmployeeId = rdline;

      readFile(SD, "/18.txt"); /*SD Read*/
      qualityInsName = rdline;
      send_text18(qualityInsName);
      send_text18(qualityInsName);

      readFile(SD, "/19.txt"); /*SD Read*/
      operationName = rdline;
      send_text15(operationName);

      readFile(SD, "/20.txt"); /*SD Read*/
      productionLine = rdline;
      send_text13(productionLine);
      send_text13(productionLine);

      readFile(SD, "/24.txt"); /*SD Read*/
      OperatorID = rdline;
      send_text1(OperatorID);
      send_text1(OperatorID);

      send_text19(deviceId);
      send_text19(deviceId);

      readFile(SD, "/54.txt"); /*SD Read*/
      t = rdline.toInt();

      readFile(SD, "/55.txt"); /*SD Read*/
      AEM = rdline.toInt();

      readFile(SD, "/52.txt");
      G = rdline.toInt();

      formattedCount = String(G);
      while (formattedCount.length() < 6) {
        formattedCount = "0" + formattedCount;  // Prepend zeros until the length is 4
      }
      send_text27(formattedCount);
      send_text27(formattedCount);

      readFile(SD, "/53.txt");
      LG = rdline.toInt();

      readFile(SD, "/68.txt");
      RG = rdline.toInt();

      readFile(SD, "/49.txt");
      N = rdline.toInt();
      send_text4(rdline);
      send_text4(rdline);
      if (N == 0) {
        String Nstr = "0";
        send_text4(Nstr);
        send_text4(Nstr);
      }

      //readFile(SD, "/45.txt");
      //CON = rdline.toInt();
      //
      readFile(SD, "/46.txt");
      J = rdline.toInt();

      readFile(SD, "/47.txt");
      L = rdline.toInt();

      delay(50);
      Lvalue = (L + 1);
      String Ldata = String(Lvalue);
      send_text23(Ldata);  //L+1
      send_text23(Ldata);
      send_text23(Ldata);

      readFile(SD, "/48.txt");
      proTar = rdline.toInt();

      //readFile(SD, "/50.txt");
      //HrTar = rdline.toInt();
      //send_text3(rdline);
      //send_text3(rdline);

      readFile(SD, "/42.txt"); /*SD Read*/
      trainingCountDown = rdline.toInt();

      readFile(SD, "/60.txt");
      EFF_RGB_MEM = rdline.toInt();

      if (ReadDone == 0) {
        EFF_RGB_MEM = 0;
      }

      if (EFF_RGB_MEM == 1) {
        readFile(SD, "/54.txt");
        t = rdline.toInt();
        meter2();
      }

      if (EFF_RGB_MEM == 0) {

        meter2();
      }

      readFile(SD, "/34.txt");
      breakCount = rdline.toInt();
      if (ReadDone == 0) {
        breakCount = 0;
      }

      readFile(SD, "/30.txt");
      mCallCount = rdline.toInt();
      if (ReadDone == 0) {
        mCallCount = 0;
      }

      readFile(SD, "/28.txt");
      mR = rdline.toInt();
      if (ReadDone == 0) {
        mR = 0;
      }

      readFile(SD, "/32.txt");
      sCallCount = rdline.toInt();
      if (ReadDone == 0) {
        sCallCount = 0;
      }

      readFile(SD, "/25.txt");
      sR = rdline.toInt();
      if (ReadDone == 0) {
        sR = 0;
      }

      readFile(SD, "/26.txt");
      S_mem = rdline.toInt();
      if (ReadDone == 0) {
        S_mem = 0;
      }

      readFile(SD, "/29.txt");
      M_mem = rdline.toInt();
      if (ReadDone == 0) {
        M_mem = 0;
      }

      if (sR == 0) {
        uint8_t data1[] = { 0x5A, 0xA5, 0x05, 0x82, 0x17, 0x00, 0x00, 0x00 };
        for (size_t i = 0; i < 8; i++) {
          Serial2.write(data1[i]);
        }
      }

      if (sR == 1) {
        uint8_t data1[] = { 0x5A, 0xA5, 0x05, 0x82, 0x17, 0x00, 0x00, 0x01 };  // Supervisor icon change to green color
        for (size_t i = 0; i < 8; i++) {
          Serial2.write(data1[i]);
        }
      }

      if (mR == 0) {
        uint8_t data1[] = { 0x5A, 0xA5, 0x05, 0x82, 0x20, 0x00, 0x00, 0x00 };
        for (size_t i = 0; i < 8; i++) {
          Serial2.write(data1[i]);
        }
      }

      if (mR == 1) {
        uint8_t data1[] = { 0x5A, 0xA5, 0x05, 0x82, 0x20, 0x00, 0x00, 0x01 };
        for (size_t i = 0; i < 8; i++) {
          Serial2.write(data1[i]);
        }
      }

      readFile(SD, "/62.txt");
      tt = rdline.toInt();

      readFile(SD, "/61.txt");
      AEMM = rdline.toInt();

      readFile(SD, "/31.txt"); /*SD Read*/
      TimeME = rdline;

      readFile(SD, "/33.txt"); /*SD Read*/
      TimeSU = rdline;

      readFile(SD, "/66.txt"); /*SD Read*/
      errorL = rdline.toInt();

      readFile(SD, "/38.txt"); /*SD Read*/
      RoundTLS = rdline.toInt();

      if (errorL == 1) {
        RoundTLS = RoundTLS - 1;
      }

      readFile(SD, "/23.txt");  // Login time
      timeStr1 = rdline;

      readFile(SD, "/59.txt");  // Efficiency time
      timeStr2 = rdline;

      readFile(SD, "/36.txt");  // Break start time
      timeStr3 = rdline;

      if (L == 0) {
        time_t TIME1 = timeStringToSeconds(timeStr1);  // Login time

        time_t TIMEst = timeStringToSeconds(timeStr3);  // Break start time

        time_t timeDifference = (TIMEst - TIME1);
        SDdiffSecond = timeDifference;
        Serial.println("SDdiffSecond1");
        Serial.print(SDdiffSecond);
      }

      if (L >= 1) {
        time_t TIME3 = timeStringToSeconds(timeStr2);  // Efficiency time

        time_t TIMEst = timeStringToSeconds(timeStr3);  // Break start time

        time_t timeDifference = (TIMEst - TIME3);
        SDdiffSecond = timeDifference;
        Serial.println("SDdiffSecond2");
        Serial.println(SDdiffSecond);
      }

      readFile(SD, "/63.txt");
      sessionId_O = rdline;

      readFile(SD, "/64.txt");
      sessionId_S = rdline;

      readFile(SD, "/65.txt");
      sessionId_M = rdline;

      readFile(SD, "/40.txt"); /*SD Read*/
      buttonPressCount = rdline.toInt();
      if ((ReadDone == 0) || (buttonPressCount == 1)) {
        buttonPressCount = 0;
      }

      readFile(SD, "/41.txt"); /*SD Read*/
      pieceCount = rdline.toInt();

      readFile(SD, "/27.txt"); /*SD Read*/
      autoCountMode = rdline.toInt() != 0;

      if (autoCountMode == 1) {
        String autoM = "A";
        send_text6(autoM);
        send_text6(autoM);
      }

      if (autoCountMode == 0) {
        String manualM = "M";
        send_text6(manualM);
        send_text6(manualM);
      }


      // readFile(SD, "/43.txt"); /*SD Read*/
      // accumulatedArea = rdline.toDouble();

      readFile(SD, "/44.txt"); /*SD Read*/
      trainedArea = rdline.toFloat();

      readFile(SD, "/69.txt");
      idleVpp = rdline.toFloat();

      readFile(SD, "/70.txt");
      Hour_Z = rdline.toInt();

      readFile(SD, "/77.txt");
      Hour_X = rdline.toInt();

      readFile(SD, "/78.txt");
      Hour_Y = rdline.toInt();

      readFile(SD, "/79.txt");
      Done45 = rdline.toInt();

      readFile(SD, "/80.txt");
      Min_X = rdline.toInt();

      readFile(SD, "/81.txt");
      Min_Y = rdline.toInt();

      readFile(SD, "/75.txt");
      smv_CAL = rdline.toInt();

      readFile(SD, "/76.txt");
      trainedSMV = rdline.toInt();
      Serial.print("trainedSMV :");
      Serial.println(trainedSMV);

      readFile(SD, "/83.txt");
      part = rdline;
      Serial.print("part :");
      Serial.println(part);

      readFile(SD, "/84.txt");
      DHU = rdline.toFloat();

      if (ReadDone == 0) {
        DHU = 0.00;
      }
      Serial.print("DHU :");
      Serial.println(DHU);

      DHUfunction();

      readFile(SD, "/85.txt");
      areaResetTime = rdline.toInt();

      Serial.println("/**** Device power interrupt end ****/");

      pageShift1();
      pageShift1();
    }


    /**** Network settingup page when download from the database in beginning of the day ****/
    else if ((pageNo == "0") && (FirstPG == 0) && (changeOP == 0) && (BreakCheck == 0) && (stdCheck == 0) && (SuMe == 0)) {
      Serial.println("/**** Download from the database ****/");

      //MQTT_Reconnect();

      unsigned long currentMillis=millis();
      if (currentMillis - previousMillis >= 3000+random(100)*10) {
      previousMillis = currentMillis;
      /** Initial get NODE RED **/
      if (fnCall_initial_data) {
        awaitReply = 1;
        restartCountDown--;
        if(restartCountDown==0)ESP.restart();
        get_initial_data(deviceId);
        
      }
    }

      for (int d = 0; d < 50; d++) {
        if (!awaitReply) d = 50;
        delay(100);
      }

      if (dataReceived_initial_data) {
        FirstPG = 1;

        uint8_t data1[] = { 0x5A, 0xA5, 0x05, 0x82, 0x00, 0xA0, 0x00, 0x0A };  // Buzzer on
        for (size_t i = 0; i < 8; i++) {
          Serial2.write(data1[i]);
        }

        delay(200);

        uint8_t data2[] = { 0x5A, 0xA5, 0x05, 0x82, 0x00, 0xA0, 0x00, 0x0A };
        for (size_t i = 0; i < 8; i++) {
          Serial2.write(data2[i]);
        }

        Serial.println("/**** Download from the database end ****/");

        // To check whether the variables have assigned properly
        Serial.println("");

        opChangeM = 0;
        String opChange_M = String(opChangeM);
        writeFile(SD, "/71.txt", opChange_M.c_str()); /*SD Save*/

        Serial.print("obbOperationId = ");
        Serial.println(obbOperationId);
        writeFile(SD, "/10.txt", obbOperationId.c_str()); /*SD Save*/

        Serial.print("machineId = ");
        Serial.println(machineId);
        writeFile(SD, "/7.txt", machineId.c_str()); /*SD Save*/
        send_text20(machineId);

        Serial.print("machineType = ");
        Serial.println(machineType);
        writeFile(SD, "/8.txt", machineType.c_str()); /*SD Save*/
        send_text7(machineType);
        send_text7(machineType);

        Serial.print("ownership = ");
        Serial.println(ownership);
        writeFile(SD, "/9.txt", ownership.c_str()); /*SD Save*/
        send_text8(ownership);
        send_text8(ownership);

        Serial.print("smv = ");
        Serial.println(smv);
        String smv_SD = String(smv);
        writeFile(SD, "/1.txt", smv_SD.c_str()); /*SD Save*/
        send_text9(smv_SD);
        send_text9(smv_SD);

        Serial.print("spi = ");
        Serial.println(spi);
        String spi_SD = String(spi);
        writeFile(SD, "/2.txt", spi_SD.c_str()); /*SD Save*/
        send_text10(spi_SD);
        send_text10(spi_SD);

        Serial.print("target = ");
        Serial.println(target);
        Tar = target;
        HrTar = Tar;
        String target_SD = String(target);
        writeFile(SD, "/6.txt", target_SD.c_str());  /*SD Save*/
        writeFile(SD, "/50.txt", target_SD.c_str()); /*SD Save*/
        send_text3(target_SD);
        send_text3(target_SD);

        Serial.print("workingHours = ");
        Serial.println(workingHours);
        String workingHours_SD = String(workingHours);
        writeFile(SD, "/3.txt", workingHours_SD.c_str()); /*SD Save*/
        workingHr = workingHours;

        DAILY = workingHr * Tar;
        String DAILY_SD = String(DAILY);
        writeFile(SD, "/51.txt", DAILY_SD.c_str()); /*SD Save*/


        Serial.print("efficiencyLevel1 = ");
        Serial.println(efficiencyLevel1);
        E_RGB_LOW = efficiencyLevel1;
        String efficiencyLevel1_SD = String(efficiencyLevel1);
        writeFile(SD, "/4.txt", efficiencyLevel1_SD.c_str()); /*SD Save*/

        Serial.print("efficiencyLevel3 = ");
        Serial.println(efficiencyLevel3);
        E_RGB_HIGH = efficiencyLevel3;
        String efficiencyLevel3_SD = String(efficiencyLevel3);
        writeFile(SD, "/5.txt", efficiencyLevel3_SD.c_str()); /*SD Save*/

        Serial.print("style = ");
        Serial.println(style);
        writeFile(SD, "/11.txt", style.c_str()); /*SD Save*/
        send_text14(style);
        send_text14(style);

        Serial.print("operationName = ");
        Serial.println(operationName);
        writeFile(SD, "/19.txt", operationName.c_str()); /*SD Save*/
        send_text15(operationName);

        Serial.print("SuEmployeeId = ");
        Serial.println(SuEmployeeId);
        writeFile(SD, "/17.txt", SuEmployeeId.c_str()); /*SD Save*/

        Serial.print("supervisorRFID = ");
        Serial.println(supervisorRFID);
        writeFile(SD, "/13.txt", supervisorRFID.c_str()); /*SD Save*/

        Serial.print("supervisorName = ");
        Serial.println(supervisorName);
        writeFile(SD, "/12.txt", supervisorName.c_str()); /*SD Save*/
        send_text17(supervisorName);
        send_text17(supervisorName);

        Serial.print("mechanicName = ");
        Serial.println(mechanicName);
        writeFile(SD, "/14.txt", mechanicName.c_str()); /*SD Save*/
        send_text16(mechanicName);
        send_text16(mechanicName);

        Serial.print("MeEmployeeId = ");
        Serial.println(MeEmployeeId);
        writeFile(SD, "/16.txt", MeEmployeeId.c_str()); /*SD Save*/

        Serial.print("mechanicRFID = ");
        Serial.println(mechanicRFID);
        writeFile(SD, "/15.txt", mechanicRFID.c_str()); /*SD Save*/

        Serial.print("qualityInsName = ");
        Serial.println(qualityInsName);
        writeFile(SD, "/18.txt", qualityInsName.c_str()); /*SD Save*/
        send_text18(qualityInsName);
        send_text18(qualityInsName);

        Serial.print("productionLine = ");
        Serial.println(productionLine);
        writeFile(SD, "/20.txt", productionLine.c_str()); /*SD Save*/
        send_text13(productionLine);

        Serial.print("part = ");
        Serial.println(part);
        writeFile(SD, "/83.txt", part.c_str()); /*SD Save*/

        dataReceived_initial_data = false;

        readFile(SD, "/84.txt");
        DHU = rdline.toFloat();

        if (ReadDone == 0) {
          DHU = 0.00;
        }

        Serial.print("DHU :");
        Serial.println(DHU);

        DHUfunction();

        readFile(SD, "/44.txt"); /*SD Read*/
        trainedArea = rdline.toFloat();
        if (ReadDone == 0) {
          trainedArea = 0.0;
        }

        readFile(SD, "/40.txt"); /*SD Read*/
        buttonPressCount = rdline.toInt();
        if ((ReadDone == 0) || (buttonPressCount == 1)) {
          buttonPressCount = 0;
        }

        readFile(SD, "/42.txt"); /*SD Read*/
        trainingCountDown = rdline.toInt();
        if (ReadDone == 0) {
          trainingCountDown = 2;
        }

        if (trainingCountDown == 2) {
          meter2();
        } else {
          meter2();
        }

        readFile(SD, "/69.txt");
        idleVpp = rdline.toFloat();
        if (ReadDone == 0) {
          idleVpp = 0.0;
        }

        readFile(SD, "/85.txt");
        areaResetTime = rdline.toInt();
        if (ReadDone == 0) {
          areaResetTime = 3;
        }

        readFile(SD, "/76.txt");
        trainedSMV = rdline.toInt();
        if (ReadDone == 0) {
          trainedSMV = 0;
        }

        pageShift7();
        pageShift7();
      }
    }
  }

  /**** WIFI reset function ****/
  else if ((buttonNo == "1320")) {
    pageShift5();
    pageShift5();
    delay(1500);
    wm.resetSettings();
    pageShift2();
    pageShift2();
    ESP.restart();
  }

  delay(20);
}

/**** RFID Card Reading Function ****/
void readCard(int blockAddr) {
  digitalWrite(SS_SD, HIGH);
  digitalWrite(SS_PIN, LOW);

  if (rfid.auth(PICC_AUTHENT1A, blockAddr, sectorKeyA[blockAddr / 4], rfid.serNum) == MI_OK) {

    Serial.print("Read from the blockAddr of the card : ");
    Serial.println(blockAddr, DEC);
    if (rfid.read(blockAddr, str) == MI_OK) {
      pageShift5();  // Page 19 (Loading Page)
      delay(500);
      //
      Serial.print("The data is : ");
      dataa = ((char *)str);
      Serial.println(dataa);
      fnCall_RFIDresult = true;
      checkAccess(dataa);
    }
  }
  digitalWrite(SS_PIN, HIGH);
}


/**** WiFi Reconnecting Function ****/
void reconnectWiFi() {

  WiFi.disconnect();
  WiFi.reconnect();

  // Attempt to reconnect
  unsigned long startTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis()) {  // Wait for 10 seconds

    leds[0] = CRGB(255, 0, 0);
    FastLED.show();
    leds[1] = CRGB(255, 0, 0);
    FastLED.show();
    vTaskDelay(300 / portTICK_PERIOD_MS);
    leds[0] = CRGB(0, 0, 0);
    FastLED.show();
    leds[1] = CRGB(0, 0, 0);
    FastLED.show();
    vTaskDelay(300 / portTICK_PERIOD_MS);
    leds[0] = CRGB(255, 0, 0);
    FastLED.show();
    leds[1] = CRGB(255, 0, 0);
    FastLED.show();
    vTaskDelay(300 / portTICK_PERIOD_MS);
    leds[0] = CRGB(0, 0, 0);
    FastLED.show();
    leds[1] = CRGB(0, 0, 0);
    FastLED.show();
    vTaskDelay(300 / portTICK_PERIOD_MS);
  }
}


/**** Main Page Dwin Functions ****/
void send_text1(String text) {  // Operator ID
  text.toCharArray(output, 50);
  Serial2.write(0x5A);
  Serial2.write(0xA5);
  Serial2.write(0x12);
  Serial2.write(0x82);
  Serial2.write(0x10);  // VP address (1000)
  Serial2.write(0x00);
  Serial2.write(output);
  Serial2.write(0x00);
}

void send_text2(String text) {  // Live time
  text.toCharArray(output, 50);
  Serial2.write(0x5A);
  Serial2.write(0xA5);
  Serial2.write(0x15);
  Serial2.write(0x82);
  Serial2.write(0x11);  // VP address (1100)
  Serial2.write(0x00);
  Serial2.write(output);
  Serial2.write(0x00);
}

void send_text3(String text) {  //Target production
  text.toCharArray(output, 50);
  Serial2.write(0x5A);
  Serial2.write(0xA5);
  Serial2.write(0x07);
  Serial2.write(0x82);
  Serial2.write(0x12);  // VP address (1200)
  Serial2.write(0x00);
  Serial2.write(output);
  Serial2.write(0x00);
}

void send_text4(String text) {  //Actual production
  text.toCharArray(output, 50);
  Serial2.write(0x5A);
  Serial2.write(0xA5);
  Serial2.write(0x07);
  Serial2.write(0x82);
  Serial2.write(0x16);  // VP address (1600)
  Serial2.write(0x00);
  Serial2.write(output);
  Serial2.write(0x00);
}



void send_text6(String text) {  // Auto Manual selection
  text.toCharArray(output, 50);
  Serial2.write(0x5A);
  Serial2.write(0xA5);
  Serial2.write(0x09);
  Serial2.write(0x82);
  Serial2.write(0x14);  // VP address (1400)
  Serial2.write(0x00);
  Serial2.write(output);
  Serial2.write(0x00);
}

/**** Table One ****/
void send_text13(String text) {  // Operation line
  text.toCharArray(output, 50);
  Serial2.write(0x5A);
  Serial2.write(0xA5);
  Serial2.write(0x0F);
  Serial2.write(0x82);
  Serial2.write(0x70);  // VP address (7000)
  Serial2.write(0x00);
  Serial2.write(output);
  Serial2.write(0x00);
}
void send_text14(String text) {  // Style Name
  text.toCharArray(output, 50);
  Serial2.write(0x5A);
  Serial2.write(0xA5);
  Serial2.write(0x0F);
  Serial2.write(0x82);
  Serial2.write(0x71);  // VP address (7100)
  Serial2.write(0x00);
  Serial2.write(output);
  Serial2.write(0x00);
}
void send_text15(String text) {  // Operation name
  text.toCharArray(output, 50);
  Serial2.write(0x5A);
  Serial2.write(0xA5);
  Serial2.write(0x0F);
  Serial2.write(0x82);
  Serial2.write(0x72);  // VP address (7200)
  Serial2.write(0x00);
  Serial2.write(output);
  Serial2.write(0x00);
}
void send_text16(String text) {  // Mechanic ID
  text.toCharArray(output, 50);
  Serial2.write(0x5A);
  Serial2.write(0xA5);
  Serial2.write(0x0F);
  Serial2.write(0x82);
  Serial2.write(0x73);  // VP address (7300)
  Serial2.write(0x00);
  Serial2.write(output);
  Serial2.write(0x00);
}
void send_text17(String text) {  //Supervisor ID
  text.toCharArray(output, 50);
  Serial2.write(0x5A);
  Serial2.write(0xA5);
  Serial2.write(0x0F);
  Serial2.write(0x82);
  Serial2.write(0x74);  // VP address (7400)
  Serial2.write(0x00);
  Serial2.write(output);
  Serial2.write(0x00);
}
void send_text18(String text) {  // QI name
  text.toCharArray(output, 50);
  Serial2.write(0x5A);
  Serial2.write(0xA5);
  Serial2.write(0x0F);
  Serial2.write(0x82);
  Serial2.write(0x75);  // VP address (7500)
  Serial2.write(0x00);
  Serial2.write(output);
  Serial2.write(0x00);
}
void send_text19(String text) {  //ELIoT ID
  text.toCharArray(output, 50);
  Serial2.write(0x5A);
  Serial2.write(0xA5);
  Serial2.write(0x0F);
  Serial2.write(0x82);
  Serial2.write(0x76);  // VP address (7600)
  Serial2.write(0x00);
  Serial2.write(output);
  Serial2.write(0x00);
}
void send_text20(String text) {  //Sewing machine ID
  text.toCharArray(output, 50);
  Serial2.write(0x5A);
  Serial2.write(0xA5);
  Serial2.write(0x0F);
  Serial2.write(0x82);
  Serial2.write(0x77);  // VP address (7700)
  Serial2.write(0x00);
  Serial2.write(output);
  Serial2.write(0x00);
}


/**** Table Two ****/
void send_text7(String text) {  //Sewing machine model
  text.toCharArray(output, 50);
  Serial2.write(0x5A);
  Serial2.write(0xA5);
  Serial2.write(0x06);
  Serial2.write(0x82);
  Serial2.write(0x80);  // VP address (8000)
  Serial2.write(0x00);
  Serial2.write(output);
  Serial2.write(0x00);
}

void send_text8(String text) {  //Sewing machine ownership
  text.toCharArray(output, 50);
  Serial2.write(0x5A);
  Serial2.write(0xA5);
  Serial2.write(0x0A);
  Serial2.write(0x82);
  Serial2.write(0x81);  // VP address (8100)
  Serial2.write(0x00);
  Serial2.write(output);
  Serial2.write(0x00);
}

void send_text9(String text) {  // Opertation SMV
  text.toCharArray(output, 50);
  Serial2.write(0x5A);
  Serial2.write(0xA5);
  Serial2.write(0x06);
  Serial2.write(0x82);
  Serial2.write(0x87);  // VP address (8700)
  Serial2.write(0x00);
  Serial2.write(output);
  Serial2.write(0x00);
}

void send_text10(String text) {  // Operation SPI
  text.toCharArray(output, 50);
  Serial2.write(0x5A);
  Serial2.write(0xA5);
  Serial2.write(0x06);
  Serial2.write(0x82);
  Serial2.write(0x85);  // VP address (8500)
  Serial2.write(0x00);
  Serial2.write(output);
  Serial2.write(0x00);
}


/**** Other Text ****/
void send_text21(String text) {  // Operator break time
  text.toCharArray(output, 50);
  Serial2.write(0x5A);
  Serial2.write(0xA5);
  Serial2.write(0x09);
  Serial2.write(0x82);
  Serial2.write(0x42);  // VP address (4200)
  Serial2.write(0x00);
  Serial2.write(output);
  Serial2.write(0x00);
}

void send_text23(String text) {  // Production time slots
  text.toCharArray(output, 50);
  Serial2.write(0x5A);
  Serial2.write(0xA5);
  Serial2.write(0x06);
  Serial2.write(0x82);
  Serial2.write(0x15);  // VP address (1500)
  Serial2.write(0x00);
  Serial2.write(output);
  Serial2.write(0x00);
}

void send_text24(String text) {  // code version
  text.toCharArray(output, 50);
  Serial2.write(0x5A);
  Serial2.write(0xA5);
  Serial2.write(0x09);
  Serial2.write(0x82);
  Serial2.write(0x83);  // VP address (8300)
  Serial2.write(0x00);
  Serial2.write(output);
  Serial2.write(0x00);
}

void send_text26(String text) {  // DHU
  text.toCharArray(output, 50);
  Serial2.write(0x5A);
  Serial2.write(0xA5);
  Serial2.write(0x09);
  Serial2.write(0x82);
  Serial2.write(0x82);  // VP address (8200)
  Serial2.write(0x00);
  Serial2.write(output);
  Serial2.write(0x00);
}

void send_text27(String text) {  //TPC
  text.toCharArray(output, 50);
  Serial2.write(0x5A);
  Serial2.write(0xA5);
  Serial2.write(0x09);
  Serial2.write(0x82);
  Serial2.write(0x44);  // VP address (4400)
  Serial2.write(0x00);
  Serial2.write(output);
  Serial2.write(0x00);
}

void send_text28(String text) {  //Topic
  text.toCharArray(output, 50);
  Serial2.write(0x5A);
  Serial2.write(0xA5);
  Serial2.write(0x0F);
  Serial2.write(0x82);
  Serial2.write(0x86);  // VP address (8600)
  Serial2.write(0x00);
  Serial2.write(output);
  Serial2.write(0x00);
}

void send_text29(String text) {  //Body 1
  text.toCharArray(output, 50);
  Serial2.write(0x5A);
  Serial2.write(0xA5);
  Serial2.write(0x0F);
  Serial2.write(0x82);
  Serial2.write(0x88);  // VP address (4400)
  Serial2.write(0x00);
  Serial2.write(output);
  Serial2.write(0x00);
}

void send_text30(String text) {  //Body 2
  text.toCharArray(output, 50);
  Serial2.write(0x5A);
  Serial2.write(0xA5);
  Serial2.write(0x0F);
  Serial2.write(0x89);
  Serial2.write(0x44);  // VP address (4400)
  Serial2.write(0x00);
  Serial2.write(output);
  Serial2.write(0x00);
}


/**** Page Changing ****/
void pageShift1()  // Page 02 (Main page)
{
  uint8_t data1[] = { 0x5A, 0xA5, 0x07, 0x82, 0x00, 0x84, 0x5A, 0x01, 0x00, 0x02 };
  for (size_t i = 0; i < 10; i++) {
    Serial2.write(data1[i]);
  }
}

void pageShift2()  // Page 00 (Network setting up page)
{
  uint8_t data1[] = { 0x5A, 0xA5, 0x07, 0x82, 0x00, 0x84, 0x5A, 0x01, 0x00, 0x00 };
  for (size_t i = 0; i < 10; i++) {
    Serial2.write(data1[i]);
  }
}

void pageShift3()  // Page 13 (Issue in progress)
{
  uint8_t data1[] = { 0x5A, 0xA5, 0x07, 0x82, 0x00, 0x84, 0x5A, 0x01, 0x00, 0x0D };
  for (size_t i = 0; i < 10; i++) {
    Serial2.write(data1[i]);
  }
}

void pageShift4()  // Page 14 (Repair in progress)
{
  uint8_t data1[] = { 0x5A, 0xA5, 0x07, 0x82, 0x00, 0x84, 0x5A, 0x01, 0x00, 0x0E };
  for (size_t i = 0; i < 10; i++) {
    Serial2.write(data1[i]);
  }
}

void pageShift5()  // Page 16 (Loading Page)
{
  uint8_t data1[] = { 0x5A, 0xA5, 0x07, 0x82, 0x00, 0x84, 0x5A, 0x01, 0x00, 0x10 };
  for (size_t i = 0; i < 10; i++) {
    Serial2.write(data1[i]);
  }
}

void pageShift6()  // Page 17 (Tryagain Page)
{
  uint8_t data1[] = { 0x5A, 0xA5, 0x07, 0x82, 0x00, 0x84, 0x5A, 0x01, 0x00, 0x11 };
  for (size_t i = 0; i < 10; i++) {
    Serial2.write(data1[i]);
  }
}

void pageShift7()  // Page 01 (Operaor login page)
{
  uint8_t data1[] = { 0x5A, 0xA5, 0x07, 0x82, 0x00, 0x84, 0x5A, 0x01, 0x00, 0x01 };
  for (size_t i = 0; i < 10; i++) {
    Serial2.write(data1[i]);
  }
}

void pageShift10()  // Page 05 (Mechanic Login)
{
  uint8_t data1[] = { 0x5A, 0xA5, 0x07, 0x82, 0x00, 0x84, 0x5A, 0x01, 0x00, 0x05 };
  for (size_t i = 0; i < 10; i++) {
    Serial2.write(data1[i]);
  }
}

void pageShift11()  // Page 07 (Supervisor Login)
{
  uint8_t data1[] = { 0x5A, 0xA5, 0x07, 0x82, 0x00, 0x84, 0x5A, 0x01, 0x00, 0x07 };
  for (size_t i = 0; i < 10; i++) {
    Serial2.write(data1[i]);
  }
}

void pageShift12()  // Page 09 (Operator Logout)
{
  uint8_t data1[] = { 0x5A, 0xA5, 0x07, 0x82, 0x00, 0x84, 0x5A, 0x01, 0x00, 0x09 };
  for (size_t i = 0; i < 10; i++) {
    Serial2.write(data1[i]);
  }
}

void pageShift13()  // Page 03 (Help page)
{
  uint8_t data1[] = { 0x5A, 0xA5, 0x07, 0x82, 0x00, 0x84, 0x5A, 0x01, 0x00, 0x03 };
  for (size_t i = 0; i < 10; i++) {
    Serial2.write(data1[i]);
  }
}

void pageShift14()  // Page 10 (Auto or manual mode confirm by supervisor)
{
  uint8_t data1[] = { 0x5A, 0xA5, 0x07, 0x82, 0x00, 0x84, 0x5A, 0x01, 0x00, 0x0A };
  for (size_t i = 0; i < 10; i++) {
    Serial2.write(data1[i]);
  }
}

void pageShift16()  // Page 25 (Page after the shustdown function`s yes button)
{
  uint8_t data1[] = { 0x5A, 0xA5, 0x07, 0x82, 0x00, 0x84, 0x5A, 0x01, 0x00, 0x19 };
  for (size_t i = 0; i < 10; i++) {
    Serial2.write(data1[i]);
  }
}

void pageShift17()  // Page 15 (Operator break time page)
{
  uint8_t data1[] = { 0x5A, 0xA5, 0x07, 0x82, 0x00, 0x84, 0x5A, 0x01, 0x00, 0x0F };
  for (size_t i = 0; i < 10; i++) {
    Serial2.write(data1[i]);
  }
}

void pageShift18()  // Page 21 (Page before Network setting up page)
{
  uint8_t data1[] = { 0x5A, 0xA5, 0x07, 0x82, 0x00, 0x84, 0x5A, 0x01, 0x00, 0x15 };
  for (size_t i = 0; i < 10; i++) {
    Serial2.write(data1[i]);
  }
}

void pageShift19()  // Page 08 (Supervisor Logout)
{
  uint8_t data1[] = { 0x5A, 0xA5, 0x07, 0x82, 0x00, 0x84, 0x5A, 0x01, 0x00, 0x08 };
  for (size_t i = 0; i < 10; i++) {
    Serial2.write(data1[i]);
  }
}

void pageShift20()  // Page 06 (Mechanic Logout)
{
  uint8_t data1[] = { 0x5A, 0xA5, 0x07, 0x82, 0x00, 0x84, 0x5A, 0x01, 0x00, 0x06 };
  for (size_t i = 0; i < 10; i++) {
    Serial2.write(data1[i]);
  }
}

void pageShift21()  // Page 26 (Training mode started)
{
  uint8_t data1[] = { 0x5A, 0xA5, 0x07, 0x82, 0x00, 0x84, 0x5A, 0x01, 0x00, 0x1A };
  for (size_t i = 0; i < 10; i++) {
    Serial2.write(data1[i]);
  }
}

void pageShift22()  // Page 34 (Dialog box page)
{
  uint8_t data1[] = { 0x5A, 0xA5, 0x07, 0x82, 0x00, 0x84, 0x5A, 0x01, 0x00, 0x22 };
  for (size_t i = 0; i < 10; i++) {
    Serial2.write(data1[i]);
  }
}

void pageShift23()  // Page 20 (OFF STANDARD)
{
  uint8_t data1[] = { 0x5A, 0xA5, 0x07, 0x82, 0x00, 0x84, 0x5A, 0x01, 0x00, 0x14 };
  for (size_t i = 0; i < 10; i++) {
    Serial2.write(data1[i]);
  }
}

void pageShift24()  // Page 10 (Auto or manual mode confirm by supervisor)
{
  uint8_t data1[] = { 0x5A, 0xA5, 0x07, 0x82, 0x00, 0x84, 0x5A, 0x01, 0x00, 0x29 };
  for (size_t i = 0; i < 10; i++) {
    Serial2.write(data1[i]);
  }
}

/**** Read Page ****/
void Readpage()  // Read page function
{
  uint8_t data1[] = { 0x5A, 0xA5, 0x04, 0x83, 0x00, 0x14, 0x01 };
  for (size_t i = 0; i < 7; i++) {
    Serial2.write(data1[i]);
  }
}


/**** Page Checker ****/
void checker() {  // DWIN page and button adress value find function

  Serial2.flush();
  Readpage();
  int startAdd = 00;
  int endAdd = 00;
  delay(20);

  while (Serial2.available()) {
    int inhex = Serial2.read();

    if (inhex == 90 || inhex == 165) {
      continue;
    }

    for (int i = 1; i <= inhex; i++) {
      while (!Serial2.available())
        ;
      int incomingByte = Serial2.read();
      if (i == 2) {
        startAdd = incomingByte;
      }
      if (i == 3) {
        endAdd = incomingByte;
      }
      if (i == 6) {
        PAGE = incomingByte;
      }
    }
    buttonNo = String(startAdd) + String(endAdd);
    pageNo = String(PAGE);
  }
}

/**** Meter One for Efficiency Icon ****/
void meter1() {  // DWIN efficency meter icon function

  float Precentage = ((N * smv * 100) / (60));
  t = round(Precentage);

  if (t < 0) {
    t = 0;
    AVG = t;
  }

  if (t >= 0) {
    AVG = t;
  }

  String tCount = String(t);
  writeFile(SD, "/54.txt", tCount.c_str()); /*SD Save*/

  T += AVG;

  float A1 = T / L;
  AEM = round(A1);

  if (100 < t) {
    t = 100;
  }

  byte Meter1 = t;

  Serial2.write(0x5A);  // start command
  Serial2.write(0xA5);  // start command
  Serial2.write(0x05);  // Data Length
  Serial2.write(0x82);  // read command
  Serial2.write(0x18);
  Serial2.write(0x00);
  Serial2.write(0x00);
  Serial2.write(Meter1);  // Changing value 100%

  Serial2.write(0x5A);  // start command
  Serial2.write(0xA5);  // start command
  Serial2.write(0x05);  // Data Length
  Serial2.write(0x82);  // read command
  Serial2.write(0x18);
  Serial2.write(0x00);
  Serial2.write(0x00);
  Serial2.write(Meter1);  // Changing value 100%

  if (wifiData == 1) {
    String t_str = String(t);
    writeFile(SD, "/62.txt", t_str.c_str()); /*SD Save*/
    String AEM_str = String(AEM);
    writeFile(SD, "/61.txt", AEM_str.c_str()); /*SD Save*/
    tt = t;
    AEMM = AEM;
    eff_mem = 1;
  }

  LG = 0;

  writeFile(SD, "/59.txt", Time1.c_str()); /*SD Save*/

  EFF_RGB_MEM = 1;
  String MEM_RGB = String(EFF_RGB_MEM);
  writeFile(SD, "/60.txt", MEM_RGB.c_str()); /*SD Save*/
}


/**** Meter Two for Efficiency Icon ****/
void meter2() {  // DWIN efficency meter icon function

  if (100 < t) {
    t = 100;
  }

  byte Meter2 = t;

  Serial2.write(0x5A);  // start command
  Serial2.write(0xA5);  // start command
  Serial2.write(0x05);  // Data Length
  Serial2.write(0x82);  // read command
  Serial2.write(0x18);
  Serial2.write(0x00);
  Serial2.write(0x00);
  Serial2.write(Meter2);  // Changing value 100%

  Serial2.write(0x5A);  // start command
  Serial2.write(0xA5);  // start command
  Serial2.write(0x05);  // Data Length
  Serial2.write(0x82);  // read command
  Serial2.write(0x18);
  Serial2.write(0x00);
  Serial2.write(0x00);
  Serial2.write(Meter2);  // Changing value 100%

  if (E_RGB_LOW >= t) {

    leds[2] = CRGB(255, 0, 0);
    FastLED.show();

    leds[3] = CRGB(255, 0, 0);
    FastLED.show();
  }
  if (E_RGB_HIGH > t && E_RGB_LOW < t) {

    leds[2] = CRGB(255, 165, 0);
    FastLED.show();

    leds[3] = CRGB(255, 165, 0);
    FastLED.show();
  }
  if (E_RGB_HIGH <= t) {

    leds[2] = CRGB(0, 255, 0);
    FastLED.show();

    leds[3] = CRGB(0, 255, 0);
    FastLED.show();
  }
  if (trainingCountDown != 0) {

    leds[2] = CRGB(0, 0, 255);
    FastLED.show();

    leds[3] = CRGB(0, 0, 255);
    FastLED.show();
  }
}


/**** CT pieces count ****/
void ctThread() {
  if ((!autoCountMode && digitalRead(button) == 1) || (autoCountMode && hmiButtonFlag)) {
    hmiButtonFlag = false;
    uint8_t longPressCount = 0;  //// max 250
    while ((!autoCountMode && digitalRead(button) == 1) && longPressCount <= 250) {
      longPressCount++;
      delay(10);
      //if (longPressCount == 250) setTrainingMode(trainingNum);
      //Serial.println(longPressCount);
    }
    buttonPressCount++;
    delay(100);

    String buttonPress = String(buttonPressCount);
    writeFile(SD, "/40.txt", buttonPress.c_str()); /*SD Save*/

    if (buttonPressCount == 1) {
      digitalWrite(ledPin, HIGH);

      delay(100);
      setIdleVpp(1);
      accumulatedArea = 0;
      Serial.println("Started trainning...");

      digitalWrite(ledPin, LOW);
    } else if (buttonPressCount == 2) {
      awaitRisingEdge = 1;
    } else if (buttonPressCount > trainingNum + 1) {
      toleranceArea = trainedArea * 0.75;
      if (trainingCountDown == 0 && accumulatedArea > toleranceArea) {
        waitASec = false;
        pieceCount++;
        autoSMV = millis() - lastPieceMillis;
        if (autoSMV < (trainedSMV * 2)) avrAutoSMV.push(autoSMV);
        lastPieceMillis = millis();
        Serial.println(pieceCount);
        Serial.println(autoSMV);
        String pieceC = String(pieceCount);
        writeFile(SD, "/41.txt", pieceC.c_str()); /*SD Save*/
        effi_cal();
      }
      accumulatedArea = 0;
      dialogBox("** Alert **", "Piece Synced", 1500);
    }

    // if (awaitButtonPress) {

    //   toleranceArea = trainedArea + (trainedArea * autoModeTolerance * 4);
    //   if (accumulatedArea < toleranceArea) {
    //     pieceCount++;  //// piece count approves only after pretrained area is reached

    //     String pieceC = String(pieceCount);
    //     writeFile(SD, "/41.txt", pieceC.c_str()); /*SD Save*/
    //     effi_cal();
    //   }

    //   awaitButtonPress = 0;
    //   accumulatedArea = 0;
    //   printStats();
    // }

    if (trainingCountDown != 0 && buttonPressCount > 1) {
      digitalWrite(ledPin, HIGH);
      if (accumulatedArea > 0.0) {
        trainedSMV = (int)(lastEdgeMillis - savedLastEdge);
        savedLastEdge = lastEdgeMillis;
        pieceCount++;  /// Take trainning piece as 1st count
        String pieceC = String(pieceCount);
        writeFile(SD, "/41.txt", pieceC.c_str()); /*SD Save*/
        effi_cal();
        trainingCountDown--;
        String trainCD = String(trainingCountDown);
        writeFile(SD, "/42.txt", trainCD.c_str()); /*SD Save*/

      } else {
        Serial.println("No trainned Area !");  /// throw 0 trainned Area error
        dialogBox("**Warning**", "No CT Signal !", 2000);
      }

      printStats();
      if (trainingCountDown == 0) {
        bool smvPass = (smv * 120000) > trainedSMV && (smv * 30000) < trainedSMV;
        if (smv == 0.0 || smvPass) {
          leds[2] = CRGB(0, 0, 0);
          FastLED.show();

          leds[3] = CRGB(0, 0, 0);
          FastLED.show();
          send_TrainingStatus(OperatorID, deviceId, "t");
          meter2();

          trainedArea = accumulatedArea / trainingNum;

          String trainnedA = String(trainedArea);
          writeFile(SD, "/44.txt", trainnedA.c_str()); /*SD Save*/

          String trainedSMV_M = String(trainedSMV);
          writeFile(SD, "/76.txt", trainedSMV_M.c_str()); /*SD Save*/  /// trainedSMV save to sd here              //// sd Save !!!!!!!!!!!!!!!

          accumulatedArea = 0;  /// reset at last
          Serial.println("\nTrainned Area,Trained SMV");
          Serial.println(trainedArea);
          Serial.println(trainedSMV);
        } else {
          Serial.println("SMV mismatch");
          dialogBox("**Warning**", "SMV Mismatch !", 2000);
          Serial.println(smv * 60000);
          Serial.println(trainedSMV);
          setTrainingMode(trainingNum);
          ////// dialogbox
        }

        pageShift1();
      }
    }
  }

  if (secCore == 0) {
    setIdleVpp(0);
    Serial.println("\nTrainned Area,Trained SMV");
    Serial.println(trainedArea);
    Serial.println(trainedSMV);
    Serial.println(buttonPressCount);
    Serial.println(trainingCountDown);
  }

  pushToBuffer(getVpp());

  // Print peak-to-peak voltage
  // Serial.print(avrVpp.get());
  // Serial.print(" ");
  // Serial.print(upperTrigger+idleVppAdj);
  // Serial.print(" ");
  // Serial.print(lowerTrigger+idleVppAdj);
  // Serial.print(" ");
  // Serial.println(idleVppAdj);

  //// check rising edge or falling edge
  int currentEdge = getEdge();

  if (currentEdge == -1) {  /// triggers on a falling edge and caluculate the needledown time
    accumulateAreaFlag = false;
    lastEdgeMillis = millis();

    if (awaitStop) {
      awaitStop = false;
      toleranceArea = trainedArea + (trainedArea * autoModeTolerance * 4);
      if (accumulatedArea < toleranceArea) {
        waitASec = true;
      } else {
        Serial.println("Access Area");
        dialogBox("** Alert **", "Wrong Piece!", 1500);
      }

      Serial.println(accumulatedArea);
      accumulatedArea = 0;
    }
  } else if (currentEdge == 1) {
    accumulateAreaFlag = true;
  }

  //accumulate area
  if (accumulateAreaFlag) {  //// awaitstop is 1 means a previous operation may not over yet.
    float cVpp = avrVpp.get() - idleVppAdj;
    if (cVpp < 0) cVpp = 0;
    accumulatedArea += cVpp;
    digitalWrite(ledPin, HIGH);
    areaReseterCounter = 0;
    if (awaitRisingEdge) {
      awaitRisingEdge = 0;
      areaResetTime = (millis() - lastEdgeMillis) / 1000;  /// divide by 1000 to make seconds and x2 to double = divide by 1000
      if (areaResetTime < 3) areaResetTime = 3;
      if (areaResetTime > 9) areaResetTime = 9;
      Serial.println("Area reset Time");
      Serial.println(areaResetTime);
      String areaResetTime_st = String(areaResetTime);
      writeFile(SD, "/85.txt", areaResetTime_st.c_str()); /*SD Save*/  /// trainedSMV save to sd here
      /// save areaResetTime
    }

  } else {
    digitalWrite(ledPin, LOW);  /// operation traking is indicated by led
    int ctCyclesPerSec = 1000 / ctThreadInterval;
    areaReseterCounter++;
    toleranceArea = trainedArea * 0.20;  /// inverse of autoModeTolarence
    bool smvPass = (millis() - lastPieceMillis) >= trainedSMV * 0.4;
    if (trainingCountDown == 0 && areaReseterCounter == (ctCyclesPerSec * areaResetTime) && accumulatedArea < toleranceArea) {
      if (!newPieceFlag) {
        newPieceFlag = 1;
        Serial.println("Reset Area @ ");
        Serial.println(areaResetTime);
        Serial.println(accumulatedArea);
        accumulatedArea = 0;
      }
    } else if (smvPass && waitASec && trainingCountDown == 0 && areaReseterCounter == ctCyclesPerSec * (areaResetTime / 3)) {
      waitASec = false;
      pieceCount++;
      accumulatedArea = 0;
      autoSMV = millis() - lastPieceMillis;
      if (autoSMV < (trainedSMV * 2)) avrAutoSMV.push(autoSMV);
      lastPieceMillis = millis();
      Serial.println(pieceCount);
      Serial.println(autoSMV);
      String pieceC = String(pieceCount);
      writeFile(SD, "/41.txt", pieceC.c_str()); /*SD Save*/
      effi_cal();
      //printStats();
    } else if (trainingCountDown == 0 && areaReseterCounter == (ctCyclesPerSec * 40)) {
      if (accumulatedArea < trainedArea * 0.3) {
        Serial.print("Reset Area :");
        Serial.println(accumulatedArea);
        accumulatedArea = 0;
        newPieceFlag = 1;
      } else if (accumulatedArea > trainedArea * 0.8) {
        pieceCount++;
        accumulatedArea = 0;
        Serial.println(pieceCount);
        dialogBox("** Alert **", "Piece Synced", 1500);
        String pieceC = String(pieceCount);
        writeFile(SD, "/41.txt", pieceC.c_str()); /*SD Save*/
        effi_cal();
      }
    }
  }

  toleranceArea = trainedArea - (trainedArea * autoModeTolerance);
  if (trainingCountDown == 0 && accumulatedArea > toleranceArea) {
    if (autoCountMode) {
      awaitStop = true;
      newPieceFlag = 0;
      // String accumulatedA = String(accumulatedArea);
      // writeFile(SD, "/43.txt", accumulatedA.c_str()); /*SD Save*/

    } else awaitButtonPress = true;  //// Accepts the button press after this area is reached
  }
}

int getEdge() {
  float minVal = vppBuffer[0];
  float maxVal = vppBuffer[0];
  int maxIndex = 0, minIndex = 0;

  // Find minimum and maximum values in the vppBuffer
  for (int i = 0; i < edgeBuffSize; i++) {
    if (vppBuffer[i] < minVal) {
      minVal = vppBuffer[i];
      minIndex = i;
    }
    if (vppBuffer[i] > maxVal) {
      maxVal = vppBuffer[i];
      maxIndex = i;
    }
  }

  int currentEdge = 0;
  // Check for rising edge
  if (avrVpp.get() > (upperTrigger + idleVppAdj)) currentEdge = 1;
  else if (avrVpp.get() < (lowerTrigger + idleVppAdj)) currentEdge = -1;
  else if (maxVal - minVal > difference) {
    if (minIndex < maxIndex) {  ////    ....../""""""   Rising edge
      currentEdge = 1;
    } else {  ////     """"""\.....     falling edge
      currentEdge = -1;
    }
  } else
    return 0;  // if difference is 0 its Flat

  if (currentEdge == lastEdge) return 0;  /// single broadcast
  else {
    lastEdge = currentEdge;
    return currentEdge;
  }
}

float getVpp() {
  // Initialize the first ADC value
  int maxVal = analogRead(adcPin);
  int minVal = maxVal;

  // Read ADC values, finding min and max on the fly
  for (int i = 1; i < vppBuffSize; i++) {
    int adcValue = analogRead(adcPin);

    if (adcValue > maxVal) {
      maxVal = adcValue;
    }
    if (adcValue < minVal) {
      minVal = adcValue;
    }

    delay(2);  // Adjust delay as needed based on sampling frequency
  }

  // Calculate peak-to-peak voltage
  float vpp = ((maxVal - minVal) / 4095.0) * 3.3;  // Assuming 12-bit ADC and 3.3V reference voltage
  avrVpp.push(vpp);
  return avrVpp.get();
}

void setIdleVpp(bool save) {
  float a = 0.0;
  for (int i = 0; i < 5; i++) {
    a += getVpp();
    delay(50);
  }
  a = a / 5.0;
  if (save) {
    idleVpp = a;
    idleVppAdj = a;
    String idleVpp_S = String(idleVpp);
    writeFile(SD, "/69.txt", idleVpp_S.c_str()); /*SD Save*/
  } else {
    if (a < idleVpp + 0.05 && a > idleVpp - 0.05) {
      idleVppAdj = a;
    } else idleVppAdj = idleVpp;
  }
  Serial.println("Idle Vpp, Adj");
  Serial.println(idleVpp);
  Serial.println(idleVppAdj);
}

void pushToBuffer(float value) {
  // If the buffer is full, shift values left
  if (currentIndex == edgeBuffSize) {
    // Shift values left by one position
    for (int i = 1; i < edgeBuffSize; i++) {
      vppBuffer[i - 1] = vppBuffer[i];
    }
    // Add the new value at the end
    vppBuffer[edgeBuffSize - 1] = value;
  } else {
    // If buffer is not full, simply add the value
    vppBuffer[currentIndex] = value;
    currentIndex++;
  }
}

void setTrainingMode(int tcd) {
  hmiButtonFlag = true;
  buttonPressCount = 0;
  trainedSMV = 0;     ////////////////////// save to card
  areaResetTime = 0;  ////////////////////// save to card int
  String areaResetTime_st = String(areaResetTime);
  writeFile(SD, "/85.txt", areaResetTime_st.c_str()); /*SD Save*/  /// trainedSMV save to sd here
  // save areaResetTime
  String trainedSMV_M = String(trainedSMV);
  writeFile(SD, "/76.txt", trainedSMV_M.c_str()); /*SD Save*/
  String buttonPress = String(buttonPressCount);
  writeFile(SD, "/40.txt", buttonPress.c_str()); /*SD Save*/
  trainingCountDown = tcd;

  String trainCD = String(trainingCountDown);
  writeFile(SD, "/42.txt", trainCD.c_str()); /*SD Save*/

  leds[2] = CRGB(0, 0, 255);
  FastLED.show();

  leds[3] = CRGB(0, 0, 255);
  FastLED.show();
  trainedArea = 0;
  String trainnedA = String(trainedArea);
  writeFile(SD, "/44.txt", trainnedA.c_str()); /*SD Save*/
  idleVpp = 0;
  String idleVpp_S = String(idleVpp);
  writeFile(SD, "/69.txt", idleVpp_S.c_str()); /*SD Save*/
}

void printStats() {
  Serial.println("------Statistics------");
  Serial.print("\nPiece count-");
  Serial.print(pieceCount);
  Serial.print("\nAccumulated Area-");
  Serial.print(accumulatedArea);
  Serial.println("\n--------------------");
  //delay(2500);
}


/**** Exact Hour Calculation****/
void onTheHour() {

  if (J == 0) {
    String HRstr = "1";
    send_text23(HRstr);
    J = 1;
    J = 1;
    String Jcount = String(J);
    writeFile(SD, "/46.txt", Jcount.c_str()); /*SD Save*/
    L = 0;
    String Lcount = String(L);
    writeFile(SD, "/47.txt", Lcount.c_str()); /*SD Save*/
    Hour_Z = 0;
    String HourZ = String(Hour_Z);
    writeFile(SD, "/70.txt", HourZ.c_str()); /*SD Save*/
    Done45 = 0;
    String done455 = String(Done45);
    writeFile(SD, "/79.txt", done455.c_str()); /*SD Save*/
  }

  currentMillisDHU = millis();
  time_t epochTime = timeClient.getEpochTime();
  struct tm *ptm = gmtime(&epochTime);

  currentMinute = ptm->tm_min;  // Get the current minute

  if (currentMinute % 5 == 0 && Min_X == 1) {

    smvAverageCal(avrAutoSMV.get());
    char formattedTime[20];
    strftime(formattedTime, sizeof(formattedTime), "%Y%m%d%H%M%S", ptm);
    String dateTimee = String(formattedTime);

    int monthDay = ptm->tm_mday;
    int currentMonth = ptm->tm_mon + 1;
    int currentYear = ptm->tm_year + 1900;

    // Ensure month and day are two digits
    String formattedMonth = (currentMonth < 10) ? "0" + String(currentMonth) : String(currentMonth);
    String formattedDay = (monthDay < 10) ? "0" + String(monthDay) : String(monthDay);

    String currentDate = String(currentYear) + "-" + formattedMonth + "-" + formattedDay;
    String Time = String(timeClient.getFormattedTime());
    String dateTime = currentDate + " " + Time;

    /*Node red*/

    set_ProductionData(OperatorID, deviceId, obbOperationId, RG, dateTime);

    Min_X = 0;
    Min_Y = 1;

    RG = 0;
    String RGcount = String(RG);
    writeFile(SD, "/68.txt", RGcount.c_str()); /*SD Save*/

    String MinX = String(Min_X);
    writeFile(SD, "/80.txt", MinX.c_str()); /*SD Save*/
    String MinY = String(Min_Y);
    writeFile(SD, "/81.txt", MinY.c_str()); /*SD Save*/
  }

  if (currentMinute % 5 != 0 && Min_Y == 1) {

    Min_X = 1;
    Min_Y = 0;
  }

  if ((currentMinute == 00) && (Hour_X == 1)) {
    Hour_X = 0;
    Hour_Y = 1;
    Hour_Z = 0;

    send_SMV(obbOperationId, OperatorID, smv_CAL, dateTime1);
    Efficiency();
    String HourX = String(Hour_X);
    writeFile(SD, "/77.txt", HourX.c_str()); /*SD Save*/
    String HourY = String(Hour_Y);
    writeFile(SD, "/78.txt", HourY.c_str()); /*SD Save*/
    String HourZ = String(Hour_Z);
    writeFile(SD, "/70.txt", HourZ.c_str()); /*SD Save*/
  }

  if ((currentMinute == 01) && (Hour_Y == 1)) {
    Hour_X = 1;
    Hour_Y = 0;
    String HourX = String(Hour_X);
    writeFile(SD, "/77.txt", HourX.c_str()); /*SD Save*/
    String HourY = String(Hour_Y);
    writeFile(SD, "/78.txt", HourY.c_str()); /*SD Save*/
  }

  if ((currentMinute > 01) && (currentMinute < 45) && (Hour_Z == 1)) {
    Done45 = 0;
    Hour_Z = 0;


    send_SMV(obbOperationId, OperatorID, smv_CAL, dateTime1);
    Efficiency();
    N = 0;
    int num2 = N;
    String str2 = String(num2);
    send_text4(str2);
    send_text4(str2);
    String done455 = String(Done45);
    writeFile(SD, "/79.txt", done455.c_str()); /*SD Save*/
    String HourZ = String(Hour_Z);
    writeFile(SD, "/70.txt", HourZ.c_str()); /*SD Save*/
  }

  if ((currentMinute > 45) && (Done45 == 0)) {
    Done45 = 1;
    Hour_Z = 1;
    String HourZ = String(Hour_Z);
    writeFile(SD, "/70.txt", HourZ.c_str()); /*SD Save*/
    String done455 = String(Done45);
    writeFile(SD, "/79.txt", done455.c_str()); /*SD Save*/
  }

  if ((currentMillisDHU - previousMillisDHUMain >= twelveMinuteInterval) && (DHU_X == 1)) {

    previousMillisDHUMain = currentMillisDHU;
    previousMillisDHUSecondary = currentMillisDHU;

    awaitReply = 1;
    get_DHU(OperatorID, obbOperationId, deviceId, part);
    for (int d = 0; d < 50; d++) {
      if (!awaitReply) d = 50;
      delay(100);
    }
    Serial.println(DHU);
    DHU_X = 0;
    DHU_Y = 1;
  }

  if ((currentMillisDHU - previousMillisDHUSecondary >= oneMinuteInterval) && (DHU_Y == 1)) {

    DHUfunction();
    String DHUs = String(DHU);
    writeFile(SD, "/84.txt", DHUs.c_str()); /*SD Save*/
    DHU_Y = 0;
    DHU_X = 1;
  }
}

/**** Efficiency Calculation for Internal Meter ****/
void Efficiency() {

  L++;
  proTar++;

  String Lcount = String(L);
  String proTarcount = String(proTar);
  writeFile(SD, "/47.txt", Lcount.c_str());      /*SD Save*/
  writeFile(SD, "/48.txt", proTarcount.c_str()); /*SD Save*/

  Lvalue = (L + 1);
  String numHRstr = String(Lvalue);
  send_text23(numHRstr);
  send_text23(numHRstr);

  meter1();

  if (E_RGB_LOW >= t) {

    leds[2] = CRGB(255, 0, 0);
    FastLED.show();

    leds[3] = CRGB(255, 0, 0);
    FastLED.show();
  }
  if (E_RGB_HIGH > t && E_RGB_LOW < t) {

    leds[2] = CRGB(255, 165, 0);
    FastLED.show();

    leds[3] = CRGB(255, 165, 0);
    FastLED.show();
  }
  if (E_RGB_HIGH <= t) {

    leds[2] = CRGB(0, 255, 0);
    FastLED.show();

    leds[3] = CRGB(0, 255, 0);
    FastLED.show();
  }
  if (trainingCountDown != 0) {

    leds[2] = CRGB(0, 0, 255);
    FastLED.show();

    leds[3] = CRGB(0, 0, 255);
    FastLED.show();
  }

  N = 0;
  N = 0;
  N = 0;

  int num2 = N;
  String str2 = String(num2);
  send_text4(str2);
  send_text4(str2);
  send_text4(str2);
  send_text4(str2);
  writeFile(SD, "/49.txt", str2.c_str()); /*SD Save*/
}

/**** Efficiency Pre Calculation ****/
void effi_cal() {

  G++;
  LG++;
  RG++;

  String Gcount = String(G);
  String LGcount = String(LG);
  String RGcount = String(RG);
  writeFile(SD, "/52.txt", Gcount.c_str());  /*SD Save*/
  writeFile(SD, "/53.txt", LGcount.c_str()); /*SD Save*/
  writeFile(SD, "/68.txt", RGcount.c_str()); /*SD Save*/

  formattedCount = String(G);
  while (formattedCount.length() < 6) {
    formattedCount = "0" + formattedCount;  // Prepend zeros until the length is 4
  }
  send_text27(formattedCount);
  send_text27(formattedCount);

  pcsRe = 1;  // Efficency time loop
  N++;

  int num2 = N;
  String str2 = String(num2);
  send_text4(str2);
  send_text4(str2);

  writeFile(SD, "/49.txt", str2.c_str()); /*SD Save*/
}

/**** SD Card Functions ****/
void appendFile(fs::FS &fs, const char *path, const char *message) {  // SD card append file
  digitalWrite(SS_SD, LOW);
  digitalWrite(SS_PIN, HIGH);
  Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    Serial.println("Failed to open file for appending");
    return;
  }
  if (file.print(message)) {
    Serial.println("Message appended");
  } else {
    Serial.println("Append failed");
  }
  file.close();
  digitalWrite(SS_SD, HIGH);
}

void readFile(fs::FS &fs, const char *path) {  // SD card read file
  digitalWrite(SS_SD, LOW);
  digitalWrite(SS_PIN, HIGH);
  Serial.printf("Reading file: %s\n", path);
  File file = fs.open(path);
  if (!file) {
    Serial.println("Failed to open file for reading");
    ReadDone = 0;
    return;
  }

  while (file.available()) {

    rdline = file.readStringUntil('\n');
    ReadDone = 1;
  }
  file.close();
  digitalWrite(SS_SD, HIGH);
}

void deleteFile(fs::FS &fs, const char *path) {  // SD card delete file
  digitalWrite(SS_SD, LOW);
  digitalWrite(SS_PIN, HIGH);
  Serial.printf("Deleting file: %s\n", path);
  if (fs.remove(path)) {
  } else {
  }
  digitalWrite(SS_SD, HIGH);
}

void writeFile(fs::FS &fs, const char *path, const char *message) {  // SD card write file
  digitalWrite(SS_SD, LOW);
  digitalWrite(SS_PIN, HIGH);

  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    return;
  }
  if (file.print(message)) {
  } else {
  }
  file.close();
  digitalWrite(SS_SD, HIGH);
}


/**** Node Red Functions ****/
void callback(char *topic, byte *payload, unsigned int length) {
  Serial.println("Call back ");

  // Create a larger buffer to handle more complex JSON objects
  DynamicJsonDocument doc(1024);  // Increase if necessary for bigger JSON objects
  DeserializationError error = deserializeJson(doc, payload, length);
  if (error) {
    Serial.print("Failed to deserialize JSON: ");
    Serial.println(error.c_str());
    return;
  }

  // Ensure the deserialized document is a JSON object
  if (!doc.is<JsonObject>()) {
    Serial.println("Received JSON is not a valid object");
    return;
  }

  // Iterate over the JSON object to process each key-value pair
  JsonObject obj = doc.as<JsonObject>();

  for (JsonPair kvp : obj) {
    String key = kvp.key().c_str();  // Get the key
    Serial.print(key);
    Serial.print(": ");
    digitalWrite(SS_SD, LOW);
    awaitReply = 0;
    fnCall_initial_data = false;

    // Determine the type of the value and handle accordingly
    if (kvp.value().is<int>()) {
      int intValue = kvp.value().as<int>();
      Serial.println(intValue);

      if (key == "spi") {
        spi = kvp.value().as<int>();

      } else if (key == "workingHours") {
        workingHours = kvp.value().as<int>();

      } else if (key == "efficiencyLevel1") {
        efficiencyLevel1 = kvp.value().as<int>();

      } else if (key == "efficiencyLevel3") {
        efficiencyLevel3 = kvp.value().as<int>();

      } else if (key == "target") {
        target = kvp.value().as<int>();
      }

    } else if (kvp.value().is<float>()) {
      float floatValue = kvp.value().as<float>();
      Serial.println(floatValue);

      if (key == "smv") {
        smv = kvp.value().as<float>();
      }


    } else if (kvp.value().is<String>()) {
      String stringValue = kvp.value().as<String>();
      Serial.println(stringValue);

      if (key == "qi") {
        qualityInsName = kvp.value().as<String>();
        dataReceived_initial_data = true;

      } else if (key == "DHU") {
        DHU_1 = kvp.value().as<String>();
        dataReceived_DHU = true;

        DHU = DHU_1.toFloat();

      } else if (key == "result") {
        result = kvp.value().as<String>();
        dataReceived_result = true;

      } else if (key == "logStatus") {
        logStatus = kvp.value().as<String>();

      } else if (key == "OPemployeeId") {
        OPemployeeId = kvp.value().as<String>();
        dataReceived_OPemployeeId = true;

      } else if (key == "colourTLS") {
        colourTLS = kvp.value().as<String>();
        dataReceived_colour = true;

      } else if (key == "machineId") {
        machineId = kvp.value().as<String>();

      } else if (key == "machineType") {
        machineType = kvp.value().as<String>();

      } else if (key == "ownership") {
        ownership = kvp.value().as<String>();

      } else if (key == "activeObbOperationId") {
        obbOperationId = kvp.value().as<String>();

      } else if (key == "style") {
        style = kvp.value().as<String>();

      } else if (key == "su") {
        supervisorName = kvp.value().as<String>();

      } else if (key == "SuRFID") {
        supervisorRFID = kvp.value().as<String>();

      } else if (key == "me") {
        mechanicName = kvp.value().as<String>();

      } else if (key == "MeEmployeeId") {
        MeEmployeeId = kvp.value().as<String>();

      } else if (key == "SuEmployeeId") {
        SuEmployeeId = kvp.value().as<String>();

      } else if (key == "MeRFID") {
        mechanicRFID = kvp.value().as<String>();

      } else if (key == "operationName") {
        operationName = kvp.value().as<String>();

      } else if (key == "part") {
        part = kvp.value().as<String>();

      } else if (key == "productionLine") {
        productionLine = kvp.value().as<String>();

      } else if (key == "update") {
        performOTA(kvp.value().as<String>());
      } else if (key == "errormsg") {
          dialogBox("** Alert **", "  NO OBB  ", 60000);
        }

    } else {
      Serial.println("Unsupported data type");
    }
    digitalWrite(SS_SD, HIGH);
  }
}

/*** connecting to a mqtt broker ***/
void MQTT_Connect() {
  espClient.setCACert(root_ca);
  client.setServer(mqtt_broker, mqtt_port);
  client.setCallback(callback);

  while ((!client.connected()) && (wifiData == 2)) {
    String client_id = "esp32-client-";
    client_id += String(WiFi.macAddress());

    Serial.printf("The client %s connects to the public MQTT broker\n", client_id.c_str());


    if (client.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
      Serial.println("MQTT broker connected");
    } else {
      Serial.print("failed with state ");
      Serial.print(client.state());
      delay(2000);
    }
  }
  client.subscribe(recv_topic);
}

void MQTT_Reconnect() {
  // Check if Wi-Fi is connected before attempting MQTT reconnection
  if (wifiData == 2) {
    while (!client.connected()) {
      MQTT_Connect();  // Reconnect to MQTT broker if not connected
      delay(200);
    }
    client.loop();  // Maintain MQTT connection
  } else {
    Serial.println("WiFi not connected. Skipping MQTT reconnect.");
  }
}


/**** SET Functions ****/
//Data which need to be sent to the API
void set_AlertLog(String verifiedKey, String machineId, String operatorRfid, String employeeId, String alertType) {

  StaticJsonDocument<100> jsonDocument;
  jsonDocument["verifiedKey"] = verifiedKey;
  jsonDocument["machineId"] = machineId;
  jsonDocument["operatorRfid"] = operatorRfid;
  jsonDocument["employeeId"] = employeeId;
  jsonDocument["alertType"] = alertType;
  jsonDocument["method"] = "API";

  char jsonBuffer[128];
  serializeJson(jsonDocument, jsonBuffer);

  appendToPublish(jsonBuffer);
  Serial.println("Published data: ");
  Serial.print("verifiedKey : ");
  Serial.println(verifiedKey);
  Serial.print("machineId : ");
  Serial.println(machineId);
  Serial.print("operatorRfid : ");
  Serial.println(operatorRfid);
  Serial.print("employeeId : ");
  Serial.println(employeeId);
  Serial.print("alertType : ");
  Serial.println(alertType);
}


//Data which need to be sent to the API
void set_SessionId(String SessionId) {

  StaticJsonDocument<100> jsonDocument;
  jsonDocument["SessionId"] = SessionId;
  jsonDocument["method"] = "API";

  char jsonBuffer[256];
  serializeJson(jsonDocument, jsonBuffer);

  appendToPublish(jsonBuffer);
  Serial.println("Published data: ");
  Serial.print("SessionId : ");
  Serial.println(SessionId);
}


//Data which need to be sent to the API (PUT)
void set_LogStatus(String verifiedKey, String machineId, String employeeId, String reqType) {

  StaticJsonDocument<100> jsonDocument;
  jsonDocument["verifiedKey"] = verifiedKey;
  jsonDocument["machineId"] = machineId;
  jsonDocument["employeeId"] = employeeId;
  jsonDocument["reqType"] = reqType;
  jsonDocument["method"] = "PUT";

  char jsonBuffer[128];
  serializeJson(jsonDocument, jsonBuffer);

  appendToPublish(jsonBuffer);
  Serial.println("Published data: ");
  Serial.print("verifiedKey : ");
  Serial.println(verifiedKey);
  Serial.print("machineId : ");
  Serial.println(machineId);
  Serial.print("employeeId : ");
  Serial.println(employeeId);
  Serial.print("reqType : ");
  Serial.println(reqType);
}

//SET data in OperatorSession, if isLoggedIn status is 0
void set_operatorSession(String id, String operatorRfid, String obbOperationId, String isLoggedIn, String LoginTimestamp) {

  StaticJsonDocument<100> jsonDocument;
  jsonDocument["id"] = id;
  jsonDocument["operatorRfid"] = operatorRfid;
  jsonDocument["obbOperationId"] = obbOperationId;
  jsonDocument["isLoggedIn"] = isLoggedIn;  // use only 't' or 'f'
  jsonDocument["LoginTimestamp"] = LoginTimestamp;
  jsonDocument["method"] = "LOG0";

  char jsonBuffer[256];
  serializeJson(jsonDocument, jsonBuffer);

  appendToPublish(jsonBuffer);
  Serial.println("Published data: ");
  Serial.print("id : ");
  Serial.println(id);
  Serial.print("operatorRfid : ");
  Serial.println(operatorRfid);
  Serial.print("obbOperationId : ");
  Serial.println(obbOperationId);
  Serial.print("isLoggedIn : ");
  Serial.println(isLoggedIn);
  Serial.print("LoginTimestamp : ");
  Serial.println(LoginTimestamp);
}

//Append data to OperatorSession, if isLoggedIn status is 1
void append_operatorSession(String id, String LogoutTimestamp) {

  StaticJsonDocument<100> jsonDocument;
  jsonDocument["id"] = id;
  jsonDocument["LogoutTimestamp"] = LogoutTimestamp;
  jsonDocument["method"] = "LOG1";


  char jsonBuffer[128];
  serializeJson(jsonDocument, jsonBuffer);

  appendToPublish(jsonBuffer);
  Serial.println("Published data: ");
  Serial.print("id : ");
  Serial.println(id);
  Serial.print("LogoutTimestamp : ");
  Serial.println(LogoutTimestamp);
}

//SET data in staffSession, if isLoggedIn status is 0
void set_staffSession(String id, String staffEmpId, String obbOperationId, String isLoggedIn, String LoginTimestamp) {

  StaticJsonDocument<100> jsonDocument;
  jsonDocument["id"] = id;
  jsonDocument["staffEmpId"] = staffEmpId;
  jsonDocument["obbOperationId"] = obbOperationId;
  jsonDocument["isLoggedIn"] = isLoggedIn;  // use only 't' or 'f'
  jsonDocument["LoginTimestamp"] = LoginTimestamp;
  jsonDocument["method"] = "WRITE";

  char jsonBuffer[256];
  serializeJson(jsonDocument, jsonBuffer);

  appendToPublish(jsonBuffer);
  Serial.println("Published data: ");
  Serial.print("id : ");
  Serial.println(id);
  Serial.print("staffEmpId : ");
  Serial.println(staffEmpId);
  Serial.print("obbOperationId : ");
  Serial.println(obbOperationId);
  Serial.print("isLoggedIn : ");
  Serial.println(isLoggedIn);
  Serial.print("LoginTimestamp : ");
  Serial.println(LoginTimestamp);
}

//Append data to staffSession, if isLoggedIn status is 1
void append_staffSession(String id, String LogoutTimestamp) {

  StaticJsonDocument<100> jsonDocument;
  jsonDocument["id"] = id;
  jsonDocument["LogoutTimestamp"] = LogoutTimestamp;
  jsonDocument["method"] = "APPEND";


  char jsonBuffer[128];
  serializeJson(jsonDocument, jsonBuffer);

  appendToPublish(jsonBuffer);
  Serial.println("Published data: ");
  Serial.print("id : ");
  Serial.println(id);
  Serial.print("LogoutTimestamp : ");
  Serial.println(LogoutTimestamp);
}

//SET in ProductionData
void set_ProductionData(String operatorRfid, String eliotSerialNumber, String obbOperationId, int productionCount, String timestamp) {

  StaticJsonDocument<100> jsonDocument;
  jsonDocument["operatorRfid"] = operatorRfid;
  jsonDocument["eliotSerialNumber"] = eliotSerialNumber;
  jsonDocument["obbOperationId"] = obbOperationId;
  jsonDocument["productionCount"] = productionCount;
  jsonDocument["timestamp"] = timestamp;
  jsonDocument["method"] = "PD";

  char jsonBuffer[256];
  serializeJson(jsonDocument, jsonBuffer);

  appendToPublish(jsonBuffer);
  Serial.println("Published data: ");
  Serial.print("operatorRfid : ");
  Serial.println(operatorRfid);
  Serial.print("eliotSerialNumber : ");
  Serial.println(eliotSerialNumber);
  Serial.print("obbOperationId : ");
  Serial.println(obbOperationId);
  Serial.print("productionCount : ");
  Serial.println(productionCount);
  Serial.print("timestamp : ");
  Serial.println(timestamp);
}


//Append 't' to the isloggedin column in Operator table
void append_operator_t(String rfid) {

  StaticJsonDocument<100> jsonDocument;
  jsonDocument["rfid"] = rfid;
  jsonDocument["method"] = "OPt";


  char jsonBuffer[128];
  serializeJson(jsonDocument, jsonBuffer);

  appendToPublish(jsonBuffer);
  Serial.println("Published data: ");
  Serial.print("rfid : ");
  Serial.println(rfid);
}

//Append 'f' to the isloggedin column in Operator table
void append_operator_f(String rfid) {

  StaticJsonDocument<100> jsonDocument;
  jsonDocument["rfid"] = rfid;
  jsonDocument["method"] = "OPf";


  char jsonBuffer[128];
  serializeJson(jsonDocument, jsonBuffer);

  appendToPublish(jsonBuffer);
  Serial.println("Published data: ");
  Serial.print("rfid : ");
  Serial.println(rfid);
}

// Send Operator login timestamp
void send_loginTS(String opRFID, String loginTS) {

  StaticJsonDocument<100> jsonDocument;
  jsonDocument["opRFID"] = opRFID;
  jsonDocument["loginTS"] = loginTS;
  jsonDocument["method"] = "LINTS";

  char jsonBuffer[256];
  serializeJson(jsonDocument, jsonBuffer);

  appendToPublish(jsonBuffer);  // Change the QoS parameter to 2
  Serial.println("Published data: ");
  Serial.print("OperatorRFID : ");
  Serial.println(opRFID);
  Serial.print("Login Timestamp : ");
  Serial.println(loginTS);
}

// Send Operator logout timestamp
void send_logoutTS(String opRFID, String logoutTS) {

  StaticJsonDocument<100> jsonDocument;
  jsonDocument["opRFID"] = opRFID;
  jsonDocument["logoutTS"] = logoutTS;
  jsonDocument["method"] = "LOUTTS";

  char jsonBuffer[256];
  serializeJson(jsonDocument, jsonBuffer);

  appendToPublish(jsonBuffer);  // Change the QoS parameter to 2
  Serial.println("Published data: ");
  Serial.print("OperatorRFID : ");
  Serial.println(opRFID);
  Serial.print("Logout Timestamp : ");
  Serial.println(logoutTS);
}

// Send supervisor break start timestamp
void send_SUBreakStartTS(String opRFID, String bStartTime) {

  StaticJsonDocument<100> jsonDocument;
  jsonDocument["opRFID"] = opRFID;
  jsonDocument["bStartTime"] = bStartTime;
  jsonDocument["method"] = "BSTAT";
  jsonDocument["bt"] = "SU";

  char jsonBuffer[256];
  serializeJson(jsonDocument, jsonBuffer);

  appendToPublish(jsonBuffer);  // Change the QoS parameter to 2
  Serial.println("Published data: ");
  Serial.print("OperatorRFID : ");
  Serial.println(opRFID);
  Serial.print("Break start Timestamp : ");
  Serial.println(bStartTime);
}

// Send supervisor break end timestamp
void send_SUBreakEndTS(String opRFID, String bEndTime) {

  StaticJsonDocument<100> jsonDocument;
  jsonDocument["opRFID"] = opRFID;
  jsonDocument["bEndTime"] = bEndTime;
  jsonDocument["method"] = "BEND";
  jsonDocument["bt"] = "SU";

  char jsonBuffer[256];
  serializeJson(jsonDocument, jsonBuffer);

  appendToPublish(jsonBuffer);  // Change the QoS parameter to 2
  Serial.println("Published data: ");
  Serial.print("OperatorRFID : ");
  Serial.println(opRFID);
  Serial.print("Break end Timestamp : ");
  Serial.println(bEndTime);
}

// Send mechanic break start timestamp
void send_MEBreakStartTS(String opRFID, String bStartTime) {

  StaticJsonDocument<100> jsonDocument;
  jsonDocument["opRFID"] = opRFID;
  jsonDocument["bStartTime"] = bStartTime;
  jsonDocument["method"] = "BSTAT";
  jsonDocument["bt"] = "ME";

  char jsonBuffer[256];
  serializeJson(jsonDocument, jsonBuffer);

  appendToPublish(jsonBuffer);  // Change the QoS parameter to 2
  Serial.println("Published data: ");
  Serial.print("OperatorRFID : ");
  Serial.println(opRFID);
  Serial.print("Break start Timestamp : ");
  Serial.println(bStartTime);
}

// Send mechanic break end timestamp
void send_MEBreakEndTS(String opRFID, String bEndTime) {

  StaticJsonDocument<100> jsonDocument;
  jsonDocument["opRFID"] = opRFID;
  jsonDocument["bEndTime"] = bEndTime;
  jsonDocument["method"] = "BEND";
  jsonDocument["bt"] = "ME";

  char jsonBuffer[256];
  serializeJson(jsonDocument, jsonBuffer);

  appendToPublish(jsonBuffer);  // Change the QoS parameter to 2
  Serial.println("Published data: ");
  Serial.print("OperatorRFID : ");
  Serial.println(opRFID);
  Serial.print("Break end Timestamp : ");
  Serial.println(bEndTime);
}

// Send lunch break start timestamp
void send_LunchBreakStartTS(String opRFID, String bStartTime) {

  StaticJsonDocument<100> jsonDocument;
  jsonDocument["opRFID"] = opRFID;
  jsonDocument["bStartTime"] = bStartTime;
  jsonDocument["method"] = "BSTAT";
  jsonDocument["bt"] = "LB";

  char jsonBuffer[256];
  serializeJson(jsonDocument, jsonBuffer);

  appendToPublish(jsonBuffer);  // Change the QoS parameter to 2
  Serial.println("Published data: ");
  Serial.print("OperatorRFID : ");
  Serial.println(opRFID);
  Serial.print("Break start Timestamp : ");
  Serial.println(bStartTime);
}

// Send lunch break end timestamp
void send_LunchBreakEndTS(String opRFID, String bEndTime) {

  StaticJsonDocument<100> jsonDocument;
  jsonDocument["opRFID"] = opRFID;
  jsonDocument["bEndTime"] = bEndTime;
  jsonDocument["method"] = "BEND";
  jsonDocument["bt"] = "LB";

  char jsonBuffer[256];
  serializeJson(jsonDocument, jsonBuffer);

  appendToPublish(jsonBuffer);  // Change the QoS parameter to 2
  Serial.println("Published data: ");
  Serial.print("OperatorRFID : ");
  Serial.println(opRFID);
  Serial.print("Break end Timestamp : ");
  Serial.println(bEndTime);
}

// Send offstand start timestamp
void send_OffstandStartTS(String opRFID, String bStartTime) {

  StaticJsonDocument<100> jsonDocument;
  jsonDocument["opRFID"] = opRFID;
  jsonDocument["bStartTime"] = bStartTime;
  jsonDocument["method"] = "BSTAT";
  jsonDocument["bt"] = "OS";

  char jsonBuffer[256];
  serializeJson(jsonDocument, jsonBuffer);

  appendToPublish(jsonBuffer);  // Change the QoS parameter to 2
  Serial.println("Published data: ");
  Serial.print("OperatorRFID : ");
  Serial.println(opRFID);
  Serial.print("Break start Timestamp : ");
  Serial.println(bStartTime);
}

// Send offstand end timestamp
void send_OffstandEndTS(String opRFID, String bEndTime) {

  StaticJsonDocument<100> jsonDocument;
  jsonDocument["opRFID"] = opRFID;
  jsonDocument["bEndTime"] = bEndTime;
  jsonDocument["method"] = "BEND";
  jsonDocument["bt"] = "OS";

  char jsonBuffer[256];
  serializeJson(jsonDocument, jsonBuffer);

  appendToPublish(jsonBuffer);  // Change the QoS parameter to 2
  Serial.println("Published data: ");
  Serial.print("OperatorRFID : ");
  Serial.println(opRFID);
  Serial.print("Break end Timestamp : ");
  Serial.println(bEndTime);
}

// Send supervisor break end timestamp
void send_SMV(String operationID, String operatorID, float SMV, String timestamp) {

  StaticJsonDocument<100> jsonDocument;
  jsonDocument["operationID"] = operationID;
  jsonDocument["operatorID"] = operatorID;
  jsonDocument["SMV"] = SMV;
  jsonDocument["timestamp"] = timestamp;
  jsonDocument["method"] = "SMV";

  char jsonBuffer[128];
  serializeJson(jsonDocument, jsonBuffer);

  appendToPublish(jsonBuffer);  // Change the QoS parameter to 2
  Serial.println("Published data: ");
  Serial.print("operationID : ");
  Serial.println(operationID);
  Serial.print("operatorID : ");
  Serial.println(operatorID);
  Serial.print("SMV : ");
  Serial.println(SMV);
}

// Send training status
void send_TrainingStatus(String opRFID, String deviceID, String st) {

  StaticJsonDocument<100> jsonDocument;
  jsonDocument["opRFID"] = opRFID;
  jsonDocument["deviceID"] = deviceID;
  jsonDocument["status"] = st;
  jsonDocument["method"] = "TRAIN";

  char jsonBuffer[256];
  serializeJson(jsonDocument, jsonBuffer);

  appendToPublish(jsonBuffer);  // Change the QoS parameter to 2
  Serial.println("Published data: ");
  Serial.print("OperatorRFID : ");
  Serial.println(opRFID);
  Serial.print("Device ID : ");
  Serial.println(deviceID);
}

/**** GET Functions ****/

// Getting initial data at the beginning
void get_initial_data(String serial_number) {
  //MQTT_Reconnect();

  StaticJsonDocument<100> jsonDocument;
  jsonDocument["serialNumber"] = serial_number;
  jsonDocument["method"] = "GET";

  char jsonBuffer[128];
  serializeJson(jsonDocument, jsonBuffer);

  appendToPublish(jsonBuffer);

  Serial.println("Retrieving data...");
}

// Get DHU
void get_DHU(String opRFID, String obbOperationId, String deviceId, String part) {
  StaticJsonDocument<100> jsonDocument;
  jsonDocument["opRFID"] = opRFID;
  jsonDocument["obbOperationId"] = obbOperationId;
  jsonDocument["deviceId"] = deviceId;
  jsonDocument["part"] = part;
  jsonDocument["method"] = "DHU";

  char jsonBuffer[256];
  serializeJson(jsonDocument, jsonBuffer);

  appendToPublish(jsonBuffer);
  fnCall_DHU = false;

  Serial.println("Published data: ");
  Serial.print("opRFID : ");
  Serial.println(opRFID);
  Serial.print("obbOperationId : ");
  Serial.println(obbOperationId);
  Serial.print("deviceId : ");
  Serial.println(deviceId);
  Serial.print("part : ");
  Serial.println(part);
}

// send operator rfid to check isLoggedin status
void send_OperatorRFID(String rfid, String deviceId) {
  StaticJsonDocument<100> jsonDocument;
  jsonDocument["OPCode"] = rfid;
  jsonDocument["deviceId"] = deviceId;
  jsonDocument["method"] = "OP";

  char jsonBuffer[256];
  serializeJson(jsonDocument, jsonBuffer);

  appendToPublish(jsonBuffer);
  Serial.println("Published data: ");
  Serial.print("OperatorRFID : ");
  Serial.println(rfid);
  fnCall_RFIDresult = false;
}

// Getting Operator EmployeeId by sending Operator RFID
void get_employeeId(String rfid, String deviceId) {

  StaticJsonDocument<100> jsonDocument;
  jsonDocument["rfid"] = rfid;
  jsonDocument["deviceId"] = deviceId;
  jsonDocument["method"] = "GET_EM";

  char jsonBuffer[256];
  serializeJson(jsonDocument, jsonBuffer);

  appendToPublish(jsonBuffer);
  Serial.println("Published data: ");
  Serial.print("OperatorRFID : ");
  Serial.println(rfid);
  fnCall_employeeId = false;
}

// Getting traffic light colour
void get_light_colour(String obbOperationId, int roundNo, String date, String deviceId) {

  StaticJsonDocument<100> jsonDocument;
  jsonDocument["obbOperationId"] = obbOperationId;
  jsonDocument["roundNo"] = roundNo;
  jsonDocument["date"] = date;
  jsonDocument["deviceId"] = deviceId;
  jsonDocument["method"] = "COLOUR";

  char jsonBuffer[128];
  serializeJson(jsonDocument, jsonBuffer);

  appendToPublish(jsonBuffer);
  fnCall_colour = false;

  Serial.println("Retrieving light colour...");
}

/**** Sorting algorithm for RFID ****/
int getPrefixNumber(String str) {
  String prefix = str.substring(0, 2);  // Extract the first two characters

  if (prefix == "OP") {
    sortRFID = 1;
  } else if (prefix == "SU") {
    sortRFID = 2;
  } else if (prefix == "ME") {
    sortRFID = 3;
  }

  return 0;  // Return 0 if none of the cases match (optional)
}



/**** New ID generating function ****/
String encodeDateTime(String dateTime) {
  return customEncode(dateTime);
}

String encodeDeviceId(String id) {
  int index = id.indexOf('-');
  String numericPart = id.substring(index + 1);
  return customEncode(numericPart);  // Only encode the numeric part
}

String customEncode(String input) {
  String encoded = "";
  for (int i = 0; i < input.length(); i++) {
    char c = input.charAt(i);
    int baseIndex = (c >= '0' && c <= '9') ? '0' : (c >= 'A' && c <= 'Z') ? 'A' - 10
                                                                          : 'a' - 36;
    int offset = (c - baseIndex + i * 3) % 62;  // Rotate and mix a bit
    if (offset < 10) {
      encoded += char('0' + offset);
    } else if (offset < 36) {
      encoded += char('A' + offset - 10);
    } else {
      encoded += char('a' + offset - 36);
    }
  }
  return encoded;
}

String generateRandomString(int length) {
  String randomString = "";
  const char *characters = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789";
  for (int i = 0; i < length; i++) {
    int randomCharIndex = random(strlen(characters));
    randomString += characters[randomCharIndex];
  }
  return randomString;
}

void startTrafficLight() {
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);

  leds[0] = CRGB(255, 0, 0);
  FastLED.show();
  leds[1] = CRGB(255, 0, 0);
  FastLED.show();
  leds[2] = CRGB(255, 0, 0);
  FastLED.show();
  leds[3] = CRGB(255, 0, 0);
  FastLED.show();
  delay(1000);

  leds[0] = CRGB(0, 255, 0);
  FastLED.show();
  leds[1] = CRGB(0, 255, 0);
  FastLED.show();
  leds[2] = CRGB(0, 255, 0);
  FastLED.show();
  leds[3] = CRGB(0, 255, 0);
  FastLED.show();
  delay(1000);

  leds[0] = CRGB(0, 0, 255);
  FastLED.show();
  leds[1] = CRGB(0, 0, 255);
  FastLED.show();
  leds[2] = CRGB(0, 0, 255);
  FastLED.show();
  leds[3] = CRGB(0, 0, 255);
  FastLED.show();
  delay(1000);

  leds[0] = CRGB(0, 0, 0);
  FastLED.show();
  leds[1] = CRGB(0, 0, 0);
  FastLED.show();
  leds[2] = CRGB(0, 0, 0);
  FastLED.show();
  leds[3] = CRGB(0, 0, 0);
  FastLED.show();
  delay(500);
}

void smvAverageCal(int newValue) {
  // Shift the array to the left if we already have 10 values
  if (avrSMVBuffIndex == 10) {
    for (int i = 1; i < 10; i++) {
      avrSMVBuff[i - 1] = avrSMVBuff[i];
    }
    avrSMVBuff[9] = newValue;  // Add the new value to the end
  } else {
    avrSMVBuff[avrSMVBuffIndex] = newValue;
    avrSMVBuffIndex++;
  }

  // Calculate the average of the current avrSMVBuff
  int sum = 0;
  for (int i = 0; i < avrSMVBuffIndex; i++) {
    sum += avrSMVBuff[i];
  }

  // Store the average in avrSMV_CT
  smv_CT = sum / avrSMVBuffIndex;
  smv_CAL = float(smv_CT) / float(60000);

  String SMV_CALC = String(smv_CAL);
  writeFile(SD, "/75.txt", SMV_CALC.c_str()); /*SD Save*/

  Serial.print("Average smv:");
  Serial.println(smv_CAL);
}

void checkInternetConnectivity() {
  WiFiClient client;

  // Attempt to connect to Google's DNS server on port 53
  if (client.connect("8.8.8.8", 53)) {
    wifiData = 2;
    netConCheck = 1;  // Connected to the internet
    client.stop();    // Close the connection
  } else {
    wifiData = 1;
    netConCheck = 2;  // Not connected to the internet
  }
}

void DHUfunction() {

  String GD = String(DHU);
  send_text26(GD);
  send_text26(GD);

  if ((0.00 <= DHU) && (DHU <= 2.00)) {

    leds[0] = CRGB(0, 255, 0);
    FastLED.show();

    leds[1] = CRGB(0, 255, 0);
    FastLED.show();

    Serial.println("################# GREEN ################");
    Serial.println(DHU);

  } else if ((2.00 < DHU) && (DHU <= 3.00)) {

    leds[0] = CRGB(255, 165, 0);
    FastLED.show();

    leds[1] = CRGB(255, 165, 0);
    FastLED.show();

    Serial.println("################# YELLOW ################");
    Serial.println(DHU);

  } else if (3.00 < DHU) {

    leds[0] = CRGB(255, 0, 0);
    FastLED.show();

    leds[1] = CRGB(255, 0, 0);
    FastLED.show();

    Serial.println("################# RED ##################");
    Serial.println(DHU);
  }
}


void performOTA(String addr) {
  if (WiFi.status() == WL_CONNECTED) {  // Check if connected to WiFi
    HTTPClient http;
    http.begin(addr);  // Local server URL for bin file

    int httpCode = http.GET();
    if (httpCode == HTTP_CODE_OK) {
      int contentLength = http.getSize();
      bool canBegin = Update.begin(contentLength);

      if (canBegin) {
        WiFiClient *client = http.getStreamPtr();
        size_t written = Update.writeStream(*client);

        if (written == contentLength) {
          Serial.println("Written: " + String(written) + " successfully");
        } else {
          Serial.println("Written only: " + String(written) + "/" + String(contentLength) + ". Retry?");
        }

        if (Update.end()) {
          Serial.println("OTA done!");
          if (Update.isFinished()) {
            Serial.println("Update successfully completed. Rebooting.");
            ESP.restart();
          } else {
            Serial.println("Update not finished. Something went wrong.");
          }
        } else {
          Serial.println("Error Occurred. Error #: " + String(Update.getError()));
        }
      } else {
        Serial.println("Not enough space to begin OTA");
      }
    } else {
      Serial.println("Failed to retrieve the bin file. HTTP response code: " + String(httpCode));
    }
    http.end();
  } else {
    Serial.println("WiFi not connected, cannot perform OTA");
  }
}

void dialogBox(String topic, String body, int d) {
  //String Body2 = "";// if needed

  send_text28(topic);
  send_text28(topic);
  delay(50);
  send_text29(body);
  send_text29(body);
  //send_text30(Body2);// if needed
  pageShift22();
  delay(d);
  pageShift1();  // Shift to page 2(Main page)
}


void appendToPublish(const char *jsonBuffer) {
  if (xSemaphoreTake(fileAccessSemaphore, portMAX_DELAY)) {   // Wait indefinitely to get the lock
    File logFile = SPIFFS.open("/mqttLog.txt", FILE_APPEND);  // Use FILE_APPEND to append to the file
    if (logFile) {
      logFile.println(jsonBuffer);
      logFile.close();
      totalLines++;  // Update total number of lines
      String totLns = String(totalLines);
      writeToSPIFFS("/112.txt", totLns.c_str()); /*SD Save*/
      Serial.println("Data written to mqttLog.txt in SPIFFS");

    } else {
      Serial.println("Error opening mqttLog.txt in SPIFFS");
    }
    xSemaphoreGive(fileAccessSemaphore);  // Release the lock
    delay(300);
  }
}


void publishNextLine() {

  if (lineNo >= totalLines) {
    // If all lines are published, delete the file
    return;  // Exit the function after file deletion
  }

  if (xSemaphoreTake(fileAccessSemaphore, portMAX_DELAY)) {  // Wait indefinitely to get the lock

    File logFile = SPIFFS.open("/mqttLog.txt", FILE_READ);  // Open the file in read mode
    if (!logFile) {
      Serial.println("Error opening mqttLog.txt for reading in SPIFFS");
      xSemaphoreGive(fileAccessSemaphore);  // Release the lock in case of failure
      return;
    }

    logFile.seek(0);  // Reset file pointer to the beginning
    for (int i = 0; i < lineNo; i++) {
      logFile.readStringUntil('\n');  // Skip lines until the desired one
    }

    String line = logFile.readStringUntil('\n');  // Read the next line
    logFile.close();
    xSemaphoreGive(fileAccessSemaphore);  // Release the lock

    if (line.length() > 0) {
      MQTT_Reconnect();                                   // Ensure MQTT is connected before publishing
      line.trim();                                        // Remove any unwanted whitespace
      if (client.publish(send_topic, line.c_str(), 2)) {  // Publish with QoS 2
        lineNo++;                                         // Increment line number after successful publish
        String lNum = String(lineNo);
        writeToSPIFFS("/111.txt", lNum.c_str()); /*SD Save*/
        Serial.print("Published line-");
        Serial.println(lineNo);
        //doneDel=0;
      }
    }
  }
}


int debugV = 0;
void mqttThread(void *parameter) {
  while (1) {
    vTaskDelay(50 / portTICK_PERIOD_MS);
    publishNextLine();
    client.loop();
    debugV++;
    if (debugV == 123) {
      Serial.println(totalLines);
      Serial.println(lineNo);
      debugV = 0;
      //totalLines=countLines();
    }
  }
}

void writeToSPIFFS(const char *path, const char *message) {
  Serial.printf("Writing file to SPIFFS: %s\n", path);

  File file = SPIFFS.open(path, FILE_WRITE);  // Open file in write mode
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }

  if (file.print(message)) {
    Serial.println("File written successfully");
  } else {
    Serial.println("File write failed");
  }

  file.close();  // Close the file
}

void readSPIFFS(const char *path) {
  Serial.printf("Reading file from SPIFFS: %s\n", path);

  File file = SPIFFS.open(path);  // Open the file for reading
  if (!file) {
    Serial.println("Failed to open file for reading");
    ReadDone = 0;
    return;
  }

  while (file.available()) {
    rdline = file.readStringUntil('\n');  // Read line by line
    Serial.println(rdline);               // Print the line for debugging purposes
  }

  file.close();  // Close the file when done
}

int countLines() {
  File logFile = SPIFFS.open("/mqttLog.txt", "r");
  if (!logFile) {
    Serial.println("Error opening mqttLog.txt");
    return -1;
  }

  int lc = 0;
  while (logFile.available()) {
    logFile.readStringUntil('\n');
    lc++;
  }
  logFile.close();
  return lc;
}
