/*
Wireless Stethoscope using MKR1000
*/

#include <SPI.h>
#include <WiFi101.h>
#include <SdFat.h>
#include <MnTAud.h>

#define DEBUGAPP 0 // Uncomment this line for debug prints.

#define sdPin 07

char ssid[] = "MnTStethoWW";
int status = WL_IDLE_STATUS;
WiFiServer server(80);

MnTAud DDB;
SdFat SDF;
char newWavFile[] = "tooikane.wav";

void setup() {
  #ifdef DEBUGAPP
    Serial.begin(9600);
    Serial.println("Initialising Access Point.");
  #endif
  pinMode(LED_BUILTIN, OUTPUT);

  if (WiFi.status() == WL_NO_SHIELD) {
    #ifdef DEBUGAPP
      Serial.println("Insert WiFi shield.");
    #endif
    digitalWrite(LED_BUILTIN, LOW);
    while (true);
  }

  // Default local IP is 192.168.1.1
  #ifdef DEBUGAPP
    Serial.print("Creating WiFi Access Point: ");
    Serial.println(ssid);
  #endif

  status = WiFi.beginAP(ssid);
  if (status != WL_AP_LISTENING) {
    #ifdef DEBUGAPP
      Serial.println("Failed to create.");
    #endif
    digitalWrite(LED_BUILTIN, LOW);
    while (true);
  }
  
  delay(10000);
  server.begin();
  #ifdef DEBUGAPP
    printWiFiStatus();
  #endif

  // Initialise the SD card.
  if (!SDF.begin(sdPin, SD_SCK_MHZ(4))) {
    #ifdef DEBUGAPP
      Serial.println("SD card failed to initialise! Please run diagnostics to debug!\nStalling!");
    #endif
    digitalWrite(LED_BUILTIN, LOW);
    while (true);
  }
  
  #ifdef DEBUGAPP
    Serial.println("SD card initialised!");
  #endif
  
  // Initialise ADC and Ticker.
  DDB.SystemAudStart();

  #ifdef DEBUGAPP
    Serial.println("Microphone system ready!");
  #endif
}

void loop() {
  if (status != WiFi.status()) {
    status = WiFi.status();

    if (status == WL_AP_CONNECTED) {
      byte remoteMac[6];
      WiFi.APClientMacAddress(remoteMac);
      #ifdef DEBUGAPP
        Serial.print("Device connected. MAC address: ");
        printMacAddress(remoteMac);
      #endif
    } else {
      #ifdef DEBUGAPP
        Serial.println("Device disconnected.");
      #endif
    }
  }

  WiFiClient client = server.available();

  if (client) {
    #ifdef DEBUGAPP
      Serial.println("New client GET.");
    #endif
    String currentLine = "";
    while (client.connected()){
      if (client.available()){
        char c = client.read();
        #ifdef DEBUGAPP
          Serial.write(c);
        #endif
        if (c == '\n') {
          if (currentLine.length() == 0) {
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println();
            client.print("Testing <br>");
            client.println();
            break;
          } else {
            if (currentLine.indexOf("POST /") != -1) {
              int tempIndex = currentLine.indexOf("POST /");
              String tempFileName = currentLine.substring(tempIndex + 6) + ".wav";
              int inputIndex = tempFileName.indexOf(" HTTP/1.1");
              tempFileName.remove(inputIndex,9);
              tempFileName.toCharArray(newWavFile, (tempFileName.length()) + 1);
              #ifdef DEBUGAPP
                Serial.println(newWavFile);
              #endif
              }
            currentLine = "";
          }
        } else if (c != '\r') {
          currentLine += c;
        }
        if (currentLine.endsWith("GET /A")){
          recordHeart();
        }
        if (currentLine.endsWith("GET /B")){
          stopHeart();
        }
        if (currentLine.endsWith("GET /C")){
          recordLungs();
        }
        if (currentLine.endsWith("GET /D")){
          stopLungs();
        }
        if (currentLine.endsWith("GET /E")){ 
          fetchFile();
        }
      }
    }
    client.stop();
    #ifdef DEBUGAPP
      Serial.println("Client cut.");
    #endif
  }
}

void printWiFiStatus() {
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  long rssi = WiFi.RSSI();
  Serial.print("Signal Strength: ");
  Serial.print(rssi);
  Serial.println(" dBm");

  byte encryptionTypeMM = WiFi.encryptionType();
  Serial.print("Encryption Type: ");
  Serial.println(encryptionTypeMM, HEX);
}

void printMacAddress(byte mac[]) {
  for (int i = 5; i >= 0; i--) {
    if(mac[i] < 16) {
      Serial.print("None.");
    }
    Serial.print(mac[i], HEX);
    if (i > 0) {
      Serial.print(":");
    }
  }
  Serial.println();
}

void recordHeart() {
  // digitalWrite(6, HIGH);
  DDB.StartRecording(newWavFile, 8000);
  #ifdef DEBUGAPP
    Serial.println("RecordHeart.");
  #endif
}

void stopHeart() {
  DDB.StopRecording(newWavFile, 8000);
  // digitalWrite(6, LOW);
  #ifdef DEBUGAPP
    Serial.println("StopHeart.");
  #endif
}

void recordLungs() {
  // digitalWrite(5, HIGH);
  DDB.StartRecording(newWavFile, 15000);
  #ifdef DEBUGAPP
    Serial.println("RecordLungs.");
  #endif
}

void stopLungs() {
  DDB.StopRecording(newWavFile, 15000);
  // digitalWrite(5, LOW);
  #ifdef DEBUGAPP
    Serial.println("StopLungs.");
  #endif
}

void fetchFile() {
  
  int dataCount = 0;
  char dataBuff[65];

  FatFile fileFetch;
  fileFetch.open(newWavFile, O_READ);

  if (!fileFetch.isOpen()) {
    #ifdef DEBUGAPP
      Serial.println("File does not exist!");
    #endif
    digitalWrite(LED_BUILTIN, LOW);
    return;
  }
  
  uint32_t contentLength = fileFetch.fileSize();
  String stringContLen = String(contentLength);
  WiFiClient client = server.available();
   
  client.println("HTTP/1.1 200 OK");
  client.println("Server: Arduino MKR1000");
  client.println("Content-length: " + stringContLen);
  client.println("Content-type: audio/wav");
  client.println("Connection: Keep-Alive");
  client.println();

  while (fileFetch.available() != 0) {
    dataBuff[dataCount] = fileFetch.read();
    dataCount++;
    dataBuff[dataCount] = 0;

    if (dataCount > 63) {
      client.write((byte *)dataBuff, 64);
      dataCount = 0;
    }
  }

  if (dataCount > 0) {
    client.write((byte *)dataBuff, dataCount);
  }

  client.println();
  fileFetch.close();

  #ifdef DEBUGAPP
    Serial.println("Fetch.");
  #endif
}
