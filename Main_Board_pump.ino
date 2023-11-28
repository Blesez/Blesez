#include <esp_now.h>
#include <EEPROM.h>
#include <Arduino.h>
//Firebase libraries
//#include <Firebase_ESP_Client.h>
//#include "addons/TokenHelper.h"
//#include "addons/RTDBHelper.h"
//wifi includes
#include <WiFiManager.h>    
#include <WiFi.h>
#include <WebSocketsClient.h>
#include <LiquidCrystal_I2C.h>
#include <ArduinoJson.h>

//-----------------------------------------------------------------------------------------------------------------------------------------------
//Declare variables
//-----------------------------------------------------------------------------------------------------------------------------------------------

//define variable
#define API_PIN "AIzaSyClG6qmdTm3YPN01P3xADp6fQ1ltHfkzYs"
#define DATABASE_URL "https://console.firebase.google.com/project/watercontroller-99262/database/watercontroller-99262-default-rtdb/data/~2F"
#define RelayPin 26

int32_t CHANNEL;
int solenoidA; // data sent to solenoid esp for control
int solenoidB;  //data sent to solenoid esp for control
int solenoidASize; // tanksize as computed by the user
int solenoidBSize;   //tanksize as computed by the user
int solenoidA_min;  //lowest level water is allowed to reach in the tank
int solenoidB_min;  //lowest level water is allowed to reach in the tank
int pump_control; //hold pump status instruction from webserver
int storedValue_0;
int storedValue_1;
int Period = 15000;
int TimeNow = 0;
int lastButtonState = LOW;  
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50; 
esp_now_peer_info_t peerInfo;

//sensor variables
int  sensorA;
int  sensorB;

//LCD variables
LiquidCrystal_I2C lcd(0x27, 16, 2);

//path and host variable
char path[]= "";
char host[]= "";

//ConfigPortalVariables
#define AP_PIN 12
volatile int RESET_WIFI = 0;
int timeout = 240;

//Firebase variables
/*FirebaseData fbdo; //define fb data object
FirebaseAuth auth;
FirebaseConfig config;
bool signupOK = false;*/


uint8_t broadcastAddress[] = {0x24, 0x6F, 0x28, 0x83, 0xF7, 0x7C};

//structure to recieve data.....must match sender's
typedef struct sensors_struct {
  int sensor_valueA;
  int sensor_valueB;
  } sensors_struct;

//structure to hold data to be sent
 typedef struct TankState_struct {
  int solenoidA_State;
  int solenoidB_State;
  }TankState_struct;


//relay structure
struct Pump{
  uint8_t Relay_pin;
  bool on;

  void update(){
      digitalWrite(Relay_pin, on ? HIGH : LOW);
    }
  };

  Pump relay = {RelayPin, false};

  WebSocketsClient webSocket;
  StaticJsonDocument<200> doc_received;
  StaticJsonDocument<200> doc_sent;


 //-----------------------------------------------------------------------------------------------------------------------------------------------
//D I S P L A Y   S E C T I O N . 
//------------------------------------------------------------------------------------------------------------------------------------------------

 void displayConnectAPSucess(){
                 lcd.clear();
                 lcd.setCursor (0,0);
               lcd.print("softAP connected");
               delay(700);
  }
  
 void displayConnectAPFailed(){
                 lcd.clear();
                 lcd.setCursor (0,0);
               lcd.print("softAP connect failed");
               delay(1000);
               for (int positionCounter = 6; positionCounter > 0; positionCounter--){
              lcd.scrollDisplayRight();
              delay(500);
                  }
  }
 
 void displayConfigAPFailed(){
                 lcd.clear();
                 lcd.setCursor (0,0);
               lcd.print("AP Config Failed");
               delay(1000);
  }
  void displayDileveryStatus(){
                 lcd.clear();
                 lcd.setCursor(0,0);
                lcd.print("Send Status: ");
                lcd.setCursor(0,1);
              lcd.println(ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
              delay (700);
  }

    void displayDataRecievedSucces(){
                   lcd.clear();
                 lcd.setCursor(0,0);
                lcd.print("Data Recieved ");
  }



//-----------------------------------------------------------------------------------------------------------------------------------------------
//WifiManager
//-----------------------------------------------------------------------------------------------------------------------------------------------

//wifimanger config callbacks
void configModeCallback (WiFiManager *myWiFiManager) {
            Serial.println("Entered config mode");
                lcd.clear();
                lcd.setCursor(0,2);
                lcd.print("Config mode");
            Serial.println(WiFi.softAPIP());
            //if you used auto generated SSID, print it
            Serial.println(myWiFiManager->getConfigPortalSSID());
            delay(1000);            
}

void saveConfigCallback (WiFiManager *myWiFiManager){
            Serial.println("Save config");
            Serial.println(WiFi.softAPIP());
}

  // configure AP SSID
void configDeviceAP() {
                  WiFi.mode(WIFI_STA);
                const char *SSID = "receiverESP_SoftAP";
                bool result = WiFi.softAP(SSID, "12345678", CHANNEL, 0);
                if (!result) {
                  Serial.println("AP Config failed.");
                  displayConfigAPFailed();
                } else {
                  Serial.println("AP Config Success. Broadcasting with AP: " + String(SSID));
                   lcd.clear();
                 lcd.setCursor (0,0);
               lcd.print("AP configured successfully to:");
               lcd.setCursor (0,1);
               lcd.print(SSID);
                delay(300);
               for (int positionCounter = 15; positionCounter > 0; positionCounter--){
              lcd.scrollDisplayRight();
              delay(100);
                }
              }
}  

void wiFiManagerInit(){
               
                   WiFiManager wifiManager;
              //reset settings - for testing
              //wifiManager.resetSettings();
              lcd.clear();
              lcd.setCursor(0,0);
              lcd.print("init WiFiManager");
              wifiManager.setConfigPortalTimeout(timeout);
             lcd.setCursor (0,1);
             lcd.print (timeout);
             lcd.print ("sec timeout");           
              if(!wifiManager.autoConnect("blesez01Tech","12345678")){
                  delay (3000);
                Serial.println("Failed to connect");
                lcd.clear();
                lcd.setCursor(0,0);
                lcd.print("init WiFiManager");
                lcd.setCursor (0,1);
                lcd.print("Failed to connect");
               delay (2000);
                ESP.restart();
                         
                delay(4000);
                }
                Serial.println("Connected.....yaaayyyy");
                
                   lcd.clear();
                lcd.setCursor(0,0);
                lcd.print("init WiFiManager");
                lcd.setCursor (0,1);
                lcd.print("Connected yaayyy");
              //GET ROUTER CONFIGURATION
                delay (2000);
  }
//-----------------------------------------------------------------------------------------------------------------------------------------------
//ESPnow 
//-----------------------------------------------------------------------------------------------------------------------------------------------

  // callback when data is recv from Master
 void OnDataRecv(const uint8_t * mac_addr, const uint8_t *data, int data_len) {
              char macStr[18];
              snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
                       mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
                        sensors_struct* sensors = (sensors_struct*) data;
              Serial.println("Last Packet Recv from:  ");
              Serial.print(macStr);
              Serial.println("Last Packet Recv Data: ");
              sensorA= sensors -> sensor_valueA;
                Serial.println("Value of sensor A:   ");
              Serial.print(sensorA);
                Serial.println("");
                lcd.clear();
                lcd.setCursor (0,0);
                lcd.print ("SensorA recieved");
                 sensorB= sensors -> sensor_valueB;
                  Serial.println("Value of sensor B:   ");
                Serial.print(sensorB);
              Serial.println("");
                 lcd.clear();
                lcd.setCursor (0,1);
                lcd.print ("SensorB recieved");
              delay(1000);
              displayDataRecievedSucces();
             sendTowebServer();
          
}
void solenoidControl(){
  memset(&peerInfo, 0, sizeof(peerInfo));
registerPeer();
sendValues();
           
}
void sendValues(){
               TankState_struct tankState;
              //set values to send
             tankState.solenoidA_State = solenoidA;
              tankState.solenoidB_State  = solenoidB;
                //send message via ESP_NOW
                
              Serial.printf("Sending: %d and %d \n", solenoidA, solenoidB);
                esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &tankState, sizeof(TankState_struct));
            
   if(result== ESP_OK){
    Serial.println("sent successfully");
    }else if (result == ESP_ERR_ESPNOW_NOT_INIT) {
    // How did we get so far!!
    Serial.println("ESPNOW not Init.");
  } else if (result == ESP_ERR_ESPNOW_ARG) {
    Serial.println("Invalid Argument");
  } else if (result == ESP_ERR_ESPNOW_INTERNAL) {
    Serial.println("Internal Error");
  } else if (result == ESP_ERR_ESPNOW_NO_MEM) {
    Serial.println("ESP_ERR_ESPNOW_NO_MEM");
  } else if (result == ESP_ERR_ESPNOW_NOT_FOUND) {
    Serial.println("Peer not found.");
  } else {
    Serial.println("Not sure what happened");
  }
}

 // callback when data is sent from Master to receiver
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
              char macStr[18];
              snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
                       mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
              Serial.print("Last Packet Sent to: ");
              Serial.println(macStr);
              Serial.print("Last Packet Send Status: ");
              Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
             lcd.clear();
             lcd.setCursor(0,1);
             lcd.print(status);
            delay(500);             
}

// Init ESP Now
void InitESPNow() {
                if (esp_now_init() == ESP_OK) {
                  Serial.println("ESPNow Init Success");
                lcd.clear();
                lcd.setCursor (1,0);
                lcd.print("ESPNow Success");    
                delay(1000);                                                              
                }
                else {
                  Serial.println("ESPNow Init Failed");
                  lcd.clear();
                lcd.setCursor (1,0);
                lcd.print("ESPNow Failed");    
                delay(1000); 
                  ESP.restart();
                }
}
        
void registerPeer(){
              
              memcpy(peerInfo.peer_addr, broadcastAddress,6);
             peerInfo.channel = CHANNEL; // pick a channel
              peerInfo.encrypt = 0; // no encryption

              
              if (esp_now_add_peer(&peerInfo)!= ESP_OK){
                Serial.println ("Failed to add peer");
                return;
                }
  }

//-----------------------------------------------------------------------------------------------------------------------------------------------
//Check button
//-----------------------------------------------------------------------------------------------------------------------------------------------

void checkButton() {

                    int RESET_WIFI = digitalRead(AP_PIN);
                    lastDebounceTime = millis();
                  
                  if ((millis() - lastDebounceTime) > debounceDelay) {
                    // RESET WIFI-CREDENTIALS VIA BUTTON
                   if (RESET_WIFI == HIGH){
                     wiFiManagerInit();
                      }
                  }
 }


 //-----------------------------------------------------------------------------------------------------------------------------------------------
//webSocket
//------------------------------------------------------------------------------------------------------------------------------------------------

 void webSocketEvent(WStype_t type, uint8_t* payload, size_t length){

                if (type == WStype_TEXT){
                   DeserializationError error = deserializeJson(doc_received,payload);
                    // Test if parsing succeeds.
                    if (error) {
                      Serial.print(F("deserializeJson() failed: "));
                      Serial.println(error.f_str());
                      
                      return;
                    }
                    pump_control = doc_received [""];
                    solenoidASize = doc_received [""];
                    solenoidBSize = doc_received [""];
                    solenoidA  = doc_received [""];
                    solenoidB  = doc_received [""];

                 saveTankSizes();
                 changePumpStatus();
                 sendValues();
                 
                }
  }

  void webSockets (){
                webSocket.begin ("wss://72daer0qkc.execute-api.us-east-1.amazonaws.com/beta?token:Hardware",3000,"/");
                webSocket.onEvent(webSocketEvent);
                webSocket.setReconnectInterval(5000);
  }
/*void onDataRecvFb(StreamData data){
  if (data.dataType() == "json"){
String payload = data.stringData();  
DeserializationError error = deserializeJson(doc_received, payload); 
if (error){
Serial.print("Json parsing error: ");
Serial.println(error.c_str());
return;
} 
      pump_control = doc_received [""];//what pump should be from user input
      solenoidASize = doc_received [""];//tank 1 size
      solenoidBSize = doc_received [""];//tank 2 size      
      solenoidA  = doc_received [""];// open/close valve
      solenoidB  = doc_received [""];// open/close tank 2 valve

      SaveTankSizes();
      changePumpStatus ();
      sendValues();
 }
}
*/
 void sendTowebServer(){
      char output[150];
        doc_sent["Sender"] = "Group 1";
          JsonArray data = doc_sent.createNestedArray("Data");         
          JsonObject data_0 = data.createNestedObject();
          
              data_0["Pump Status"] = relay.on ? "On" : "Off";
              data_0["Tank 1"] = sensorA;
               data_0["Tank 2"] = sensorB;
                   
        
         serializeJson (doc_sent, output);
webSocket.sendTXT(output);         
/*if (Firebase.ready()&& signupOK == true){                  
 if (Firebase.RTDB.setString(fbdo,"json-output", output)){
Serial.println("Data Sent to firebase successfully");

 } else{
Serial.println("Data failed to send to firebase");
 }
delay (5000);
} else{
  firebaseconf();
}*/
}

//-----------------------------------------------------------------------------------------------------------------------------------------------
//Data from webserver
//------------------------------------------------------------------------------------------------------------------------------------------------

void saveTankSizes(){
                storedValue_0 = EEPROM.read(0);
                if (solenoidASize != storedValue_0){
                EEPROM.write(0, solenoidASize);
                EEPROM.commit();
                storedValue_0 = EEPROM.read(0);
                Serial.println(storedValue_0);
              } 
              else {
                Serial.println("solenoidA size already saved");
              }
              storedValue_1 = EEPROM.read(1);
              if (solenoidBSize != storedValue_1){
                EEPROM.write(1, solenoidBSize);
                EEPROM.commit();
                storedValue_1 = EEPROM.read(1);
                Serial.println(storedValue_1);
              }
              else{
                Serial.println("solenoidB size already saved");
              }
}


void tankMinimum() {
    
  storedValue_0 = EEPROM.read(0);
  storedValue_1= EEPROM.read(1);
  solenoidA_min = 90/100 * (storedValue_0);
  solenoidB_min = 90/100 * (storedValue_1);
  }

 //-----------------------------------------------------------------------------------------------------------------------------------------------
//Firebase
//------------------------------------------------------------------------------------------------------------------------------------------------

/*void firebaseconf(){
config.api_key = API_KEY;
config.database_url = DATABASE_URL; 

if (Firebase.signUp(&config, &auth, "b@gmail.com","1234567890")){
  Serial.println("ok");
  signupOK = true; 
} else{
serial.println("%s\n", config.signer.signupError.message.c_str());
}
config.token_status_callback = tokenStatusCallback;
Firebase.begin(&config, &auth);
Firebase.reconnectWiFi(true);
}*/

 //-----------------------------------------------------------------------------------------------------------------------------------------------
//Pheripherals
//------------------------------------------------------------------------------------------------------------------------------------------------
 
  void changePumpStatus(){
    if ( pump_control== 0){
      relay.on = false;
       Serial.println("Turned pump OFF");
       lcd.clear();
       lcd.setCursor(0,1);
       lcd.print("Turned pump OFF");
    }else if (pump_control == 1){
      relay.on = true;
       Serial.println("Turned pump ON");
        lcd.clear();
       lcd.setCursor(0,1);
       lcd.print("Turned pump ON");
    }else{
       Serial.println("error!!!!!!");
       lcd.clear();
       lcd.setCursor(0,1);
       lcd.print("Pump ERROR!!!!!");
      }
  relay.update();
 }


 
//-----------------------------------------------------------------------------------------------------------------------------------------------
//Setup
//------------------------------------------------------------------------------------------------------------------------------------------------


 void setup(){
               lcd.init();
               lcd.clear();
               lcd.backlight();
                   lcd.setCursor (5,0);
               lcd.print("Welcome");
               delay (2000);
               WiFi.mode(WIFI_AP_STA);
               //Re-Configure wifi button
              pinMode(AP_PIN,INPUT);
              attachInterrupt(digitalPinToInterrupt(AP_PIN),checkButton, CHANGE);
              
              //serial monitor init
                Serial.begin(115200);
                EEPROM.begin(512);
               //Relay init
              pinMode(relay.Relay_pin, OUTPUT);
               wiFiManagerInit();
               CHANNEL = WiFi.channel();
               Serial.println(CHANNEL);
               configDeviceAP();
              webSockets ();
               //firebaseconf();
              //Firebase.setStreamCallback(onDataRecvFb);
              //String path = "/path/to/data";
              //Firebase.stream(path);                                                       
              Serial.print("AP MAC: "); Serial.println(WiFi.softAPmacAddress());
              
              // Init ESPNow
              InitESPNow();

               esp_now_register_recv_cb(OnDataRecv);
               esp_now_register_send_cb(OnDataSent);

               //firebase
               
      }

 //-----------------------------------------------------------------------------------------------------------------------------------------------
//LOOP
//------------------------------------------------------------------------------------------------------------------------------------------------
 

void loop(){
      webSocket.loop();
       if(millis()-  TimeNow > Period){
          TimeNow = millis();
      AutoPrecaution();
       }
}

 //-----------------------------------------------------------------------------------------------------------------------------------------------
//Auto precaution
//------------------------------------------------------------------------------------------------------------------------------------------------
 

void AutoPrecaution() {
tankMinimum();

if (sensorA <= 5 && sensorB <= 5){
   relay.on = false;
   Serial.println ("Turn pump off by AutoPre");
   lcd.clear();
   lcd.setCursor (0,0);
   lcd.print("Pump is OFF");
   delay (500);
  }
  
      else if (sensorA <= 5 && sensorB > 5){
            solenoidA =1;
            solenoidB =0;
            solenoidControl();
            Serial.println("Send seginal to solenoid A valve");
            lcd.clear();
               lcd.setCursor (0,0);
           lcd.print("Pump is ON");
           lcd.setCursor (0,1);
           lcd.print("TANK 1 CLOSED");
           delay (500);
            }
       else if (sensorA > 5 && sensorB <= 5){
              solenoidA =0;
              solenoidB =1;
            solenoidControl();
            Serial.println("Send seginal to solenoid B valve");
                lcd.clear();
              lcd.setCursor (0,0);
               lcd.print("Pump is ON");
               lcd.setCursor (0,1);
               lcd.print("TANK 2 CLOSED");
               delay (500);
                  }
      else if ((sensorA > 5 || sensorA < solenoidA_min)  && (sensorB > 5 || sensorB < solenoidB_min)){
        
        return;
        }
        else if ((sensorA >= solenoidA_min && sensorB < solenoidA_min)){
          relay.on = true;
           Serial.println ("Turn pump on by AutoPre");
           solenoidA =1;
           solenoidB =0;
           solenoidControl();
            lcd.clear();
                 lcd.setCursor (0,0);
               lcd.print("Pump is ON"); 
               lcd.setCursor (0,1);
               lcd.print("TANK 1 OPENED");
               delay (500);
          }
           else if ((sensorA < solenoidA_min && sensorB >= solenoidA_min)){
          relay.on = true;
           Serial.println ("Turn pump on by AutoPre");
           solenoidA =0;
           solenoidB =1;
           solenoidControl();
               lcd.clear();
                 lcd.setCursor (0,0);
               lcd.print("Pump is  ON"); 
               lcd.setCursor (0,1);
               lcd.print("TANK 2 OPENED");
               delay (500);
           }
            else if ((sensorA >= solenoidA_min && sensorB >= solenoidA_min)){
          relay.on = true;
           Serial.println ("Turn pump on by AutoPre");
           solenoidA =1;
           solenoidB =1;
           solenoidControl();
            lcd.clear();
               lcd.setCursor (0,0);
               lcd.print("Pump is  ON"); 
               lcd.setCursor (0,1);
               lcd.print("TANK 1&2 OPENED");
               delay (500);
            }
delay(5000);
}
