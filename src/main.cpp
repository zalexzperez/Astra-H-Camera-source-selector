#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

/* Definitions */

#define DEBUG
#ifdef DEBUG
  #define DEBUGPRINT(X) Serial.print(X)
  #define DEBUGPRINTLN(X) Serial.println(X)
  #define DEBUG_PRINTDEC(x) Serial.print (x, DEC)
  #define DEBUG_PRINTHEX(x) Serial.print (x, HEX)
  #define DEBUG_SERIALBEGIN(X) Serial.begin(X)
#else 
  #define DEBUGPRINT(X) // As nothing
  #define DEBUGPRINTLN(X) // As nothing
  #define DEBUG_PRINTDEC(x) // As nothing
  #define DEBUG_PRINTHEX(x) // As nothing
  #define DEBUG_SERIALBEGIN(X) // As nothing
#endif

/* Variable definitions */

// Receiver MAC Address. //TODO In the real project, replace with the actual receiver MAC address
uint8_t broadcastAddress[] = {0x78, 0xE3, 0x6D, 0x18, 0xCC, 0x78};

// Create a variable of type esp_now_peer_info_t to store information about the receiving peer.
esp_now_peer_info_t peerInfo;

// Structure to send data. Must match the receiver's structure
typedef struct struct_message1 {
  bool speed_required = 0; // Flag to be enabled when we need to receive speed data
  bool fcam_status = 0; // Flag that will be HIGH when the front camera is currently active
} struct_message1;

// Create a struct_message called mySendingData
struct_message1 mySendingData;

// Structure to receive data. Must match the receiver's structure
typedef struct struct_message2 {
  uint8_t car_speed = 0; // This variable will allow us to know whether the car is stopped or moving too fast, used in the camera logic.
  bool fcam_kill_switch = 0; // This flag will allow killing the front camera view if steering wheel button is pressed. Enabled from the external uC
} struct_message2;

// Create a struct message called myReceivingData
struct_message2 myReceivingData;

// Standard variables
volatile int SM_state = 1; // This'll record the current state in the state machine. Our initial state is state 1. 
volatile bool reverseEngaged = 0; // Flag to toggle in the ISR, depending on the reverse gear status. 
unsigned long time_car_stopped = 0; // Varible to record when the car stops after disengaging reverse gear
unsigned long time_fcam_enabled = 0; // Records when the front camera was enabled
unsigned long wait_after_stopped = 2000; // Controls how much time to keep back camera signal after reverse disengaged and car stop.
unsigned long wait_after_fcam_enabled = 6000; // Time after which the front camera and trigger signal need to be switched off

// Pin definitions
#define videoSW_CTRL 4 
#define optocoupler_CTRL 15
#define reverseSignalpin 16

// Function declarations

// Callback function that will be executed when data is received.
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len); 
// Callback function that will be executed when data is sent.
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) ; 
// Interrupt Service Routine that will trigger everytime the reverse gear is engaged or disengaged.
void IRAM_ATTR reverseSignal(); 
// Function to request or unrequest speed and inform front cam status to the external uC via ESP-NOW commmunication.
void send_information(); 


void setup() {

  // Initialize Serial Monitor
  DEBUG_SERIALBEGIN(250000);
  
  // Set device as a Wi-Fi Station (necessary for ESP-NOW)
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    DEBUGPRINTLN("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to get recv packer info
  esp_now_register_recv_cb(OnDataRecv);

  // Once ESPNow is successfully Init, we will register for Send CB to get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);

  // Register receving peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  // Add receiving peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    DEBUGPRINTLN("Failed to add peer");
    return;
  }

  // Pin configuration
  pinMode(videoSW_CTRL, OUTPUT); // This output will allow us to select which video input of the two to select. [LOW == Input 1 == Front camera signal  | HIGH == Input 2 == Back camera signal]
  pinMode(optocoupler_CTRL, OUTPUT); // This will generate a 12V camera trigger signal for the head unit. [LOW == Trigger signal OFF. HIGH == Trigger signal ON]
  pinMode(reverseSignalpin, INPUT_PULLDOWN); // [LOW == Reverse disengaged. HIGH == Reverse engaged]
  #ifdef DEBUG
  pinMode(BUILTIN_LED, OUTPUT);
  digitalWrite(BUILTIN_LED, LOW);
  #endif

  // Call reverseEngaged() when interrupt pin goes through a change transition
	attachInterrupt(digitalPinToInterrupt(reverseSignalpin), reverseSignal, CHANGE);
  
  // Trigger signal off initially.
  digitalWrite(optocoupler_CTRL, LOW);

  // Select front camera video source.
  digitalWrite(videoSW_CTRL, LOW);

  // If reverse signal changes before system loads, we'd miss the transition. The next block prevents that.
  if (digitalRead(reverseSignalpin)){
    reverseEngaged = 1;
    SM_state=1;
    DEBUGPRINTLN("Reverse gear preselected");
  } 
}


void loop() {

  switch(SM_state){

    case 0:{ // RESET STATE 
      
      DEBUGPRINTLN("State = 0");
      // Disable trigger signal
      digitalWrite(optocoupler_CTRL, LOW);

      // Reselect front camera video source
      digitalWrite(videoSW_CTRL, LOW);

      // Update front camera inactive status
      mySendingData.fcam_status = 0;

      // Speed no longer required
      mySendingData.speed_required = 0;

      // Send new information to external uC
      send_information();

      // Turn off built-in LED to visually inform of trigger signal status
      #ifdef DEBUG
      digitalWrite(BUILTIN_LED, LOW);
      #endif

      // Move to the next state
      SM_state++;
      DEBUGPRINT("State = ");
      DEBUGPRINTLN(SM_state);
    } break;
  
    case 1:{  // WAIT FOR REVERSE ENGAGEMENT
      if (reverseEngaged){
        DEBUGPRINTLN("Reverse engaged");
        SM_state++;
        DEBUGPRINT("State = ");
        DEBUGPRINTLN(SM_state);
      }
    } break;

    case 2:{ // ENABLE BACK CAMERA

      // Select the back camera video source
      digitalWrite(videoSW_CTRL, HIGH);

      // Enable trigger signal
      digitalWrite(optocoupler_CTRL, HIGH);

      // Request speed to external uC
      mySendingData.speed_required = 1;
      send_information();
      
      #ifdef DEBUG
        // Turn on built-in LED to visually inform of trigger signal status
        digitalWrite(BUILTIN_LED, HIGH);
      #endif

      // Move to the next state
      SM_state++;
      DEBUGPRINT("State = ");
      DEBUGPRINTLN(SM_state);
      DEBUGPRINTLN("Waiting for disengagement");
    } break;

    case 3:{ // WAIT FOR DISENGAGEMENT

      if(!reverseEngaged){
        DEBUGPRINTLN("Reverse disengaged");
        SM_state++;
        DEBUGPRINT("State = ");
        DEBUGPRINTLN(SM_state);
        DEBUGPRINTLN("Waiting for car to stop");
      }
    } break;

    case 4:{ // WAIT FOR CAR STOP
 
      if(myReceivingData.car_speed == 0){
        DEBUGPRINTLN("Car supposedly stopped after reverse disengaged");

        // Record time when car is stopped
        time_car_stopped = millis();

        // Move to the next state;
        SM_state++;
        DEBUGPRINT("State = ");
        DEBUGPRINTLN(SM_state);
        DEBUGPRINTLN("Waiting some time after stopped");
      }
    } break;  

    case 5:{ // WAIT SOME TIME

      if (millis()-time_car_stopped >= wait_after_stopped){
        SM_state++;
        DEBUGPRINT("State = ");
        DEBUGPRINTLN(SM_state);
        DEBUGPRINTLN("Front camera will be enabled");
      }
    } break;

    case 6:{ // ENABLE FRONT CAMERA

      // Select the front camera video source
      digitalWrite(videoSW_CTRL, LOW);

      // Inform external uC of front camera active status
      mySendingData.fcam_status = 1;

      // Send new information to external uC
      send_information();

      // Record time when front camera active status
      time_fcam_enabled = millis();

      // Move to the next state
      SM_state++;
      DEBUGPRINT("State = ");
      DEBUGPRINTLN(SM_state);
      DEBUGPRINTLN("Wait for time or speed conditions to kill trigger signal");
    } break;

    case 7:{ // WAIT FOR TIME, SPEED OR OTHER CONDITIONS

      if (millis()-time_fcam_enabled >= wait_after_fcam_enabled || myReceivingData.car_speed >= 6 || myReceivingData.fcam_kill_switch){
        
        SM_state=0;
        
        #ifdef DEBUG
          if(millis()-time_fcam_enabled >= wait_after_fcam_enabled) DEBUGPRINTLN("Time condition killed trigger signal");
          else if(myReceivingData.car_speed >= 6) DEBUGPRINTLN("Speed condition killed trigger signal");
          else DEBUGPRINTLN("Pushbutton press killed trigger signal");
        #endif
      }
    } break;  
  }  
}


void IRAM_ATTR reverseSignal(){

  // Reverse gear was engaged
  if (digitalRead(reverseSignalpin)){
    reverseEngaged = 1;
    SM_state=1;
    DEBUGPRINT("State = ");
    DEBUGPRINTLN(SM_state);
  } 
  //Reverse gear was disengaged:
  else{
    reverseEngaged = 0;
  }  
}


void send_information(){

  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &mySendingData, sizeof(mySendingData));
  if (result == ESP_OK) {//DEBUGPRINTLN("Sent with success");
  }
  else  DEBUGPRINTLN("Error sending the data");
}



void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {

  memcpy(&myReceivingData, incomingData, sizeof(myReceivingData));
  //DEBUGPRINT("Bytes received: ");
  //DEBUGPRINTLN(len);

  // Only make sure we're printing speed messages if speed is required.
  if (mySendingData.speed_required){
    DEBUGPRINTLN("");
    DEBUGPRINT("Vehicle speed: ");
    DEBUGPRINT(myReceivingData.car_speed);
    DEBUGPRINTLN(" km/h");
  }

}


void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {

  //DEBUGPRINT("\r\nLast Packet Send Status:\t");
  //DEBUGPRINTLN(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}