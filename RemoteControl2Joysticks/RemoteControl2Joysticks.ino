/*
Board type: ESP32-WROOM-DA Module
*/


#include <esp_now.h>
#include <WiFi.h>

//PIN for left Joystick
#define VRX_PIN_L  36 // ESP32 pin GPIO36 (ADC0) connected to VRX pin
#define VRY_PIN_L  39 // ESP32 pin GPIO39 (ADC0) connected to VRY pin
#define BTN_PIN_L  17 // ESP32 pin GPIO17 connected to BTN pin

//PIN for right Joystick
#define VRX_PIN_R  35 // ESP32 pin GPIO36 (ADC0) connected to VRX pin
#define VRY_PIN_R  32 // ESP32 pin GPIO39 (ADC0) connected to VRY pin
#define BTN_PIN_R  33 // ESP32 pin GPIO17 connected to BTN pin

//Set Receiver MAC Address
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};


int valueXLeft = 0; // to store the X-axis value
int valueYLeft = 0; // to store the Y-axis value
int bValueLeft = 0; // To store value of the button

int valueXRight = 0; // to store the X-axis value
int valueYRight = 0; // to store the Y-axis value
int bValueRight = 0; // To store value of the button

int motor_left; // left motor
int motor_right; // right motor

// Struct used to send joystick state to the robot
typedef struct MotorData {
  int motor_left;
  int motor_right;
  int button;
} MotorData;

MotorData motors;

int buttonStateLeft = 0;
int buttonStateRight = 0;

esp_now_peer_info_t peerInfo;

// What to do when data has been sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}


void setup() {

  // Set PIN mode
  pinMode(VRX_PIN_L, INPUT_PULLUP);
  pinMode(VRY_PIN_L, INPUT_PULLUP);
  pinMode(BTN_PIN_L, INPUT_PULLUP);

  pinMode(VRX_PIN_R, INPUT_PULLUP);
  pinMode(VRY_PIN_R, INPUT_PULLUP);
  pinMode(BTN_PIN_R, INPUT_PULLUP);

  // Set WiFi in station mode
  WiFi.mode(WIFI_STA);

  //Initialize Serial interface
  Serial.begin(9600) ;

  // Verify initilization of WiFi communication
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register function to execute when data is sent
  esp_now_register_send_cb(OnDataSent);
  
  // Define peer and channel configuration
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  //peerInfo.peer_addr = peerAddress;
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  //esp_now_register_recv_cb(OnDataRecv);
}

void loop() {

  // read X and Y analog values
  valueXLeft = analogRead(VRX_PIN_L);
  valueYLeft = analogRead(VRY_PIN_L);
  // read button state
  buttonStateLeft = digitalRead(BTN_PIN_L);


  // read X and Y analog values
  valueXRight = analogRead(VRX_PIN_R);
  valueYRight = analogRead(VRY_PIN_R);
  // read button state
  buttonStateRight = digitalRead(BTN_PIN_R);

  // reverse button state so that 0 is not pressed and 1 is pressed
  if (buttonStateLeft) {
    buttonStateLeft = false;
  }
  else {
    buttonStateLeft = true;
  }

  if (buttonStateRight) {
    buttonStateRight = false;
  }
  else {
    buttonStateRight = true;
  }

  // print data to Serial Monitor on Arduino IDE
  Serial.println("Left Joystick");
  Serial.print("x = ");
  Serial.print(valueXLeft);
  Serial.print(", y = ");
  Serial.print(valueYLeft);
  Serial.print(" : button = ");
  Serial.println(buttonStateLeft);

  Serial.println("Right Joystick");
  Serial.print("x = ");
  Serial.print(valueXRight);
  Serial.print(", y = ");
  Serial.print(valueYRight);
  Serial.print(" : button = ");
  Serial.println(buttonStateRight);

  // Get X and Y analog value
  getReadings();

  // Set the struct value
  motors.motor_left = motor_left;
  motors.motor_right = motor_right;
  motors.button = buttonStateRight;

  // Send the struct to the peer
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &motors, sizeof(motors));
  
  // Check if the communication was successfull
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
}


void getReadings() {
  motor_left = analogRead(VRY_PIN_L);
  motor_right = analogRead(VRY_PIN_R);
}
