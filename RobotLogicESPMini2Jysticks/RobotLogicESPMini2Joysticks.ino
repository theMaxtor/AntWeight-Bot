/*
Board type: ESP32C3 Dev Module
USB CDC On Boot: Enabled
Flash Mode: QIO
JTAG Adapter: Integrated USB JTAG
Partition Schema: Minimal SPIFFS

To flash, if necessary, hold BOOT button, press RST button and relase BOOT button
*/

#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h>
#include <esp_wifi.h>

#define LEFT_MOTOR_PIN 5
#define RIGHT_MOTOR_PIN 4
#define WEAPON_MOTOR_PIN 2

// MAC address to set for the receiver
uint8_t newMACAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Structure to receive data
// Must match the sender structure
typedef struct MotorData {
  int motor_left;
  int motor_right;
  int button;
} MotorData;

// Create a struct_message
MotorData myMotors;

// Because of the servo library, the engine speed goes from 0 to 180:
// 0 -> full speed backwards
// 90 -> motor is stopped
// 180 -> full sped straight
Servo leftMotor;  // create servo object to control left motor
Servo rightMotor;  // create servo object to control right motor
Servo weapon;

bool weaponOn = false;


// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myMotors, incomingData, sizeof(myMotors));

  // Print data received by remote control
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("Left Motor: ");
  Serial.println(myMotors.motor_left);
  Serial.print("Right Motor: ");
  Serial.println(myMotors.motor_right);
  Serial.print("Button: ");
  Serial.println(myMotors.button);
  Serial.println();


  int leftSpeed = 0;
  int rightSpeed = 0;

  // map joystick values to -2000 +2000 range
  int left = map(myMotors.motor_left, 0, 4000, -2000, 2000);
  int right = map(myMotors.motor_right, 0, 4000, -2000, 2000);

  // compute left motor speed
  // use high tolerance due to low joystick precision
  if (myMotors.motor_left < 1500 || myMotors.motor_left > 2000) {
    leftSpeed = map(left, -2000, 2000, 0, 180);
    // This is need because the left motor is 180 degree compared to the right one, so the setting the same value makes them going to opposite direction
    leftSpeed = 180 - leftSpeed; 
  }
  else {
    Serial.println("Stop left motor");
    leftSpeed = 90;
  }

  // compute right motor speed
  // use high tolerance due to low joystick precision
  if ( myMotors.motor_right < 1500 || myMotors.motor_right > 2000) {
    rightSpeed = map(right, -2000, 2000, 0, 180);
  }
  else {
    Serial.println("Stop right motor");
    rightSpeed = 90;
  }
  
  // Set motors speed
  rightMotor.write(rightSpeed);
  leftMotor.write(leftSpeed);

  // Activate/Deactivate weapon based on button state
  if (myMotors.button == 1)
  {
    weapon.write(0);
  }
  else {
    weapon.write(90);
  }
}
 
void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Change ESP32 Mac Address to the one of the ESP2866 for consisentcy
  esp_wifi_set_mac(WIFI_IF_STA, &newMACAddress[0]);

  // Init ESP-NOW
if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Attach pin to servo objects
  leftMotor.attach(LEFT_MOTOR_PIN,1000,2000);
  rightMotor.attach(RIGHT_MOTOR_PIN,1000,2000);
  weapon.attach(WEAPON_MOTOR_PIN,1000,2000);
  
  // Register callback function AKA what to do when data is received
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
}
 
void loop() {

}
