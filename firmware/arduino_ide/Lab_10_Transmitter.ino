/*
 * nRF24L01 Transmitter with LCD
 * Sends motor commands and displays status on LCD
 * Receives status updates from receiver
 */

#include <SPI.h>
#include <RF24.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Pin definitions
#define CE_PIN 7
#define CSN_PIN 8

// Create RF24 object
RF24 radio(CE_PIN, CSN_PIN);

// Create LCD object (address 0x27, 16 columns, 2 rows)
// If 0x27 doesn't work, try 0x3F
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Address through which two modules communicate
const byte txAddress[6] = "00001";  // Transmit to receiver
const byte rxAddress[6] = "00002";  // Receive from receiver

// Current motor status
String motorStatus = "Stop";
String lastMotorStatus = "Stop";
String receiverStatus = "Unknown";

unsigned long lastStatusRequest = 0;
const unsigned long STATUS_REQUEST_INTERVAL = 500; // Request status every 500ms

void setup() {
  Serial.begin(9600);
  
  // Initialize LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("TX:");
  lcd.setCursor(8, 0);
  lcd.print("RX:");
  lcd.setCursor(0, 1);
  lcd.print(motorStatus);
  lcd.setCursor(8, 1);
  lcd.print(receiverStatus);
  
  // Initialize nRF24L01
  if (!radio.begin()) {
    Serial.println("nRF24L01 not responding!");
    while (1);
  }
  
  radio.openWritingPipe(txAddress);
  radio.openReadingPipe(1, rxAddress);
  radio.setPALevel(RF24_PA_LOW);
  radio.startListening(); // Start in listening mode
  
  Serial.println("=== nRF24L01 Transmitter Ready ===");
  Serial.println("Commands: F/f=Forward, R/r=Reverse, S/s=Stop");
}

void updateLCD() {
  // Update transmitter status (left side)
  lcd.setCursor(0, 1);
  lcd.print("        "); // Clear
  lcd.setCursor(0, 1);
  lcd.print(motorStatus);
  
  // Update receiver status (right side)
  lcd.setCursor(8, 1);
  lcd.print("        "); // Clear
  lcd.setCursor(8, 1);
  lcd.print(receiverStatus);
}

void sendCommand(String command) {
  radio.stopListening(); // Switch to transmit mode
  
  char text[32];
  command.toCharArray(text, sizeof(text));
  
  bool success = radio.write(&text, sizeof(text));
  
  if (success) {
    Serial.print("✓ Command sent: ");
    Serial.println(command);
  } else {
    Serial.println("✗ Transmission failed!");
  }
  
  radio.startListening(); // Switch back to listening mode
  delay(10); // Small delay to ensure mode switch
}

void loop() {
  unsigned long currentTime = millis();
  
  // Check for status updates from receiver
  if (radio.available()) {
    char text[32] = "";
    radio.read(&text, sizeof(text));
    
    String statusUpdate = String(text);
    statusUpdate.trim();
    
    if (statusUpdate.length() > 0) {
      receiverStatus = statusUpdate;
      updateLCD();
      Serial.print("← Receiver status: ");
      Serial.println(receiverStatus);
    }
  }
  
  // Handle serial commands
  if (Serial.available() > 0) {
    char command = Serial.read();
    
    // Clear any remaining characters in serial buffer
    while(Serial.available() > 0) {
      Serial.read();
    }
    
    // Process command
    if (command == 'F' || command == 'f') {
      motorStatus = "Forward";
    } 
    else if (command == 'R' || command == 'r') {
      motorStatus = "Reverse";
    } 
    else if (command == 'S' || command == 's') {
      motorStatus = "Stop";
    } 
    else {
      Serial.println("Invalid command! Use F/R/S");
      return;
    }
    
    // Send command
    sendCommand(motorStatus);
    lastMotorStatus = motorStatus;
    updateLCD();
  }
  
  // Periodically request status (optional - receiver sends automatically)
  if (currentTime - lastStatusRequest > STATUS_REQUEST_INTERVAL) {
    // The receiver will send status on any state change
    // This just updates the display
    lastStatusRequest = currentTime;
  }
}