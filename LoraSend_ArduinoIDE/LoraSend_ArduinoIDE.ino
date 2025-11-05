#include <SPI.h>
#include <LoRa.h>
#include <esp_task_wdt.h> // Include the Watchdog Timer library for ESP32

// ----- Pin Definitions -----
// LoRa Module
#define ss 5
#define rst 1   // <-- Make sure this pin is correct, 1 is often TX
#define dio0 22

// E-Stop Button and LED
#define BUTTON_PIN 12 // Arbitrary pin for the button
#define LED_PIN 13    // Arbitrary pin for the LED

// ----- Timing Settings -----
#define HEARTBEAT_INTERVAL 500 // Send "chill" packet every 500ms
#define ESTOP_INTERVAL 100     // Send "stop" packet every 100ms (faster)
#define WDT_TIMEOUT 3          // 3-second watchdog timeout

// ----- Global Variables -----
unsigned long lastSendTime = 0; // Tracks the last time a packet was sent

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("LoRa E-Stop Sender");

  // --- Configure Button and LED ---
  // We use INPUT_PULLUP so the pin reads HIGH by default.
  // Connect the button from BUTTON_PIN to GND. No other resistor needed.
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  // Set the LED pin as an output
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW); // Start with LED off

  // --- Configure LoRa Module ---
  LoRa.setPins(ss, rst, dio0);
  while (!LoRa.begin(915E6)) {
    Serial.println(".");
    delay(500);
  }
  LoRa.setSyncWord(0xB);
  Serial.println("LoRa Initializing OK!");

  // --- Configure Watchdog Timer ---
  Serial.println("Initializing Watchdog Timer...");
  // Initialize WDT with a 3-second timeout and enable panic (reboot)
  esp_task_wdt_init(WDT_TIMEOUT, true);
  // Add the current task (this loop) to the WDT
  esp_task_wdt_add(NULL);
}

// Helper function to send the LoRa packet
void sendLoRaPacket(byte message) {
  Serial.print("Sending packet: 0x");
  if (message < 0x10) Serial.print("0"); // Add leading zero for 0x00
  Serial.println(message, HEX);

  LoRa.beginPacket();
  LoRa.write(message); // Use .write() for raw bytes
  LoRa.endPacket();
}

void loop() {
  // --- 1. Read the button state ---
  // digitalRead will be LOW when pressed because we used INPUT_PULLUP.
  bool isButtonPressed = (digitalRead(BUTTON_PIN) == LOW);

  // --- 2. Handle E-Stop (Button PRESSED) ---
  if (isButtonPressed) {
    digitalWrite(LED_PIN, HIGH); // Turn on the LED

    // Check if it's time to send the rapid-fire E-STOP packet
    if (millis() - lastSendTime > ESTOP_INTERVAL) {
      lastSendTime = millis();    // Reset the send timer
      sendLoRaPacket(0xFF);       // Send the "STOP" command
    }
  }
  // --- 3. Handle Heartbeat (Button NOT pressed) ---
  else {
    digitalWrite(LED_PIN, LOW); // Turn off the LED

    // Check if it's time to send the normal "CHILL" heartbeat
    if (millis() - lastSendTime > HEARTBEAT_INTERVAL) {
      lastSendTime = millis();    // Reset the send timer
      sendLoRaPacket(0x00);       // Send the "CHILL" command
    }
  }

  // --- 4. Feed the Watchdog ---
  // This line tells the WDT that the code is not frozen.
  // If this line doesn't run for 3 seconds, the ESP32 will reboot.
  esp_task_wdt_reset();
}