//gpio pin
const int buttonPin = 23;

void setup() {
  // Start the Serial Monitor so we can see the output
  Serial.begin(115200);
  
  // Initialize the button pin as an input with an internal pull-up resistor.
  // This means the pin will be HIGH (3.3V) by default.
  // When the button is pressed (connecting it to GND), the pin will go LOW (0V).
  pinMode(buttonPin, INPUT_PULLUP);
  
  Serial.println("Button test started. Press the button...");
}

void loop() {
  // Read the state of the button
  int buttonState = digitalRead(buttonPin);

  // Check if the button is pressed
  // Because we are using INPUT_PULLUP, a LOW signal means the button is pressed.
  if (buttonState == LOW) {
    Serial.println("Button is PRESSED");
  } else {
    // The button is not pressed (in the HIGH state)
    Serial.println("Button is released");
  }

  // Wait 100 milliseconds before checking again
  // This avoids spamming the Serial Monitor and acts as a simple debounce.
  delay(100);
}