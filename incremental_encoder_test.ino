// Sketch to test the functionality of the rotary encoder.
// Turn the encoder an watch on the serial monitor the movement steps.

volatile long temp, counter = 0; //This variable will increase or decrease depending on the rotation of encoder


void setup() {


  Serial.begin (115200);

  pinMode(16, INPUT_PULLUP);

  pinMode(17, INPUT_PULLUP);
  //Setting up interrupt
  //A rising pulse from encodenren activated ai0(). AttachInterrupt 0 is DigitalPin nr 16
  attachInterrupt(digitalPinToInterrupt(16), ai0, RISING);
  //B rising pulse from encodenren activated ai1(). AttachInterrupt 1 is DigitalPin nr 17
  attachInterrupt(digitalPinToInterrupt(17), ai1, RISING);

}

void loop() {
  // Send the value of counter
  if ( counter != temp ) {
    Serial.println (counter);
    temp = counter;
  }
}

void ai0() {
  // ai0 is activated if DigitalPin nr 2 is going from LOW to HIGH
  // Check pin 17 to determine the direction
  if (digitalRead(17) == LOW) {
    counter++;
  } else {
    counter--;
  }
}

void ai1() {
  // ai0 is activated if DigitalPin nr 3 is going from LOW to HIGH
  // Check with pin 16 to determine the direction
  if (digitalRead(16) == LOW) {
    counter--;
  } else {
    counter++;
  }
}
