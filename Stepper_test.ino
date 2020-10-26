// Sketch to test the functionality of the Nema17 stepper motor
// Turns the motor with a constant speed

#include <AccelStepper.h>

AccelStepper stepper(2, 33, 32); // Motortyp, STEP_PIN, DIR_PIN

void setup()
{
  pinMode(14, OUTPUT); // DRV8025 ENABLE

  stepper.setMaxSpeed(3000);
  stepper.setSpeed(500);
  stepper.enableOutputs();
}

void loop()
{
  stepper.runSpeed();
}
