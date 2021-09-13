#include "pins.h"
#include "StepperController.h"
#include <AccelStepper.h>

AccelStepper stepper_x(AccelStepper::DRIVER, STR3_X_STEP, STR3_X_DIR, 0, 0, false);
AccelStepper stepper_y1(AccelStepper::DRIVER, STR3_Y1_STEP, STR3_Y1_DIR, 0, 0, false);
AccelStepper stepper_y2(AccelStepper::DRIVER, STR3_Y2_STEP, STR3_Y2_DIR, 0, 0, false);

void pullUPLimitSwitches(){
  pinMode(LIM_X1_MIN, INPUT_PULLUP);
  pinMode(LIM_X2_MIN, INPUT_PULLUP);
  pinMode(LIM_X1_MAX, INPUT_PULLUP);
 pinMode(LIM_X2_MAX, INPUT_PULLUP);
  
 pinMode(LIM_Y1_MIN, INPUT_PULLUP);
 pinMode(LIM_Y2_MIN, INPUT_PULLUP);
 pinMode(LIM_Y1_MAX, INPUT_PULLUP);
 pinMode(LIM_Y2_MAX, INPUT_PULLUP);
}


void stepperEnablePins(){
  stepper_x.setEnablePin(STR3_X_EN);
 stepper_y1.setEnablePin(STR3_Y1_EN);
 stepper_y2.setEnablePin(STR3_Y2_EN);
}


void stepperInvertPins(){
  stepper_x.setPinsInverted(true, true, true);
 stepper_y1.setPinsInverted(true, true, true);
 stepper_y2.setPinsInverted(true, true, true);
}

void stepperInitialize(){
  stepper_x.setMaxSpeed(MAX_SPEED);
  stepper_x.setAcceleration(ACCELERATION);
  stepper_x.enableOutputs();
  
 stepper_y1.setMaxSpeed(MAX_SPEED);
 stepper_y1.setAcceleration(ACCELERATION);
 stepper_y1.enableOutputs();
 
 stepper_y2.setMaxSpeed(MAX_SPEED);
 stepper_y2.setAcceleration(ACCELERATION);
 stepper_y2.enableOutputs();
}

void setup() {
 
  pullUPLimitSwitches();

  stepperEnablePins();
  stepperInvertPins();
  stepperInitialize();
  
  // initialize the serial port:
  Serial.begin(115200);

  delay(10);

  Serial.println("Done setup!");
}

void loop() {
  // put your main code here, to run repeatedly:
  if(!digitalRead(LIM_X1_MIN)) {
    Serial.println("LIM_X1_MIN");
  }
  if(!digitalRead(LIM_X2_MIN)) {
    Serial.println("LIM_X2_MIN");
  }
  if(!digitalRead(LIM_X1_MAX)) {
    Serial.println("LIM_X1_MAX");
  }
  if(!digitalRead(LIM_X2_MAX)) {
    Serial.println("LIM_X2_MAX");
  }
  if(!digitalRead(LIM_Y1_MIN)) {
    Serial.println("LIM_Y1_MIN");
  }
  if(!digitalRead(LIM_Y2_MIN)) {
    Serial.println("LIM_Y2_MIN");
  }
  if(!digitalRead(LIM_Y1_MAX)) {
    Serial.println("LIM_Y1_MAX");
  }
  if(!digitalRead(LIM_Y2_MAX)) {
    Serial.println("LIM_Y2_MAX");
  }

  delay(10);

}
