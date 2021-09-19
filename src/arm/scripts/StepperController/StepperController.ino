#include "pins.h"
#include "StepperController.h"
#include <AccelStepper.h>

AccelStepper stepper_x(AccelStepper::DRIVER, STR3_X_STEP, STR3_X_DIR, 0, 0, false);
AccelStepper stepper_y1(AccelStepper::DRIVER, STR3_Y1_STEP, STR3_Y1_DIR, 0, 0, false);
AccelStepper stepper_y2(AccelStepper::DRIVER, STR3_Y2_STEP, STR3_Y2_DIR, 0, 0, false);



void blinkLight(){
  digitalWrite(LED_PIN, LOW);
  delay(1000);
  digitalWrite(LED_PIN, HIGH);
  delay(1000);
  digitalWrite(LED_PIN, LOW);
  delay(1000);
  digitalWrite(LED_PIN, HIGH);
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

const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];        // temporary array for use when parsing
boolean newData = false;

// Read serial input
void recvWithStartEndMarkers() {
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;

  while(Serial.available() > 0 && newData == false){
    rc = Serial.read();
    if(recvInProgress == true){
      if(rc != endMarker){
        receivedChars[ndx] = rc;
        ndx++;
        if(ndx >= numChars){
          ndx = numChars - 1;
          }
        }
        else{
          receivedChars[ndx] = '\0'; // terminate the string
          recvInProgress = false;
          ndx = 0;
          newData = true;
        }
      }
      else if(rc == startMarker){
        recvInProgress = true;
    }
  }
}

int xInt = 0;
int yInt = 0;

// Parse serial input
void parseData(){
  char *strtokIndx;                         // this is used by strtok() as an index
  strtokIndx = strtok(tempChars,",");       // get the first part - the string
  xInt = atoi(strtokIndx);                  // convert this part to an integer

  strtokIndx = strtok(NULL, ",");           // this continues where the previous call left off
  yInt = atoi(strtokIndx);                  // convert this part to an integer
}

// Serial print the parsed input
void showParsedData() {
  Serial.print("X : ");
  Serial.print(xInt);
  Serial.print(", Y : ");
  Serial.println(yInt);
}

// Funtion the wait until data is received
void readData(){
  while(1){
    recvWithStartEndMarkers();
    if(newData == true){
      strcpy(tempChars, receivedChars);
      // this temporary copy is necessary to protect the original data
      // because strtok() used in parseData() replaces the commas with \0
      parseData();
      newData = false;
      break;
    }
  }
}

struct LimitStatus checkLimits() {
  static LimitStatus limitStatus;

  Serial.println(limitStatus.xMinCount);

  if(!digitalRead(LIM_X1_MIN) && !digitalRead(LIM_X2_MIN)){
    if(++limitStatus.xMinCount >= PAST_READING_REQ) {
      limitStatus.xAxis = LimitState::Min;
    }
  }
  else if(!digitalRead(LIM_X1_MAX) && !digitalRead(LIM_X2_MAX)){
    if(++limitStatus.xMaxCount >= PAST_READING_REQ) {
    limitStatus.xAxis = LimitState::Max;
    }
  }
  else {
    limitStatus.xAxis = LimitState::None;
    limitStatus.xMinCount = 0;
    limitStatus.xMaxCount = 0;
  }

  if(!digitalRead(LIM_Y1_MIN) && !digitalRead(LIM_Y2_MIN)){
    if(++limitStatus.yMinCount >= PAST_READING_REQ) {
      limitStatus.yAxis = LimitState::Min;
    }
  }
  else if(!digitalRead(LIM_Y1_MAX) && !digitalRead(LIM_Y2_MAX)){
    if(++limitStatus.yMaxCount >= PAST_READING_REQ) {
      limitStatus.yAxis = LimitState::Max;
    }
  }
    else {
    limitStatus.yAxis = LimitState::None;
    limitStatus.yMinCount = 0;
    limitStatus.yMaxCount = 0;
  }

  return limitStatus;
}

// Function to home X-axis
void homeXAxis(){
  // Reducing speed for acuracy
  stepper_x.setMaxSpeed(MIN_SPEED);
  delay(5);
  
  long initial_homing=1;
  
  Serial.print("Homing X-axis...");
  // Move until X-min limit switch is hit
  while(checkLimits().xAxis != LimitState::Min){
    stepper_x.moveTo(initial_homing);
    initial_homing++;
    stepper_x.run();
    delay(5);
  }
  stepper_x.setCurrentPosition(0);
  initial_homing=-1;
  // Move until X-min limit switch is deactivated
  while(checkLimits().xAxis != LimitState::None){
    stepper_x.moveTo(initial_homing);
    initial_homing--;
    stepper_x.run();
    delay(5);
  }
  stepper_x.moveTo(initial_homing-10);
  while(stepper_x.distanceToGo() != 0){
    stepper_x.run();
  }
  
  stepper_x.setCurrentPosition(0);
  stepper_x.stop();

  stepper_x.setMaxSpeed(MAX_SPEED);
  delay(5);
      
  Serial.println("Completed");
}

// Function to home Y-axis
void homeYAxis(){
  // Reducing speed for acuracy
  stepper_y1.setMaxSpeed(MIN_SPEED);
  stepper_y2.setMaxSpeed(MIN_SPEED);
  delay(5);
  
  long initial_homing=-1;
  
  Serial.print("Homing Y-axis...");
  // Move until Y-min limit switch is hit
  while(checkLimits().yAxis != LimitState::Min){
    stepper_y1.moveTo(initial_homing);
    stepper_y2.moveTo(initial_homing);
    initial_homing--;
    stepper_y1.run();
    stepper_y2.run();
    delay(5);
  }
  stepper_y1.setCurrentPosition(0);
  stepper_y2.setCurrentPosition(0);
  initial_homing=1;
  // Move until Y-min limit switch is deactivated
  while(checkLimits().yAxis != LimitState::None){
    stepper_y1.moveTo(initial_homing);
    stepper_y2.moveTo(initial_homing);
    initial_homing++;
    stepper_y1.run();
    stepper_y2.run();
    delay(5);
  }
  stepper_y1.moveTo(initial_homing+10);
  while(stepper_y1.distanceToGo() != 0){
    stepper_y1.run();
    stepper_y2.run();
  }
  
  stepper_y1.setCurrentPosition(0);
  stepper_y2.setCurrentPosition(0);

  stepper_y1.stop();
  stepper_y2.stop();

  stepper_y1.setMaxSpeed(MAX_SPEED);
  stepper_y2.setMaxSpeed(MAX_SPEED);
  delay(5);
      
  Serial.println("Completed");
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  
  stepperEnablePins();
  stepperInvertPins();
  stepperInitialize();
  
  pullUPLimitSwitches();
  
  // initialize the serial port:
  Serial.begin(115200);
  blinkLight(); // Signal that code has started
}

robot_states current_state = WAITING_TO_START;

bool needHandleLimits(struct LimitStatus limitStatus) {
  if(limitStatus.xAxis == LimitState::Min){
    current_state = X_MIN_LS_HIT;
    stepper_x.stop();
  } 
  else if (limitStatus.xAxis == LimitState::Max){
    current_state = X_MAX_LS_HIT;
    stepper_x.stop();
  }
  else if (limitStatus.yAxis == LimitState::Min){
    current_state = Y_MIN_LS_HIT;
    stepper_y1.stop();
    stepper_y2.stop();
  }
  else if (limitStatus.yAxis == LimitState::Max){
    current_state = Y_MAX_LS_HIT;
    stepper_y1.stop();
    stepper_y2.stop();
  }

  return current_state != MOVING;
}

void loop(){
  LimitStatus limitStatus = checkLimits();

  switch(current_state){
    case WAITING_TO_START:
      Serial.println("Waiting to Start");
      readData();
      showParsedData();
      if(xInt == -99 && yInt == -99){
        current_state = HOMING;
        Serial.println("Robot Started");
      }
      break;
    case HOMING:
      Serial.println("Homing");
      homeXAxis();
      homeYAxis();
      Serial.println("Homing Completed");
      current_state = WAITING_FOR_INSTRUCTION;
      break;
    case WAITING_FOR_INSTRUCTION:
      Serial.println("Waiting for instruction");
      readData();
      showParsedData();
      if(xInt == -99 && yInt == -99){
        current_state = HOMING;
      }
      else{
        stepper_x.moveTo(xInt);
        stepper_y1.moveTo(yInt);
        stepper_y2.moveTo(yInt);
        current_state = MOVING;
        Serial.println("Moving");
      }
      break;
    case MOVING:
      if(!needHandleLimits(limitStatus)){
        if(stepper_x.distanceToGo() != 0){
          stepper_x.run();
        }

        if(stepper_y1.distanceToGo() != 0 && stepper_y2.distanceToGo() != 0){
         stepper_y1.run();
         stepper_y2.run();
         }

        if (stepper_y1.distanceToGo() == 0 && stepper_y2.distanceToGo() == 0 && stepper_x.distanceToGo() == 0) {
          current_state = WAITING_FOR_INSTRUCTION;
          Serial.println("Move Completed");
        }
      }
      break;
    case X_MIN_LS_HIT:
      Serial.println("X_MIN Limit switch hit");
      homeXAxis();
      stepper_x.moveTo(stepper_x.currentPosition());
      current_state = MOVING;
      break;
    case X_MAX_LS_HIT:
      Serial.println("X_MAX Limit switch hit");
      if(limitStatus.xAxis == LimitState::Max){
        stepper_x.move(1);
      }
      else {
        stepper_x.moveTo(stepper_x.currentPosition());
        current_state = MOVING;
      }
      
      break;
    case Y_MIN_LS_HIT:
      Serial.println("Y_MIN Limit switch hit");
      homeYAxis();
      stepper_y1.moveTo(stepper_y1.currentPosition());
      stepper_y2.moveTo(stepper_y2.currentPosition());
      current_state = MOVING;
      break;
    case Y_MAX_LS_HIT:
      Serial.println("Y_MAX Limit switch hit");
      if(limitStatus.yAxis == LimitState::Max){
        stepper_y1.move(-1);
        stepper_y2.move(-1);
      }
      else {
        stepper_y1.moveTo(stepper_y1.currentPosition());
        stepper_y2.moveTo(stepper_y2.currentPosition());
        current_state = MOVING;
      }
      break;
  }
        
  delay(5);
}
