#include <U8x8lib.h>
#include "LIS3DHTR.h"

#ifdef SOFTWAREWIRE
	#include <SoftwareWire.h>
	SoftwareWire myWire(3, 2);
	LIS3DHTR<SoftwareWire> LIS;
	#define WIRE myWire
#else
	#include <Wire.h>
	LIS3DHTR<TwoWire> LIS;
	#define WIRE Wire
#endif

U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(U8X8_PIN_NONE);

#define LED 6
#define BUZZER 5
#define button 4 
#define potentiometer A0

#define moveOneFloorTime 4000
#define openCloseDoorTime 2000
#define doorRemainOpenTime 5000
#define closeConfirmationTime 600
int light_sensor = A3;

uint8_t upArrow[8] = {
  0b11000000, 
  0b11110000, 
  0b00111100,
  0b00001111, 
  0b00001111,
  0b00111100,
  0b11110000, 
  0b11000000
};

uint8_t downArrow[8] = {
  0b00000011, 
  0b00001111, 
  0b00111100,
  0b11110000, 
  0b11110000,
  0b00111100,
  0b00001111, 
  0b00000011
};

uint8_t arrowLine[8] = {
  0b00000000, 
  0b00000000, 
  0b00000000,
  0b11111111, 
  0b11111111,
  0b00000000,
  0b00000000, 
  0b00000000
};

uint8_t blank[8] = {
  0b00000000, 
  0b00000000, 
  0b00000000, 
  0b00000000, 
  0b00000000, 
  0b00000000, 
  0b00000000, 
  0b00000000
};

String floorsQueue = "";  //A queue containing all the destination floors
int currentFloorLevel[1] = {1}; //Current floor level
int targetFloorLevel[1] = {0};  //Next floor to visit
int state[1] = {0};  //0: idle; 1: moving upwards (up); 2: moving downwards (down); 3: stopping
int direction[1] = {0}; // 0: idle, 1: moving upwards (up), 2: moving downwards (down)
int skipOpenTime[1] = {0}; //Skip the 5s of remaining open if current state is IDLE

unsigned long prevCheckPointTime[1] = {0};  //Previous checkpoint of time
int ringChimeAgain[1] = {0};  //If the elevator is descending, turn this variable to 1 to ring the chime again
int doorState[1] = {0}; //0:open, 1: closed, 2: opening, 3: closing, 4: close confirmed, 5: door locked
int ledOn[1] = {0};  //When opening/closing the door, toggle this value to flash LED 

float x_acc[1] = {0};
float y_acc[1] = {0};

volatile int earthquakeStopMoving = 0;  //stop moving when there's horizontal acceleration
volatile int earthquakeMovingToClosestFloor = 0;  //then after 10 s withoud shaking, move to the closest floor
volatile int earthquakeOutOfOrder = 0;  //then report out of order

volatile int noHorizontalMovingTimeCount = 0; //count for the time without horizontal 
volatile int PWMTimeCount = 0;

//Removing a floor from floorsQueue
int removeFloorFromQueue(int floorLevel){
  String newFloorsQueue = "";
  //Add all characters != floorLevel to newFloorsQueue
  for(int i=0; i<floorsQueue.length(); i++){
    if(floorsQueue.charAt(i) != (char)(floorLevel + '0')){
      newFloorsQueue = newFloorsQueue + floorsQueue.charAt(i);
    }
  }
  if(newFloorsQueue.length() == floorsQueue.length() - 1){
    //Update the floorsQueue
    floorsQueue = newFloorsQueue;
    return 1;
  }else{
    Serial.println("ERROR");
    return 0;
  }
}

//Get next higher floor in floorsQueue
int getNextHigherFloor(int floorLevel){
  int nextHigherFloor = 10; //Any number larger than 5
  for(int i = 0; i < floorsQueue.length(); i++){
    if((floorsQueue[i] - '0' > floorLevel) && (floorsQueue[i] - '0' < nextHigherFloor)){
      nextHigherFloor = floorsQueue[i] - '0';
    }
  }
  if(nextHigherFloor == 10){
    return 0;
  }else{
    return nextHigherFloor;
  }
}

//Get next lower floor in floorsQueue
int getNextLowerFloor(int floorLevel){
  int nextLowerFloor = 0; //Any number smaller than 1
  for(int i = 0; i < floorsQueue.length(); i++){
    if((floorsQueue[i] - '0' < floorLevel) && (floorsQueue[i] - '0' > nextLowerFloor)){
      nextLowerFloor = floorsQueue[i] - '0';
    }
  }
  if(nextLowerFloor == 0){
    return 0;
  }else{
    return nextLowerFloor;
  }
}

//Display the non-highlighted floor numbers on OLED
void drawFloorLevel(int floor_level){
  u8x8.noInverse();
  switch (floor_level) {
    case 1:
      u8x8.draw2x2UTF8(1, 6, "1");
      break;
    case 2:
      u8x8.draw2x2UTF8(3, 3, "2");
      break;
    case 3:
      u8x8.draw2x2UTF8(0, 3, "3");
      break;
    case 4:
      u8x8.draw2x2UTF8(3, 0, "4");
      break;
    case 5:
      u8x8.draw2x2UTF8(0, 0, "5");
      break;
    default:
      break;
  }
}

//Display the highlighted floor numbers on OLED
void drawInverseFontFloorLevel(int floor_level){
  u8x8.inverse();
  switch (floor_level) {
    case 1:
      u8x8.draw2x2UTF8(1, 6, "1");
      break;
    case 2:
      u8x8.draw2x2UTF8(3, 3, "2");
      break;
    case 3:
      u8x8.draw2x2UTF8(0, 3, "3");
      break;
    case 4:
      u8x8.draw2x2UTF8(3, 0, "4");
      break;
    case 5:
      u8x8.draw2x2UTF8(0, 0, "5");
      break;
    default:
      break;
  }
}

//Check if each floor is selected as destination, and manages the display
void handleFloorLevelsDisplay(int currentSelectFloorLevel){
  for(int i = 1; i < 6; i++){
    if(i != currentSelectFloorLevel){
      if(floorIsSelected(i)){
        drawInverseFontFloorLevel(i);
      }else{
        drawFloorLevel(i);
      }
    }else{
      drawInverseFontFloorLevel(i);
    }
  }
}

//Display current floor on OLED
void handleCurrentFloorLevelDisplay(){
  u8x8.noInverse();
  switch (currentFloorLevel[0]) {
    case 1:
      u8x8.draw1x2UTF8(11, 0, "1");
      break;
    case 2:
      u8x8.draw1x2UTF8(11, 0, "2");
      break;
    case 3:
      u8x8.draw1x2UTF8(11, 0, "3");
      break;
    case 4:
      u8x8.draw1x2UTF8(11, 0, "4");
      break;
    case 5:
      u8x8.draw1x2UTF8(11, 0, "5");
      break;
    default:
      break;
  }
}

//Display the arrow to indicate moving direction
void handleArrowDisplay(){
  if(direction[0] == 1){  //Moving up
    u8x8.drawTile(13, 0, 1, upArrow);
    u8x8.drawTile(13, 1, 1, arrowLine);
  }else if(direction[0] == 2){  //Moving down
    u8x8.drawTile(13, 0, 1, arrowLine);
    u8x8.drawTile(13, 1, 1, downArrow);
  }else{  //idle
    u8x8.drawTile(13, 0, 1, blank);
    u8x8.drawTile(13, 1, 1, blank);  
  }
}

//Display the status of the door
void handleDoorStateDisplay(){
  u8x8.noInverse();
  if(doorState[0] == 0){
    u8x8.draw1x2UTF8(6, 2, "OPEN   ");
  }else if(doorState[0] == 1 || doorState[0] == 4 || doorState[0] == 5){
    u8x8.draw1x2UTF8(6, 2, "CLOSED  ");
  }else if(doorState[0] == 2){
    u8x8.draw1x2UTF8(6, 2, "OPENING");
  }else if(doorState[0] == 3){
    u8x8.draw1x2UTF8(6, 2, "CLOSING");
  }
}

//Select a certain floor
void selectFloor(int floorLevel){
  if((!floorIsSelected(floorLevel)) && (floorLevel != currentFloorLevel[0])){  //(This floor hasn't been previously selected) && (This floor isn't the current floor)
    floorsQueue = floorsQueue + floorLevel;
    if(floorsQueue.length() == 1 && state[0] == 0 && doorState[0] == 0){ //Update prevCheckPointTime when a button is pressed in IDLE state
      prevCheckPointTime[0] = millis();
    }
  }else if(floorLevel == currentFloorLevel[0]){ //Pressing the button of current floor level
    if(state[0] == 0 || state[0] == 3){ //At STOPPING/IDLE
      if(doorState[0] == 1 || doorState[0] == 3){  //If the door is closed (but not confirmed)/closing, open the door
        prevCheckPointTime[0] = millis();
        doorState[0] = 2;
      }else if(doorState[0] == 0){  //If the door is open, extend the time for the door to remain open
        prevCheckPointTime[0] = millis();
      }
    }
  }
}

//Select the floor & update the floor selection to OLED
void handleFloorSelection(int floorLevel, int isPressed){
  if(isPressed && !earthquakeOutOfOrder && !earthquakeStopMoving && !earthquakeMovingToClosestFloor){  //disable when earthquake
    selectFloor(floorLevel);
  }
  if(!earthquakeOutOfOrder && !earthquakeStopMoving && !earthquakeMovingToClosestFloor){
    handleFloorLevelsDisplay(floorLevel);
  }else{
    handleFloorLevelsDisplay(0);
  }
  
}

//Remove the floor from floorsQueue after arriving a floor & ring chime
void handleFloorQueueRemoveElement(){
    if(state[0] == 3){
      if(floorIsSelected(currentFloorLevel[0])){
        removeFloorFromQueue(currentFloorLevel[0]);
        if(direction[0] == 1){
          ringChime();
        }else if(direction[0] == 2){
          ringChime();
          ringChimeAgain[0] = 1;
        }
      }
    }
}

//display "DON'T PANIC" and "OUT OF ORDER"
void handleEarthquakeDisplay(){
  if(earthquakeStopMoving || earthquakeMovingToClosestFloor || earthquakeOutOfOrder){ //display"dont panic" if there's earthquake
    u8x8.setCursor(6, 4);
    u8x8.println("DON'T");
    u8x8.setCursor(6, 5);
    u8x8.println("PANIC!");
  }else{  //or display nothing
    u8x8.setCursor(6, 4);
    u8x8.println("     ");
    u8x8.setCursor(6, 5);
    u8x8.println("     ");
  }
  if(earthquakeOutOfOrder && doorState[0] == 0){  //display "out of order"
    u8x8.setCursor(10, 6);
    u8x8.println("OUT OF");
    u8x8.setCursor(10, 7);
    u8x8.println("ORDER!");
  }else if(earthquakeStopMoving){ //display "stopping"
    u8x8.setCursor(10, 6);
    u8x8.println("STOPP-");
    u8x8.setCursor(10, 7);
    u8x8.println("ING!  ");
  }else if(earthquakeMovingToClosestFloor){ //display "moving slowly"
    u8x8.setCursor(10, 6);
    u8x8.println("MOVING");
    u8x8.setCursor(10, 7);
    u8x8.println("SLOWLY");
  }else{  //clear the display
    u8x8.setCursor(10, 6);
    u8x8.println("      ");
    u8x8.setCursor(10, 7);
    u8x8.println("     ");
  }
}

//display "resetting"
void handleRecoverDisplay(){  
  //erase everything related to earthquake
  u8x8.setCursor(6, 4);
  u8x8.println("     ");
  u8x8.setCursor(6, 5);
  u8x8.println("     ");
  u8x8.setCursor(10, 6);
  u8x8.println("      ");
  u8x8.setCursor(10, 7);
  u8x8.println("     ");

  u8x8.setCursor(6, 6);
  u8x8.println("RESETTING");  //reset for 1 s
}

//This function check if a floor is contained in floorsQueue
int floorIsSelected(int floorLevel){
  for(int i = 0; i < floorsQueue.length(); i++){
    if(floorsQueue[i] - '0' == floorLevel){
      return 1;
    }
  }
  return 0;
}

//This function determines the movement of elevator
//Inputs: direction & floorsQueue; Outputs: targetFloorLevel
void handleElevatorMovement(){
  if(direction[0] == 0){  //Direction = IDLE
    if(floorsQueue.length()>0){ //If new tasks are assigned
      if((floorsQueue[0] - '0') > currentFloorLevel[0]){  //Move up
        direction[0] = 1;
        targetFloorLevel[0] = getNextHigherFloor(currentFloorLevel[0]);
      }else if((floorsQueue[0] - '0') < currentFloorLevel[0]){  //Move down
        direction[0] = 2;
        targetFloorLevel[0] = getNextLowerFloor(currentFloorLevel[0]);
      } 
    }
  }else if(direction[0] == 1){  //Direction =  UP
    if(floorsQueue.length() == 0){  //No pending tasks, switch to IDLE
      targetFloorLevel[0] = 0;
      direction[0] = 0;
    }else{
      if(getNextHigherFloor(currentFloorLevel[0]) == 0){ //Only descending tasks left
        targetFloorLevel[0] = getNextLowerFloor(currentFloorLevel[0]);
        direction[0] = 2;
      }else{
        targetFloorLevel[0] = getNextHigherFloor(currentFloorLevel[0]);
      }
    }
  }else if(direction[0] == 2){  //Direction =  DOWN
    if(floorsQueue.length() == 0){  //No pending tasks, switch to IDLE
      targetFloorLevel[0] = 0;
      direction[0] = 0;
    }else{
      if(getNextLowerFloor(currentFloorLevel[0]) == 0){  //Only ascending tasks left
        targetFloorLevel[0] = getNextHigherFloor(currentFloorLevel[0]);
        direction[0] = 1;
      }else{
        targetFloorLevel[0] = getNextLowerFloor(currentFloorLevel[0]);
      }
    }
  }
}

//Ring the chime
void ringChime(){
  tone(BUZZER, 300, 500);
}

//Flash the led whe opening/closing the door
void flashLED(){
  if(ledOn[0]){
    digitalWrite(LED, HIGH);
  }else{
    digitalWrite(LED, LOW);
  }
  ledOn[0] = 1 - ledOn[0];  //toggle
}


void setup() {
  pinMode(LED,OUTPUT); 
  pinMode(BUZZER, OUTPUT);
  pinMode(button, INPUT);
  pinMode(potentiometer, INPUT);

  u8x8.begin();
  u8x8.setFont(u8x8_font_chroma48medium8_r);
  Serial.begin(9600);
  while (!Serial){ };

  LIS.begin(WIRE, 0x19);
  while (!LIS){ };

  //Ranging from 1Hz to 5kHz
  LIS.setOutputDataRate(LIS3DHTR_DATARATE_10HZ);
  //"true" or "false"
  LIS.setHighSolution(true);
  //"POWER_MODE_NORMAL" or "POWER_MODE_LOW"
  LIS.setPowerMode(POWER_MODE_NORMAL);

  //Timer/Counter1 Control Registers A
  TCCR1A = 0; //reset TCCR0A to 0 
  //Timer/Counter1 Control Registers B
  TCCR1B = 0; //reset TCCR0B to 0 
  //Timer/Counter Register
  TCNT1 = 0; //reset counter to 0
  //Output Compare Register
  OCR1A = 6250; //10 Hz
  //Timer/Counter1 Control Registers B
  TCCR1B |= (1 << CS12) | (1 << WGM12); //256 prescalar //WGM12: Clear Timer on Compare match (CTC) mode, improves accuracy
  //Timer Interrupt Mask Register
  TIMSK1 |= (1 << OCIE1A); //enable interrupt //OCIE1A: Timer/Counter1, Output Compare A Match Interrupt Enable


  //For Timer0:
  //Timer/Counter0 Control Registers A
  TCCR0A = 0; //reset TCCR2A to 0 
  //Timer/Counter1 Control Registers B
  TCCR0B = 0; //reset TCCR2B to 0 
  //Timer/Counter Register
  TCNT0 = 0; //reset counter to 0
  //Output Compare Register
  OCR0A = 255; //counter compare value for Timer2
  //Timer/Counter1 Control Registers B
  TCCR0B |= (1 << CS00) | (1 << CS01) | (1 << WGM02); //64 prescalar
  //Timer Interrupt Mask Register
  TIMSK0 |= (1 << OCIE0A); //enable interrupt


  //OLED initialization
  drawFloorLevel(1);
  drawFloorLevel(2);
  drawFloorLevel(3);
  drawFloorLevel(4);
  drawFloorLevel(5);
  u8x8.draw1x2UTF8(6, 0, "FLOOR");

  handleCurrentFloorLevelDisplay();
  handleArrowDisplay();
}


void loop() {

  int floorLevel = round((float) analogRead(potentiometer)/1023 * 4) + 1; //Floor level indicated by potentiometer
  int isPressed = (digitalRead(button) == HIGH);
  int brightness = analogRead(light_sensor);  //If brightness < 50, there exists obstruction
  unsigned long currentTime = millis();

  x_acc[0] = LIS.getAccelerationX();  //read horizontal accelerations
  y_acc[0] = LIS.getAccelerationY();

  handleFloorSelection(floorLevel, isPressed);
  handleFloorQueueRemoveElement();
  handleElevatorMovement();

  if(isPressed && earthquakeOutOfOrder){   //if the button is pressed when earthquakeOutOfOrder
    earthquakeStopMoving = 0;
    earthquakeMovingToClosestFloor = 0;
    earthquakeOutOfOrder = 0;

    handleRecoverDisplay(); //reset the system for 2 s
    delay(2000);
    u8x8.setCursor(6, 6);
    u8x8.println("          "); //finish resetting
  }

  switch (state[0]) {
    case 0: //IDLE, no pending task
      if(floorsQueue.length()>0){ //If new task has been added
        state[0] = 3;
        skipOpenTime[0] = 1;
      }
      break;

    case 1: //MOVING UP
      //If time() > moveOneFloorTime, currentFloorLevel++;
      //add consideration of stopping at earthquake
      if(earthquakeStopMoving){
        prevCheckPointTime[0] = currentTime; //if earthquakeStopMoving, pause moving
      }

      if(earthquakeMovingToClosestFloor){
        if(currentTime - prevCheckPointTime[0] > 2 * moveOneFloorTime){ //Ascend by one floor, move slower if there's earthquake
          currentFloorLevel[0] = currentFloorLevel[0] + 1;
          prevCheckPointTime[0] = currentTime;  //Reset prevCheckPointTime
        }
      }else{
        if(currentTime - prevCheckPointTime[0] > moveOneFloorTime){ //Ascend by one floor
          currentFloorLevel[0] = currentFloorLevel[0] + 1;
          prevCheckPointTime[0] = currentTime;  //Reset prevCheckPointTime
        }
      }

      if(currentFloorLevel[0] == targetFloorLevel[0]){
        state[0] = 3; //transition to STOP state after reaching the target floor
      }
      break;

    case 2: //MOVING DOWN
      //If time() > moveOneFloorTime, currentFloorLevel--;
      //add consideration of stopping at earthquake
      if(earthquakeStopMoving){
        prevCheckPointTime[0] = currentTime; //if earthquakeStopMoving, pause moving
      }

      if(earthquakeMovingToClosestFloor){
        if(currentTime - prevCheckPointTime[0] > 2 * moveOneFloorTime){ //Descend by one floor, move slower if there's earthquake
          currentFloorLevel[0] = currentFloorLevel[0] - 1;
          prevCheckPointTime[0] = currentTime;  //Reset prevCheckPointTime
        }
      }else{
        if(currentTime - prevCheckPointTime[0] > moveOneFloorTime){ //Descend by one floor
          currentFloorLevel[0] = currentFloorLevel[0] - 1;
          prevCheckPointTime[0] = currentTime;  //Reset prevCheckPointTime
        }
      }

      if(currentFloorLevel[0] == targetFloorLevel[0]){
        state[0] = 3; //Transition to STOP state after reaching the target floor
      }
      break;

    case 3: //STOPPING AT A FLOOR
      if(doorState[0] == 0){  //Open
        if(floorsQueue.length() == 0){
          state[0] = 0;
        }
        if(skipOpenTime[0]){
          skipOpenTime[0] = 0;
          doorState[0] = 3;
        }else{
          if(currentTime - prevCheckPointTime[0] >= doorRemainOpenTime){
            prevCheckPointTime[0] = currentTime;
            doorState[0] = 3; //Closing
          }
        }
      }else if(doorState[0] == 1){  //Closed
        if(currentTime - prevCheckPointTime[0] >= closeConfirmationTime){ //After finishing waiting for closed confirmation
          prevCheckPointTime[0] = currentTime;
          doorState[0] = 4; //Close confirmation
        }
      }else if(doorState[0] == 2){  //Opening
        if(currentTime - prevCheckPointTime[0] >= openCloseDoorTime){ //After finishing opening the door
          prevCheckPointTime[0] = currentTime;
          doorState[0] = 0;
          ledOn[0] = 0; //Disable LED
        }
        if((currentTime - prevCheckPointTime[0] >= 1000) && ringChimeAgain[0] == 1){  //For descending, ring the chime again
          ringChime();
          ringChimeAgain[0] = 0;
        }
        flashLED();
      }else if(doorState[0] == 3){  //Closing
        if(currentTime - prevCheckPointTime[0] >= openCloseDoorTime){ //After finishing closing the door
          prevCheckPointTime[0] = currentTime;
          doorState[0] = 1;
          ledOn[0] = 0; //Disable LED
        }
        if(brightness < 50){
          prevCheckPointTime[0] = currentTime;
          doorState[0] = 2;
        }
        flashLED();
      }else if(doorState[0] == 4){  //Closed confirmed
        if(targetFloorLevel[0] == 0){
          state[0] = 0;
        }else if(targetFloorLevel[0] > currentFloorLevel[0]){ //Ascend if next target floor is above current floor
          state[0] = 1;
        }else if(targetFloorLevel[0] < currentFloorLevel[0]){ //Descend if next target floor is below current floor
          state[0] = 2;
        }
        doorState[0] = 5; //Lock the door after confirming that doors are closed
      }else if(doorState[0] == 5){  //Door locked
        doorState[0] = 2; //Unlock & open the door after reaching the next floor level
      }
      break;

    default:
      Serial.print("ERROR");
      break;
  }

  //Update display
  handleArrowDisplay();
  handleCurrentFloorLevelDisplay();
  handleDoorStateDisplay();
  handleEarthquakeDisplay();

  delay(20);
}


ISR(TIMER1_COMPA_vect){ //ISR for timer1

  if(!earthquakeStopMoving && !earthquakeMovingToClosestFloor && !earthquakeOutOfOrder){  //if the elevator is moving normally
    if(x_acc[0] * x_acc[0] + y_acc[0] * y_acc[0] >= 0.04){  //x^2 + y^2 >= 0.04, there's earthquake

      OCR1A = 1250; //set interrupt frequency to 50 Hz

      if(state[0] == 0 || state[0] == 3){ //idle or stopping
        if(doorState[0] == 0 || doorState[0] == 2){ //door open or opening
          targetFloorLevel[0] = 0;
          floorsQueue = "";
          earthquakeOutOfOrder = 1; //disable the elevator
        }else{  //closing or closed
          doorState[0] = 2; //open the door
          targetFloorLevel[0] = 0;
          floorsQueue = "";
          earthquakeOutOfOrder = 1; //stop the elevator
        }
      }else{

        if(state[0] == 1){  //is moving up
          targetFloorLevel[0] = currentFloorLevel[0] + 1;
        }else if(state[0] == 2){  //is moving down
          targetFloorLevel[0] = currentFloorLevel[0] - 1;
        }
        //couldn't find a smarter way to do this
        if(targetFloorLevel[0] == 1){
          floorsQueue = "1";
        }else if(targetFloorLevel[0] == 2){
          floorsQueue = "2";
        }else if(targetFloorLevel[0] == 3){
          floorsQueue = "3";
        }else if(targetFloorLevel[0] == 4){
          floorsQueue = "4";
        }else if(targetFloorLevel[0] == 5){
          floorsQueue = "5";
        }
        earthquakeStopMoving = 1;
      }
    }
  }
  
  if(earthquakeStopMoving){ //when the elevator is stopped
    if(x_acc[0] * x_acc[0] + y_acc[0] * y_acc[0] < 0.04){ //x^2 + y^2 < 0.04, earthquake stopped
      noHorizontalMovingTimeCount = noHorizontalMovingTimeCount + 1;  //time count ++
    }else{
      noHorizontalMovingTimeCount = 0;  //reset time count
    }
    if(noHorizontalMovingTimeCount >= 500){ //after 10 s
      //reset parameters
      OCR1A = 6250;  //set interrupt frequency back to 10 Hz
      noHorizontalMovingTimeCount = 0;
      earthquakeStopMoving = 0;
      earthquakeMovingToClosestFloor = 1;
    }
  }

  if(earthquakeMovingToClosestFloor){ //if door open, set to out of order
    if(doorState[0] == 0){
      earthquakeMovingToClosestFloor = 0;
      earthquakeOutOfOrder = 1;
    }
  }
}


ISR(TIMER0_COMPA_vect){ //ISR for timer0

  if(earthquakeStopMoving || earthquakeMovingToClosestFloor || earthquakeOutOfOrder){
    //duty cycle = 50%
    if(PWMTimeCount == 0 | PWMTimeCount == 1){
      PORTD = B00100000;  //sets digital pins 5 HIGH
    }else if(PWMTimeCount == 2 | PWMTimeCount == 3){
      PORTD = B00000000;  //sets digital pins 5 LOW
    }
    PWMTimeCount = (PWMTimeCount + 1) % 4;
  }

}


