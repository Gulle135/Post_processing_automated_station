// ##### Libraries #####

#include <LiquidCrystal.h>
#include <AccelStepper.h>
#include <Servo.h>

// ##### Wiring pins #####

// Stepper - z-axis
const int STEPPER_Z_STEP = 52;
const int STEPPER_Z_DIR = 53;

// Stepper - stirring
const int STEPPER_STIR_STEP = 40;
const int STEPPER_STIR_DIR = 41;

// Stepper - print rotater
const int STEPPER_ROT_STEP = 43;
const int STEPPER_ROT_DIR = 42;

// Servo left
const int SERVO_LEFT_SIG = 45;
// VCC to 7.5V powersupply
// GND to GND powersupply and arduino

// Servo right
const int SERVO_RIGHT_SIG = 44;
// VCC to 7.5V powersupply
// GND to GND powersupply and arduino

// END-stop
const int END_STOP = 48;
// VCC to 5V arduino
// GND to GND arduino

// Fan
const int FAN = 31;
// VCC to 24V powersupply
// GND to IRFZ48N MOSFET to GND powersupply

// Solenoid valve
const int VALVE = 47;
// VCC to 24V powersupply
// GND to IRFZ48N MOSFET to GND powersupply

// UV light
const int UV = 39;
// VCC to 390Ohm resistor to 24V powersupply
// GND to IRFZ48N MOSFET to GND powersupply

// Heating element
const int HEAT;// = 24;
// VCC 24V powersupply
// GND IRFZ48N MOSFET to GND powersupply

// Thermistor
const int TEMP_PIN = A0;
// VCC to 5V arduino
// GND to GND arduino

// Rotary encoder pins
const int CLK = 2;
const int DT = 3;
const int SW = 4;
// + to 5V arduino
// GND to GND arduino

// LCD pins
LiquidCrystal lcd(10, 9, 8, 7, 6, 5);
// GND to GND arduino
// VDD to 220Ohm to 5V arduino
// V0 to digital pin 5 arduino
// RS to digital pin 6 arduino
// RW to digital pin 7 arduino
// E to digital pin 8 arduino
// DB4 to digital pin 9 arduino
// DB5 to GND arduino
// DB6 to digital pin 10 arduino
// DB7 to GND arduino
// BL1 to 5V arduino
// BL2 to GND arduino

// ##### Initializing objects #####

// Stepper - z-axis
AccelStepper STEPPER_Z(AccelStepper::DRIVER,STEPPER_Z_STEP,STEPPER_Z_DIR);

// Stepper - stirring
AccelStepper STEPPER_STIR(AccelStepper::DRIVER,STEPPER_STIR_STEP,STEPPER_STIR_DIR);

// Stepper - print rotater
AccelStepper STEPPER_ROT(AccelStepper::DRIVER,STEPPER_ROT_STEP,STEPPER_ROT_DIR);

// Servo left
Servo SERVO_LEFT;

// Servo right
Servo SERVO_RIGHT;

// ##### Other variables #####

// Stepper motors
const int STEPS_PER_REV = 400;
const float DESIRED_SPEED_Z = 200;
const float DESIRED_SPEED_STIR = 1000;
const float DESIRED_SPEED_ROT = 50;

// Z-axis positions
const long POSITION_HOME = 0;
const long POSITION_WASH = 9500;
const long POSITION_CURE = 1500;

// Servo positions
int servoLeftPos = 95;
int servoRightPos = 5;

// Thermistor
int tempSensorVal;
int actualTemp;
const unsigned long TEMP_UPDATE = 1000;
unsigned long previousTempUpdate;

// Rotary encoder CLK-state
int currentStateCLK;
int lastStateCLK;
int oldBtnState = HIGH;

// Initial configurement
unsigned long washTime = 15;
int cureTemp = 25;
unsigned long cureTime = 5;

// Configurement limits
const int LOWER_WASH_LIMIT = 1;
const int UPPER_WASH_LIMIT = 60;
const int LOWER_TEMP_LIMIT = 25;
const int UPPER_TEMP_LIMIT = 60;
const int LOWER_CURE_LIMIT = 1;
const int UPPER_CURE_LIMIT = 30;

// Starttimers
unsigned long washStarttime;
unsigned long cureStarttime;

// Remaining time
unsigned long washTimeLeft;
unsigned long cureTimeLeft;

// Previous remaining time
unsigned long previousWashTimeLeft;
unsigned long previousCureTimeLeft;

// Initial menu selection
int menu = 23;

// ##### Functions #####

// LCD update function
void updateMenu() {
  switch (menu) {
    case 0:
      menu = 1;
      break;
    case 1:
      lcd.clear();
      lcd.print(">Start");
      lcd.setCursor(0,1);
      lcd.print(" Configure");
      break;
    case 2:
      lcd.clear();
      lcd.print(" Start");
      lcd.setCursor(0,1);
      lcd.print(">Configure");
      break;
    case 3:
      menu = 2;
      break;
    case 4:
      menu = 5;
      break;
    case 5:
      lcd.clear();
      lcd.print(">Wash: ");
      lcd.print(washTime);
      lcd.print("min");
      lcd.setCursor(0,1);
      lcd.print(" Temp: ");
      lcd.print(cureTemp);
      lcd.print("C");
      lcd.print((char)223);
      break;
    case 6:
      lcd.clear();
      lcd.print(" Wash: ");
      lcd.print(washTime);
      lcd.print("min");
      lcd.setCursor(0,1);
      lcd.print(">Temp: ");
      lcd.print(cureTemp);
      lcd.print("C");
      lcd.print((char)223);
      break;
    case 7:
      lcd.clear();
      lcd.print(">Cure: ");
      lcd.print(cureTime);
      lcd.print("min");
      lcd.setCursor(0,1);
      lcd.print(" Back");
      break;
    case 8:
      lcd.clear();
      lcd.print(" Cure: ");
      lcd.print(cureTime);
      lcd.print("min");
      lcd.setCursor(0,1);
      lcd.print(">Back");
      break;
    case 9:
      menu = 8;
      break;
    case 10:
      menu = 11;
      break;
    case 11:
      lcd.clear();
      lcd.print("Washing: ");
      lcd.print(washTimeLeft);
      lcd.print("min");
      lcd.setCursor(0,1);
      lcd.print(">Stop");
      break;
    case 12:
      menu = 11;
      break;
    case 13:
      menu = 14;
      break;
    case 14:
      lcd.clear();
      lcd.print("Curing: ");
      lcd.print(cureTimeLeft);
      lcd.print("min");
      lcd.setCursor(0,1);
      lcd.print(">Stop");
      break;
    case 15:
      menu = 14;
      break;
    case 16:
      lcd.clear();
      lcd.print(">Wash: ");
      lcd.print(washTime);
      lcd.print("min");
      break;
    case 17:
      lcd.clear();
      lcd.print(">Temp: ");
      lcd.print(cureTemp);
      lcd.print("C");
      lcd.print((char)223);
      break;
    case 18:
      lcd.clear();
      lcd.print(">Cure: ");
      lcd.print(cureTime);
      lcd.print("min");
      break;
    case 19:
      menu = 20;
      break;
    case 20:
      lcd.clear();
      lcd.print(actualTemp);
      lcd.print("C");
      lcd.print((char)223);
      lcd.print(" -> ");
      lcd.print(cureTemp);
      lcd.print("C");
      lcd.print((char)223);
      lcd.setCursor(0,1);
      lcd.print(">Stop");
      break;
    case 21:
      menu = 20;
      break;
    case 22:
      menu = 23;
      break;
    case 23:
      lcd.clear();
      lcd.print("Calibrating...");
      break;
    case 24:
      menu = 23;
      break;
  }
}

void menuSelection() {
  // Read the current state of CLK
  currentStateCLK = digitalRead(CLK);
  
  // If last and current state of CLK are different, then pulse occurred
  // React to only 1 state change to avoid double count
  if (currentStateCLK != lastStateCLK  && currentStateCLK == 1){
    if (menu == 16) {
      if (digitalRead(DT) != currentStateCLK and washTime > LOWER_WASH_LIMIT) {
        washTime--;
        updateMenu();
      } 
      else if (washTime < UPPER_WASH_LIMIT) {
        // Encoder is rotating CW so increment
        washTime++;
        updateMenu();
      }
    }
    else if (menu == 17) {
      if (digitalRead(DT) != currentStateCLK and cureTemp > LOWER_TEMP_LIMIT) {
        cureTemp--;
        updateMenu();
      } 
      else if (cureTemp < UPPER_TEMP_LIMIT) {
        // Encoder is rotating CW so increment
        cureTemp++;
        updateMenu();
      }
    }
    else if (menu == 18) {
      if (digitalRead(DT) != currentStateCLK and cureTime > LOWER_CURE_LIMIT) {
        cureTime--;
        updateMenu();
      } 
      else if (cureTime < UPPER_CURE_LIMIT) {
        // Encoder is rotating CW so increment
        cureTime++;
        updateMenu();
      }
    }
    else {
      // If the DT state is different than the CLK state then
      // the encoder is rotating CCW so decrement
      if (digitalRead(DT) != currentStateCLK) {
        menu--;
        updateMenu();
      } 
      else {
        // Encoder is rotating CW so increment
        menu++;
        updateMenu();
      }
    }
  }

  // Remember last CLK state
  lastStateCLK = currentStateCLK;

  // Read the button state
  int btnState = digitalRead(SW);

  //If we detect LOW signal, button is pressed
  if (btnState == LOW && oldBtnState != btnState) {
    if (menu == 2)  {
      menu = 5;
      updateMenu();
    }
    else if (menu == 8) {
      menu = 2;
      updateMenu();
    }
    else if (menu == 1) {
      openLid();
      delay(2000);
      STEPPER_Z.runToNewPosition(POSITION_WASH);
      washStarttime = millis();
      menu = 11;
      updateMenu();
    }
    else if (menu == 11) {
      STEPPER_Z.runToNewPosition(POSITION_HOME);
      menu = 1;
      updateMenu();
    }
    else if (menu == 14) {
      STEPPER_Z.runToNewPosition(POSITION_HOME);
      menu = 1;
      updateMenu();
    }
    else if (menu == 20) {
      STEPPER_Z.runToNewPosition(POSITION_HOME);
      menu = 1;
      updateMenu();
    }
    else if (menu == 5) {
      menu = 16;
      updateMenu();
    }
    else if (menu == 16)  {
      menu = 5;
      updateMenu();
    }
    else if (menu == 6)  {
      menu = 17;
      updateMenu();
    }
    else if (menu == 17)  {
      menu = 6;
      updateMenu();
    }
    else if (menu == 7)  {
      menu = 18;
      updateMenu();
    }
    else if (menu == 18)  {
      menu = 7;
      updateMenu();
    }
  }
  else if (menu == 11) {
    washRotation();
    washTimeLeft = washTime - ((millis() - washStarttime) / 60000);
    if (previousWashTimeLeft != washTimeLeft) {
      previousWashTimeLeft = washTimeLeft;
      updateMenu();
    }
    if (washTimeLeft == 0) {
      STEPPER_Z.runToNewPosition(POSITION_CURE);
      delay(2000);
      closeLid();
      delay(1000);
      drying();
      menu = 20;
      updateMenu();
    }
  }
  else if (menu == 14) {
    digitalWrite(UV, HIGH);
    runSteps(&STEPPER_ROT, DESIRED_SPEED_ROT, 400);
    cureTimeLeft = cureTime - ((millis() - cureStarttime) / 60000);
    if (previousCureTimeLeft != cureTimeLeft) {
      previousCureTimeLeft = cureTimeLeft;
      updateMenu();
    }
    if (cureTimeLeft == 0) {
      digitalWrite(HEAT, LOW);
      digitalWrite(FAN, LOW);
      digitalWrite(UV, LOW);
      STEPPER_Z.runToNewPosition(POSITION_HOME);
      menu = 1;
      updateMenu();
    }
  }
  else if (menu == 20) {
    digitalWrite(FAN, HIGH);
    tempSensorVal = analogRead(TEMP_PIN);
    actualTemp = (int)((((tempSensorVal / 1024.0) * 5.0) - 0.5) * 100);
    if (millis() - previousTempUpdate > TEMP_UPDATE) {
      if (actualTemp >= cureTemp) {
        cureStarttime = millis();
        menu = 14;
        updateMenu();
        delay(500);
      }
      else {
        previousTempUpdate = millis();
        updateMenu();
      }
    }
  }
  // Put in a slight delay to help debounce the reading
  delay(1);

  oldBtnState = btnState;
}

void runSteps(AccelStepper* stepper, float speed, int steps) {
  float oldSpeed = stepper->speed();
  stepper->setSpeed(speed);
  while(steps--) {
    while(!stepper->runSpeed());
    delay(1);
  }
  stepper->setSpeed(oldSpeed);
}

// Calibrating upon restart
void calibration() {
  openLid();
  while (true)  {
    runSteps(&STEPPER_Z, -DESIRED_SPEED_Z, 1);
    if (digitalRead(END_STOP) == 0) {
      runSteps(&STEPPER_Z, DESIRED_SPEED_Z, 100);
      break;
    }
  }
  delay(1000);
  STEPPER_Z.setCurrentPosition(POSITION_HOME);
  STEPPER_Z.setSpeed(DESIRED_SPEED_Z);
  menu = 1;
  updateMenu();
}

// 95/5
void openLid() {
  while (servoLeftPos < 100 || servoRightPos > 0)  {
    if (servoLeftPos < 100)  {
      SERVO_LEFT.write(servoLeftPos);
      delay(15);
      servoLeftPos += 1;
    }
    if (servoRightPos > 0)  {
      // in steps of 1 degree
      SERVO_RIGHT.write(servoRightPos);
      delay(15);
      servoRightPos -= 1;
    }
  }
}

// 17/80
void closeLid() {
  while (servoLeftPos > 17 || servoRightPos < 80) {
    if (servoLeftPos > 17)  {
      SERVO_LEFT.write(servoLeftPos);
      delay(15);
      servoLeftPos -= 1;
    }
    if (servoRightPos < 80) {
      SERVO_RIGHT.write(servoRightPos);
      delay(15);
      servoRightPos += 1;
    }
  }
}

void washRotation() {
  runSteps(&STEPPER_STIR, DESIRED_SPEED_STIR, 1);
}

void drying() {
  digitalWrite(VALVE, HIGH);
  delay(1000);
  runSteps(&STEPPER_ROT, DESIRED_SPEED_ROT, 400);
  delay(1000);
  digitalWrite(VALVE, LOW);
}

void tempController() {
  tempSensorVal = analogRead(TEMP_PIN);
  actualTemp = (int)((((tempSensorVal / 1024.0) * 5.0) - 0.5) * 100 + 0.5);
  if (actualTemp >= cureTemp) {
    digitalWrite(HEAT, LOW);
  }
  else {
    digitalWrite(HEAT, HIGH);
  }
}

void setup() {
  // Stepper initialization
  STEPPER_Z.setMaxSpeed(1000);
  STEPPER_Z.setAcceleration(100);
  STEPPER_STIR.setMaxSpeed(1000);
  STEPPER_STIR.setAcceleration(100);
  STEPPER_ROT.setMaxSpeed(1000);
  STEPPER_ROT.setAcceleration(100);

  // Servo initialization
  SERVO_LEFT.attach(SERVO_LEFT_SIG) ;
  SERVO_RIGHT.attach(SERVO_RIGHT_SIG);
  SERVO_LEFT.write(servoLeftPos);
  SERVO_RIGHT.write(servoRightPos);

  // End stop initialization
  pinMode(END_STOP, INPUT);

  // Fan initialization
  pinMode(FAN, OUTPUT);
  digitalWrite(FAN, LOW);

  // Valve initialization
  pinMode(VALVE, OUTPUT);
  digitalWrite(VALVE, LOW);

  // Heating element initialization
  pinMode(HEAT, OUTPUT);
  digitalWrite(HEAT, LOW);
  
  // UV initialization
  pinMode(UV, OUTPUT);
  digitalWrite(UV, LOW);
  
  // LCD initialization
  lcd.begin(16, 2);
  updateMenu();

  // Rotary encoder pins initialization
  pinMode(CLK,INPUT);
  pinMode(DT,INPUT);
  pinMode(SW, INPUT_PULLUP);

  // Read the initial state of CLK
  lastStateCLK = digitalRead(CLK);

  // Running calibration program
  calibration();
}

void loop() {
  menuSelection();
}
