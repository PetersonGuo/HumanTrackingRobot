/*
 * Object Tracking Robot
 * Move in front of the robot to get the robot to calibrate to you
 * The robot will then proceed to follow you is a 1 - 64 cm range
 * **Robot is unable to distinguish between a objects and a human, do not go too close to obstacles
 * To use the LCD to check sensor data, press the button and wait 1 second to view the data that follows the screen
 * 
 * Motor Shield Pins
 * Function        Channel A   Channel B
 * Direction       Digital 12  Digital 13
 * Speed (PWM)     Digital 3   Digital 11
 * Brake           Digital 9   Digital 8
 * Current Sensing Analog 0    Analog 1
*/
/* Left, Right Heading are the robot's left and right
 * MotorA is the Right Motor, MotorB is the left motor
 * To move forward, right motor should have a dir of HIGH and left has a dir of LOW
 * 
 * Right Ultrasonic functionality was removed and pathfinding was removed
 * Calibrate ultrasonic and ir sensors in beginning of program (find best distance for bot IR sensors to work) 
*/
#include <NewPing.h>
#include <LiquidCrystal.h>
#include <ArduinoSTL.h>
#include <list>

#define MAX 255 // High Value
#define FORWARD 0 // Define forward and backward to use in program
#define BACKWARD 1

// Ultrasonic pins
#define TrigPin 30    // Middle Trig Pin
#define EchoPin 28    // Middle Echo Pin
#define minMS 29      // Minimum amount of time to wait between pings
#define MAXDist 200   // Max Distance for robot to consider
const int pings = 5;  // Number of pings to send
float CM = 0;         // Distance from middle of robot to object
NewPing sonar(TrigPin, EchoPin, MAXDist);  // 200 is the max distance to look for a ping

// Motor 4.5 Rotations/s at max speed
#define maxSpeed 125        // Max speed to allow ultrasonic sensor time to ping
#define minSpeed 60         // Min speed for motors to turn
#define dirA 12             // Direction A 
#define dirB 13             // Direction B
#define bA 9                // Brake A
#define bB 8                // Brake B
#define mA 3                // Motor Speed A
#define mB 11               // Motor Speed B
#define r 3                 // Wheel Radius
const int diff  = 4;        // Difference in speed between motors to account for a slower motor - for left faster, + for right faster
const int dir[] = {12, 13}; // Motor DIR pins (digital)
const int pwm[] = {3, 11};  // Motor PWM pins (analog)
const int brake[] = {8, 9}; // Motor Brake pins (digital)
int leftSpeed = 0;          // Left Motor speed (0 - 255)
int rightSpeed = 0;         // Right Motor speed (0 - 255)

// IR pins
#define LIR 50    // Left IR pin
#define RIR 52    // Right IR pin
int IRDist = 26;  // Max distance for IR sensors to ping
int leftIR = 0;   // Left Sensor value
int rightIR = 0;  // Right Sensor value

// Button
#define button 32     // Button pin
int lastState = HIGH; // Allow only one button press

// Speaker
#define speaker 2 // Speaker Pin

// LCD pins
#define rs 10
#define e 22
#define d4 4
#define d5 5
#define d6 6
#define d7 7
#define hpixel 14 // LCD horizontal pixels
#define vpixel 2  // LCD Vertical pixels
#define displayTime 2000 // Amount of time to change what sensor to display in milliseconds
LiquidCrystal lcd(rs, e, d4, d5, d6, d7);
int lcdMode = 1;

long lastTime = 0;
int counter = 0;
std::list<int> calibrate;
/*
 * lcdMode Chooses a statistic screen to display (button cycles screen)
 * 0: No Display
 * 1: Motors
 * 2: Ultrasonic
 * 3: IR
 */

 /*To Do:
    Add searching algo
    robot has waited too long since stopping find another object: Need pathfinding algorithm first
    Add a move backwards and turn towards IR sensor if IR sensor detects presence but not ultrasonic
    Remove any outliers in the averaging function*/
    
void setup() {  
  // Pin Setup
  pinMode(LIR, INPUT);
  pinMode(RIR, INPUT);
  pinMode(button, INPUT_PULLUP);
  pinMode(speaker, OUTPUT);
  analogWrite(speaker, 65); // Buzz
  for (int i = 0; i < 2; i++) {
    digitalWrite(speaker, HIGH);
    delay(200);
    digitalWrite(speaker, LOW);
    delay(200);
  }

  calibrate.push_back(7);
  calibrate.push_back(5);
  IRDist = getMaxAverage();
  
  lcd.begin(hpixel, vpixel);  // Begin LCD
  lcd.noBlink();
  
  Serial.begin(9600);
  
  // Set pins of the motor
  for (int i = 0; i < 2; i++) {
    pinMode(dir[i], OUTPUT);
    pinMode(brake[i], OUTPUT);
    pinMode(pwm[i], OUTPUT);
  }
  checkLCD();
  delay(2000); // Wait 2s before beginning program
}

void loop() {
  timer();
  checkButton();
  IRDist = getMaxAverage();
  // Read IR sensor
  leftIR = digitalRead(LIR);
  rightIR = digitalRead(RIR);
  Serial.println(leftIR+rightIR);
  
  // Ping Ultrasonic and get average of 5 pings
  int arr[pings];
  for (int i = 0; i < pings; i++) {
    arr[i] = sonar.ping_cm();
    delay(minMS);
  }
  CM = avgArr(arr, &pings);
  delay(minMS);
  Serial.println(CM);
  if (CM <= 5 && CM >= 1) {   // If object is too close
    moveRobot(minSpeed, BACKWARD, 4);
  } else if (leftIR == 0 && rightIR == 0) {
    if (CM != 0) 
      calibrate.push_back(CM);
    stopRobot(true, true);
  } else if (leftIR == 0 && rightIR == 1) {         // Object is detected on left IR
    moveRobot(minSpeed, maxSpeed, FORWARD, true);
  } else if (rightIR == 0 && leftIR == 1) {         // Object is detected on right IR
    moveRobot(maxSpeed, minSpeed, FORWARD, true);
  } else if (CM >= 64 && CM < 80) {
    moveRobot(255, FORWARD, CM - 50);
  } else if (CM < 64 && CM >= IRDist) {
    moveRobot(maxSpeed / 2, FORWARD);
  } else if (CM < IRDist && CM > IRDist * 3 / 4) {
    moveRobot(minSpeed, FORWARD);
  } else if (CM <= IRDist * 3 / 4 && CM > 5) {
    stopRobot(true, true);
  } else if (CM == 0 && leftIR == 1 && rightIR == 1) {
    stopRobot(true, true);
    if (counter == 8000000) { // Lost track for more than 500ms :: Arduino runs loop() 16,000,000 times per second
      lcd.print("No Data");
      lcd.setCursor(0, 1);
      lcd.print("Lost Tracking");
      for (int i = 0; i < 3; i++) {  // Buzz for 3s
        digitalWrite(speaker, HIGH);
        delay(500); // 500 ms delay
        if (checkSens()) 
          break;
        delay(500); // 500 ms delay
      }
      counter = 0;
    } else
      counter++;
  }
}

int getMaxAverage() {
  int highest = 0;
  for (int i : calibrate) {
    if (i > highest)
      highest = i;
  }
  for (std::list<int>::iterator it = calibrate.begin(); it != calibrate.end(); it++) {
    if (*it < highest / 2)
      calibrate.remove(*it);
  }
  return avgList(calibrate);
}

int avgArr(int *arr, const int* length) {    // Take the average of an array
    int result = 0;
    for (int i = 0; i < *length; i++)
        result += arr[i];
  return result / *length; // Return average
}

int avgList(std::list<int> l) {
  int result = 0;
  for (int i : l)
    result += i;
  return result / l.size();
}

bool checkSens() {  // Check if sensors have a valid input
  digitalWrite(speaker, LOW);
  return (rightIR = digitalRead(LIR)) == 0 || (leftIR = digitalRead(RIR)) == 0 || ((CM = sonar.ping_cm()) < 100 && CM != 0);
}

void turnRight(int deg) {
  rightSpeed = 0;
  leftSpeed = minSpeed;
  timer();  // Update LCD
  stopRobot(false, true);   // Stop right Motor
  digitalWrite(dirB, LOW);  // Move left Motor forward
  digitalWrite(bB, LOW);    // Disengage left Brake
  analogWrite(mB, minSpeed);// Set B speed
  delay(deg * 1500 / 90.0); // Determine delay based off how long to wait to turn 90 degrees at speed 40
  stopRobot(true, true);
}

void turnLeft(int deg) {
  leftSpeed = 0;
  rightSpeed = minSpeed;
  timer();  // Update LCD
  stopRobot(true, false);   // Stop left Motor
  digitalWrite(dirA, HIGH); // Move right Motor forward
  digitalWrite(bA, LOW);    // Disengage right Brake
  analogWrite(mA, minSpeed);// Set A speed
  delay(deg * 1500 / 90.0); // Determine delay based off how long to wait to turn 90 degrees at speed 40
  stopRobot(true, true);
}

void moveRobot(int speed, int direction) {  // General movement
  leftSpeed = speed;
  rightSpeed = speed;
  timer();  // Update LCD
  if (speed > maxSpeed) // Make sure speed stays under predefined max speed
    speed = maxSpeed;
  if (direction == FORWARD) { // if DIR == FORWARD
    digitalWrite(dir[0], HIGH);
    digitalWrite(dir[1], LOW);
  }
  else if (direction == BACKWARD) {  // if DIR == BACKWARD
    digitalWrite(dir[0], LOW);
    digitalWrite(dir[1], HIGH);
  }
  
  releaseBrakes(true, true);
  
  analogWrite(mA, direction == FORWARD ? speed - (diff >> 1) : speed + (diff >> 1));
  analogWrite(mB, direction == BACKWARD ? speed - (diff >> 1) : speed + (diff >> 1));
}

/*Allow variable speed motors*/
void moveRobot(int lSpeed, int rSpeed, int direction, bool isDiff) { // Add bool so as to not be confused with the latter func
  leftSpeed = lSpeed;
  rightSpeed = rSpeed;
  timer();  // Update LCD
  if (lSpeed > maxSpeed) // Make sure speed stays under predefined max speed
    lSpeed = maxSpeed;
  if (rSpeed > maxSpeed)
    rSpeed = maxSpeed;
  if (direction == FORWARD) { // if DIR == FORWARD
    digitalWrite(dir[0], HIGH);
    digitalWrite(dir[1], LOW);
  }
  else if (direction == BACKWARD) {  // if DIR == BACKWARD
    digitalWrite(dir[0], LOW);
    digitalWrite(dir[1], HIGH);
  }
  
  releaseBrakes(true, true);

  analogWrite(pwm[0], direction == FORWARD ? rSpeed - (diff >> 1) : rSpeed + (diff >> 1));
  analogWrite(pwm[1], direction == BACKWARD ? lSpeed - (diff >> 1) : lSpeed + (diff >> 1));
}

void moveRobot(int speed, int direction, int CM) {  // Move robot based on distance
  const int maxSpeed1 = 50;
  if (speed > maxSpeed1) // Make sure speed stays under predefined max speed (lowered so CM is more accurate)
    speed = maxSpeed1;
  leftSpeed = speed;
  rightSpeed = speed;
  timer();  // Update LCD
  if (direction == FORWARD) { // if DIR == FORWARD
    digitalWrite(dir[0], HIGH);
    digitalWrite(dir[1], LOW);
  }
  else if (direction == BACKWARD) {  // if DIR == BACKWARD
    digitalWrite(dir[0], LOW);
    digitalWrite(dir[1], HIGH);
  }
  
  releaseBrakes(true, true);
  
  analogWrite(mA, direction == FORWARD ? speed - (diff >> 1) : speed + (diff >> 1));
  analogWrite(mB, direction == BACKWARD ? speed - (diff >> 1) : speed + (diff >> 1));
  delay(CM / ((speed / 255.0 * 4.5) * (2.0 * PI * r)) * 1000);  // Calculate delay by using radius, speed, rotations/s at max speed, and CM
  stopRobot(true, true);
}

void releaseBrakes(bool left, bool right) {
  if (left && right) 
    for (int i : brake)
      digitalWrite(i, LOW);
   else if (left)
    digitalWrite(bB, LOW);
   else if (right)
    digitalWrite(bA, LOW);
}

void stopRobot(bool left, bool right) {   // Stop Motors
  if (left && right) {                    // Stop left and right motors
    for (int i : brake)                   // Engage brakes
      digitalWrite(i, HIGH);

    for (int j : pwm)
      analogWrite(j, 0);                  // Set speed to 0
      
    leftSpeed = 0;
    rightSpeed = 0;
  }
  else if (left) {                        // Stop left motor
    digitalWrite(bB, HIGH);
    analogWrite(mB, 0);
    leftSpeed = 0;
  }
  else if (right) {                       // Stop right motor
    digitalWrite(bA, HIGH);
    analogWrite(mA, 0);
    rightSpeed = 0;
  }
  timer();                                // Update LCD
}

void checkButton() {                              // Check for Button Press
  int current = digitalRead(button);              // Read button State
  if (current == HIGH && current != lastState) {  // Check if button is pressed and that the state has changed
    lcdMode++;  // Cycle LCD state
    checkLCD(); // Change LCD display
  }
  lastState = current;  // Last state is the current state
}

void timer() {  // Used to determine when to change LCD screen
  if (millis() - lastTime > displayTime) { // Check that timer has finished
    lcd.clear();
    switch(lcdMode) { // Switch Case on lcdMode
      case 1: // Display Motor speed
        printLCD("LEFT:  ", leftSpeed / 255.0 * 4.5 * 60, "RPM");
        lcd.setCursor(0,1);
        printLCD("RIGHT: ", rightSpeed / 255.0 * 4.5 * 60, "RPM");
        break;
      case 2: // Display Ultrasonic distance
        if (CM == 0) {
            lcd.print("ULTRASONIC: ");
            lcd.setCursor(0, 1);
            printLCD("", "OUT OF RANGE");
        } else {
            lcd.print("ULTRASONIC: ");
            lcd.setCursor(0, 1);
            printLCD("", CM, "CM");
        }
        break;
      case 3: // Display IR Sensor
        printLCD("LEFT:  ", leftIR == 1 ? "LOW" : "HIGH");
        lcd.setCursor(0,1);
        printLCD("RIGHT: ", rightIR == 1 ? "LOW" : "HIGH");
        break;
    }
  }
}

void checkLCD() { // Check what to display on LCD
  lastTime = millis(); // Start timer
  lcd.clear();
  switch(lcdMode) { // Switch case on lcdMode
    case 4: // If out of bounds, set to 0
      lcdMode = 0;
      break;
    case 1: // Display Motors
      lcd.print("MOTORS");
      break;
    case 2: // Display Ultrasonic
      lcd.print("ULTRASONIC");
      break;
    case 3: // Display IR
      lcd.print("IR SENSORS");
      break;
  }
}

// Helper functions to print lcd
void printLCD(String str, int num, String str1) {
  lcd.print(str);
  for (int i = 0; i < 16 - (str.length() + str1.length() + String(num).length()); i++)
    lcd.print(" ");
  lcd.print(num);
  lcd.print(str1);
}

void printLCD(String str, String str1) {
  lcd.print(str);
  for (int i = 0; i < 16 - (str.length() + str1.length()); i++)
    lcd.print(" ");
  lcd.print(str1);
}
