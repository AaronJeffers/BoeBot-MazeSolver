#include <Arduino.h>
#include <Servo.h>
#include <EEPROM.h>

bool myQTIs[4];
bool myLastQTIs[4];
bool QTIsChanged;
bool middleQTIs;
bool finished;
byte turn;
byte mode;
int addr;
int startButton = 4;
int selectButton = 5;
Servo servoLeft;
Servo servoRight;

/*    Pin setup
 *    0-3   LED
 *    4-5   Buttons
 *    6     Speaker
 *    7-10  QTI Sensors
 *    11-12 Servos
 */

void setup()
{
  // Signal start of program
  tone(3, 300, 1000);

  Serial.begin(9600);

  mode = 0;
  addr = 0;
  finished = false;

  pinMode(0, OUTPUT);
  pinMode(1, OUTPUT);
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);

  pinMode(startButton, INPUT);
  pinMode(selectButton, INPUT);

  pinMode(6, OUTPUT);

  servoLeft.attach(12);
  servoRight.attach(11);

  //pinMode(13, OUTPUT);
}

void loop()
{
   waitForInput();
}

void waitForInput()
{
  // Switch to next mode on button release
  if (digitalRead(selectButton))
  {
    while (digitalRead(selectButton))
    {
      delay(10);
    }

    mode += 64;

    pinMode(0, LOW);
    pinMode(1, LOW);
    pinMode(2, LOW);
    pinMode(3, LOW);

    // Light correct LED
    if (mode == 0)
    {
      pinMode(0, HIGH);
    }
    else if (mode == 64)
    {
      pinMode(1, HIGH);
    }
    else if (mode == 128)
    {
      pinMode(2, HIGH);
    }
    else if (mode == 192)
    {
      pinMode(3, HIGH);
    }
  }

  // Start current mode on button release
  if (digitalRead(startButton))
  {
    while (digitalRead(startButton))
    {
      delay(10);
    }

    if (mode == 0)
    {
      findFinishLine(); // Right hand meathod
    }
    else if (mode == 64)
    {
      displaySolution();
    }
      else if (mode == 128)
    {
      solveMaze();
    }
      else if (mode == 192)
    {
      findFinishLine(); // Left hand meathod
    }
  }
}

void updateQTIs()
{
  // Process for reading all QTIs. Store results in myQTIs[]
  for (int x = 0; x < 4; x++)
  {
    myLastQTIs[x] = myQTIs[x];
    int sensorIn = x + 7;

    pinMode(sensorIn, OUTPUT);     // Make pin OUTPUT
    digitalWrite(sensorIn, HIGH);  // Pin HIGH
    delayMicroseconds(250);

    pinMode(sensorIn, INPUT);      // Make pin INPUT
    digitalWrite(sensorIn, LOW);   // Turn off internal pullups
    delayMicroseconds(250);

    myQTIs[x] = digitalRead(sensorIn);
    Serial.println(myQTIs[x]);
  }

  if (myQTIs[1] == 1 || myQTIs[2] == 1)
  {
    middleQTIs = 1;
  }
  else
  {
    middleQTIs = 0;
  }

  for (int x = 0; x < 4; x++)
  {
    if (myLastQTIs[x] == myQTIs[x])
    {
      QTIsChanged = 0;
    }
    else
    {
      QTIsChanged = 1;
      return;
    }
  }
}

// Go through the maze and find the "Finish Line" or end of maze.
void findFinishLine()
{
  while (!finished)
  {
    moveAlongLine();
    check();
  }
}

// Go through the maze using the result stored in EEPROM.
void solveMaze()
{
  addr = 0;
  finished = false;

  while (!finished)
  {
    moveAlongLine();
    moveForwardOneInch();

    bool b = EEPROM.read(addr);
    if (b == 64)
    {
      pivotLeft();
    }
    else if (b == 128)
    {

    }
    else if (b == 192)
    {
      pivotRight();
    }
    else if (b == 7)
    {
      finished = true;
    }

    addr +=1;
  }
}

// Move along the line until an intersection is reached.
void moveAlongLine()
{
  servoLeft.writeMicroseconds(1700);
  servoRight.writeMicroseconds(1300);

  do
  {
    updateQTIs();
    if (QTIsChanged)
    {
      if (myQTIs[1] == 1 && myQTIs[2] == 1)
      {
        servoLeft.writeMicroseconds(1700);
        servoRight.writeMicroseconds(1300);
      }
      else if (myQTIs[1] == 1)
      {
        servoLeft.writeMicroseconds(1700);
        servoRight.writeMicroseconds(1450);
      }
      else if (myQTIs[2] == 1)
      {
        servoLeft.writeMicroseconds(1550);
        servoRight.writeMicroseconds(1300);
      }
      else if (!myQTIs[0] && !myQTIs[1] && !myQTIs[2] && !myQTIs[3])
      {
        pivotRight();
        servoLeft.writeMicroseconds(1700);
        servoRight.writeMicroseconds(1300);
      }
    }
  } while (!atIntersection());

  delay(20);
  stopServos();
}

// Check if the sensors detect an intersection.
bool atIntersection()
{
  if (myQTIs[0] == 1 || myQTIs[3] == 1)
  {
    return true;
  }
  else
  {
    return false;
  }
}

// Move forward one inch. This is used at an intersection to see if continuing straight is an option
// and to line up the robots pivot point to the center of the intersection.
void moveForwardOneInch()
{
  servoLeft.writeMicroseconds(1600);
  servoRight.writeMicroseconds(1300);
  delay(350);
  stopServos();

  updateQTIs();
}

// Check what kind of intersection is detected.
void check()
{
  updateQTIs();
  moveForwardOneInch();

  if (myLastQTIs[0])
  {
    turn = 64;
    pivotLeft();
  }
  else if (middleQTIs)
  {
    turn = 128;
  }
  else if (myLastQTIs[3])
  {
    turn = 192;
    pivotRight();
  }
  else if (!myQTIs[0] && !middleQTIs && !myQTIs[3])
  {
    turn = 0;
    pivotRight();
  }
  else if (myQTIs[0] && middleQTIs && myQTIs[3])
  {
    turn = 7;
    finished = true;
  }

  EEPROM.write(addr, turn);
  correctPath();
  addr += 1;
}

// Weed out any U-turns in the maze solution.
void correctPath()
{
  if (EEPROM.read(addr - 1) == 0 && addr >= 2)
  {
    byte b = EEPROM.read(addr) + EEPROM.read(addr - 2);
    addr -= 2;
    EEPROM.write(addr, b);
  }
}

void pivotLeft()
{
  servoLeft.writeMicroseconds(1300);
  servoRight.writeMicroseconds(1300);
  delay(500);
  updateQTIs();

  while (myQTIs[1] == 0 && myQTIs[2] == 0)
  {
    updateQTIs();
  }

  servoLeft.writeMicroseconds(1450);
  servoRight.writeMicroseconds(1450);
  updateQTIs();

  while (myQTIs[1] == 0 || myQTIs[2] == 0)
  {
    updateQTIs();
  }
  stopServos();
}

void pivotRight()
{
  // Start turning right.
  servoLeft.writeMicroseconds(1700);
  servoRight.writeMicroseconds(1700);

  // Wait half a second.
  delay(500);
  updateQTIs();

  // Continue turning until the two middle QTIs detect the line.
  while (myQTIs[1] == 0 && myQTIs[2] == 0)
  {
    updateQTIs();
  }

  // Slow down to prevent overshooting the line.
  servoLeft.writeMicroseconds(1550);
  servoRight.writeMicroseconds(1550);
  updateQTIs();

  // Continue until parrallel with the line.
  while (myQTIs[1] == 0 || myQTIs[2] == 0)
  {
    updateQTIs();
  }
  stopServos();
 }

void stopServos()
{
  servoLeft.writeMicroseconds(1500);
  servoRight.writeMicroseconds(1500);
}

// Light up the LEDs to display the current maze solution stored in EEPROM.
void displaySolution()
{
  pinMode(0, LOW);
  pinMode(1, LOW);
  pinMode(2, LOW);
  pinMode(3, LOW);

  finished = false;
  addr = 0;

  while(!finished)
  {
    byte b = EEPROM.read(addr);

    if (b == 0)
    {

    }
    else if (b == 64)
    {
      pinMode(3, HIGH);
    }
    else if (b == 128)
    {
      pinMode(0, HIGH);
      pinMode(3, HIGH);
    }
    else if (b == 192)
    {
      pinMode(0, HIGH);
    }
    else if (b == 7)
    {
      pinMode(0, HIGH);
      pinMode(1, HIGH);
      pinMode(2, HIGH);
      pinMode(3, HIGH);
      finished = true;
    }

    addr += 1;
    delay(100);
  }

  addr = 0;
}


