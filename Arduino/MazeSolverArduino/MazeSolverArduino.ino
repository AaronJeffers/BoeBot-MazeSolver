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
  // Singnal start of program
  tone(3, 300, 1000);

  Serial.begin(9600);

  mode = 0;
  addr = 0;
  finished = false;

  servoLeft.attach(12);
  servoRight.attach(11);

  pinMode(startButton, INPUT);
  pinMode(selectButton, INPUT);
  pinMode(13, OUTPUT);
}

void loop()
{
   waitForInput();
}

void waitForInput()
{
  if (digitalRead(selectButton))
  {
    mode += 64;

    while (digitalRead(selectButton))
    {
      delay(10);
    }
  }
  
  if (digitalRead(startButton))
  {
    if (mode == 0)
    {
      findFinishLine();
    }
    else if (mode == 64)
    {
      
    }
      else if (mode == 128)
    {
      solveMaze();
    }
      else if (mode == 192)
    {
      
    }

    
  }
}

void updateQTIs()
{
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

void findFinishLine()
{
  while (!finished)
  {
    moveAlongLine();
    check();
  }
}

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

void moveForwardOneInch()
{
  servoLeft.writeMicroseconds(1600);
  servoRight.writeMicroseconds(1300);
  delay(350);
  stopServos();

  updateQTIs();
}

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


