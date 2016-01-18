#include <Servo.h>

bool myQTIs[4];
bool myLastQTIs[4];
bool QTIsChanged;
bool middleQTIs;
byte turn;
Servo servoLeft;
Servo servoRight;

void setup()
{
  tone(3, 300, 1000);

  Serial.begin(9600);

  servoLeft.attach(12);
  servoRight.attach(11);

  pinMode(13, OUTPUT);
}

void loop()
{
  moveAlongLine();
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

  check();
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
    pivotLeft();
    moveAlongLine();
  }
  else if (middleQTIs)
  {
    moveAlongLine();
  }
  else if (myLastQTIs[3])
  {
    pivotRight();
    moveAlongLine();
  }
  else if (!myQTIs[0] && !middleQTIs && !myQTIs[3])
  {
    pivotRight();
    moveAlongLine();
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

  // Continue turning until the two middle QTIs start to detect the line.
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


