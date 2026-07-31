// Defines pins numbers
const int stepPin = 3;
const int dirPin  = 2;

const int MOVE_STEPS     = 150;
const int STEP_DELAY_US  = 1000;

// Software position tracking
long currentPos = 0;

// Allowed travel range
const long MIN_POS = 50;   // 50 steps above bottom
const long MAX_POS = 1750;  

void setup() {
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);

  Serial.begin(115200);

  Serial.println("Commands:");
  Serial.println("e = set bottom position");
  Serial.println("w = move up");
  Serial.println("s = move down");
}

void moveSteps(int numSteps, bool direction)
{
  digitalWrite(dirPin, direction);

  for (int i = 0; i < numSteps; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(STEP_DELAY_US);

    digitalWrite(stepPin, LOW);
    delayMicroseconds(STEP_DELAY_US);
  }
}

void moveUp()
{
  long stepsRemaining = MAX_POS - currentPos;

      if (stepsRemaining > 0) {

        int stepsToMove = min((long)MOVE_STEPS, stepsRemaining);

        moveSteps(stepsToMove, HIGH);
        currentPos += stepsToMove;

        Serial.print("Moved up ");
        Serial.print(stepsToMove);
        Serial.println(" steps");
      }
      else {
        Serial.println("Upper limit reached");
      }

      Serial.print("Position = ");
      Serial.println(currentPos);
}

void moveDown()
{
  long stepsRemaining = currentPos - MIN_POS;

      if (stepsRemaining > 0) {

        int stepsToMove = min((long)MOVE_STEPS, stepsRemaining);

        moveSteps(stepsToMove, LOW);
        currentPos -= stepsToMove;

        Serial.print("Moved down ");
        Serial.print(stepsToMove);
        Serial.println(" steps");
      }
      else {
        Serial.println("Lower limit reached");
      }

      Serial.print("Position = ");
      Serial.println(currentPos);
}

void loop() {
  if (Serial.available()) {

    char cmd = Serial.read();

    // Set current position as the bottom reference
    if (cmd == 'e') {
      currentPos = 0;

      Serial.println("Bottom position set.");
      Serial.print("Position = ");
      Serial.println(currentPos);
    }

    // Move up
    else if (cmd == 'w') {
      moveUp();
    }

    // Move down
    else if (cmd == 's') {
      moveDown();
    }
  }
}