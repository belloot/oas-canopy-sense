
#include <stdlib.h>

class Nema17 {
  public: 
    int stepPin = 3;
    int dirPin  = 2;

    int STEP_DELAY_US  = 1000;

    // Software position tracking
    long curPos = 0;

    // Allowed travel range
    long MIN_POS = 50;   // 50 steps above bottom
    long MAX_POS = 1750;  

    Nema17() = default;
    Nema17(int stepPin, int dirPin, int STEP_DELAY_US, long MIN_POS, long MAX_POS): stepPin(stepPin), dirPin(dirPin), STEP_DELAY_US(STEP_DELAY_US), MIN_POS(MIN_POS), MAX_POS(MAX_POS){}

    void Nema17_init(){
      pinMode(this->stepPin, OUTPUT);
      pinMode(this->dirPin, OUTPUT);
    }

    void Nema17_resetBottomEndpoint(){
      this->curPos=0;
    }

    int Nema17_tryMove(int stepsToMove){
      if (stepsToMove == 0) return 0;
      if(stepsToMove>0){//move up
        long remaining=this->MAX_POS-this->curPos;
        if(remaining<=0)return 0;
        return min((long)stepsToMove, remaining);
      }else{//move down
        long remaining=this->curPos-this->MIN_POS;
        if(remaining<=0)return 0;
        return -min((long)abs(stepsToMove), remaining);
      }
    }
    
    void Nema17_moveSteps(int stepsToMove)
    {
      int ret=Nema17_tryMove(stepsToMove);
      if(ret==0)Serial.println("Limit reached");
      else if(ret>0)Serial.println("Moved Up");
      else if(ret<0)Serial.println("Moved Down");
      bool direction = ret > 0 ? HIGH : LOW;
      if(ret!=0){
        digitalWrite(this->dirPin, direction);
        for (int i = 0; i < abs(ret); i++) {
          digitalWrite(this->stepPin, HIGH);
          delayMicroseconds(this->STEP_DELAY_US);

          digitalWrite(this->stepPin, LOW);
          delayMicroseconds(this->STEP_DELAY_US);
        }
      }
      this->curPos+=ret;
      Serial.print("Steps Moved: ");
      Serial.println(ret);

      Serial.print("Current Pos: ");
      Serial.println(this->curPos);
      return;
    }

};

Nema17 nema = Nema17(3,2,1000,50,1750);

void setup() {
  nema.Nema17_init();

  Serial.begin(115200);

  Serial.println("Commands:");
  Serial.println("e = set bottom position");
  Serial.println("w = move up");
  Serial.println("s = move down");
}

const int steptomove=150;

void loop() {
  if (Serial.available()) {

    char cmd = Serial.read();

    // Set current position as the bottom reference
    if (cmd == 'e') {
      nema.Nema17_resetBottomEndpoint();
      Serial.println("Bottom position set.");
      Serial.print("Position = ");
      Serial.println(nema.curPos);
    }

    // Move up
    else if (cmd == 'w') {
      nema.Nema17_moveSteps(steptomove);
    }

    // Move down
    else if (cmd == 's') {
      nema.Nema17_moveSteps(-steptomove);
    }
  }
}