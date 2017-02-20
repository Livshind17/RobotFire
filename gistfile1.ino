#define bluetooth Serial3
enum Directions {
    Left = 2,Right = 1,Forward = 0,Back = 3,Angle = 4};
typedef struct MotorPin { 
    int forward; 
    int back;
    int pwm;
} MotorPin;
typedef struct UltrasonicPin { 
    int echo;
    int trig;
    int lastDistance;
} UltrasonicPin;
typedef struct DirectionrRobot { 
    int _direction[4][10];
    int  _angle=0;
} DirectionrRobot;
const MotorPin MotorsPins[4]={{22,23,4},{24,25,5},{26,27,6},{28,29,9}};
const UltrasonicPin UsPins[4]={{52,53,0},{46,47,0},{50,51,0},{48,49,0}};
int DIRECTION_0[]={LOW,LOW,LOW,LOW,HIGH,LOW,HIGH,LOW,UsPins[Forward].trig,UsPins[Forward].echo};
int DIRECTION_90[]={LOW,HIGH,HIGH,LOW,LOW,LOW,LOW,LOW,UsPins[Right].trig,UsPins[Right].echo};
int DIRECTION_270[]={HIGH,LOW,LOW,HIGH,LOW,LOW,LOW,LOW,UsPins[Left].trig,UsPins[Left].echo};
int DIRECTION_180[]={LOW,LOW,LOW,LOW,LOW,HIGH,LOW,HIGH,UsPins[Back].trig,UsPins[Back].echo};
DirectionrRobot CurrDirection;
void setup () {
  memcpy(CurrDirection._direction[Forward], DIRECTION_0, 10 * sizeof(int));
  memcpy(CurrDirection._direction[Back], DIRECTION_180, 10 * sizeof(int));
  memcpy(CurrDirection._direction[Left], DIRECTION_270, 10 * sizeof(int));
  memcpy(CurrDirection._direction[Right], DIRECTION_90, 10 * sizeof(int));
  CurrDirection._angle=0;
  for(int i=0;i<4;i++){
  pinMode(MotorsPins[i].forward, OUTPUT);
  pinMode(MotorsPins[i].back, OUTPUT);
  pinMode(MotorsPins[i].pwm, OUTPUT);
  pinMode(UsPins[i].trig, OUTPUT);
  pinMode(UsPins[i].echo, INPUT);
  }
  Serial.begin (9600);
  bluetooth.begin(115200);
  bluetooth.print("$$$");
  bluetooth.println("U,9600,N");
  bluetooth.begin(9600);

}
void loop () {
_move(Forward,100,200);

}

/*void fix()
{
   
  if( usRight() < 10)
  {
     turn_step(STEP_S,'l');
  }
  if(usLeft() <10)
  {
    turn_step(STEP_S,'r');
  }
  return;
}*/
void left(){
  rotate();       
  rotate();
  rotate();
}
void right(){
  rotate();
}


int us(int Direction){
 // lastDistanceB = thisDistanceB;
  long duration = 0;
  digitalWrite(CurrDirection._direction[Direction][8], LOW);
  delayMicroseconds(2);
  digitalWrite(CurrDirection._direction[Direction][8], HIGH);
  delayMicroseconds(10);
  digitalWrite(CurrDirection._direction[Direction][8], LOW);
  duration = pulseIn(CurrDirection._direction[Direction][9], HIGH);
  duration = duration / 58;
//  thisDistanceB=duration;
  return duration;
}

void _move(int Direction,int steps,int _speed){

  for(int i =0;i<4;i++)
{
  digitalWrite(MotorsPins[i].forward,HIGH);
  digitalWrite(MotorsPins[i].back, LOW);
  analogWrite(MotorsPins[i].pwm, _speed);
}
}
void rotate(){
    CurrDirection._angle=CurrDirection._angle%360;  
    if(CurrDirection._angle==0){
      memcpy(CurrDirection._direction[Forward], DIRECTION_90, 10 * sizeof(int));
      memcpy(CurrDirection._direction[Right], DIRECTION_180, 10 * sizeof(int));
      memcpy(CurrDirection._direction[Left], DIRECTION_0, 10 * sizeof(int));
      memcpy(CurrDirection._direction[Back], DIRECTION_270, 10 * sizeof(int));
      CurrDirection._angle+=90;
    }
    else if(CurrDirection._angle==90){
        memcpy(CurrDirection._direction[Forward], DIRECTION_180, 10 * sizeof(int));
        memcpy(CurrDirection._direction[Right], DIRECTION_270, 10 * sizeof(int));
        memcpy(CurrDirection._direction[Left], DIRECTION_90, 10 * sizeof(int));
        memcpy(CurrDirection._direction[Back], DIRECTION_0, 10 * sizeof(int));
        CurrDirection._angle+=90;
    }
    else if(CurrDirection._angle==180){
        memcpy(CurrDirection._direction[Forward], DIRECTION_270, 10 * sizeof(int));
        memcpy(CurrDirection._direction[Right], DIRECTION_0, 10 * sizeof(int));
        memcpy(CurrDirection._direction[Left], DIRECTION_180, 10 * sizeof(int));
        memcpy(CurrDirection._direction[Back], DIRECTION_90, 10 * sizeof(int));
        CurrDirection._angle+=90;
    }
    else if(CurrDirection._angle==270)
    {
        memcpy(CurrDirection._direction[Forward], DIRECTION_0, 10 * sizeof(int));
        memcpy(CurrDirection._direction[Right], DIRECTION_90, 10 * sizeof(int));
        memcpy(CurrDirection._direction[Left], DIRECTION_270, 10 * sizeof(int));
        memcpy(CurrDirection._direction[Back], DIRECTION_180, 10 * sizeof(int));
        CurrDirection._angle+=90;
    }
}


