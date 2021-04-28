//time
long previousMillis = 0;        // will store last time
unsigned long currentMillis=0; //will store current time
long interval = 1000;           // interval/delay at which to operate

//ENGINE
 const int ena = 3; //left motor - set speed
 const int in1 = 6;
 const int in2 = 13;
 const int in3 = 2; 
 const int in4 = 4;
 const int enb = 5; //right motor - set speed


//controll the engine
int leftMotorSpeed=0;
int rightMotorSpeed=0;

//Initial Speed of Motor
int initial_motor_speed = 100;

//IR SENSOR
const int sensorNum=6;

//const int sensorPIN[sensorNum] {12,11,10,7,8,9};
const byte sensorPIN[sensorNum] {7,12,11,10,9,8};
//int linePosition;

long linePositionNum=5;//random starting number so i can set 0 as a first digit, 5 will be removed
//COLOR & DISTANCE SENSOR


//PID CONTROLLER
//PID VARIABLES
int PID_Val=0;
int error =0;
int P=0;
int I=0;
int D=0;
int prevError=0;
int prevI=0;
//PID CONSTANT
const int Kp=5;
const int Kd=2;
const int Ki=0;
const int M0=0;


//Iterate over the IR sensors, and create a 6digit num (sensor)*10+new sensor - 000000 001100 010000
void getLinePositionNum(){
  linePositionNum=5;
  for(int i=0; i < sensorNum;++i)
      linePositionNum= (linePositionNum*10)+digitalRead(sensorPIN[i]);
}

void getError()
{
  switch(linePositionNum)
  {
    case 5111110:
    {
      error = -16;
      break;
    }
    case 5111100:
    {
      error = -14;
      break;
    }
    case 5111000:
    {
      error = -12;
      break;
    }
    case 5100000:
    {
      error = -10;
      break;
    }
    case 5110000:
    {
      error = -8;
      break;
    }
    case 5010000:
    {
      error = -6;
      break;
    }
    case 5011000:
    {
      error = -4;
      break;
    }
    case 5001000:
    {
      error = -2;
      break;
    }
    case 5001100:
    {
      error = 0;
      break;
    }
    case 5000100:
    {
      error = 2;
      break;
    }
    case 5000110:
    {
      error = 4;
      break;
    }
    case 5000010:
    {
      error = 6;
      break;
    }
    case 5000011:
    {
      error = 8;
      break;
    }
    case 5000001: //turn right
    {
      error = 10;
      break;
    }

    case 5000111: //turn right
    {
      error = 12;
      break;
    }
    case 5001111: //turn right
    {
      error = 14;
      break;
    }
    case 5011111: //turn right
    {
      error = 16;
      break;
    }

   case 5000000: //continue
     {
       error = 999;
       break;
     }
    case 5111111: //continue
    {
      error = 777;
      break;
    }
    
    default: 
    {
       Serial.println("Unkown Error - Line Following Status"); 
       break; 
    }
  }
}



void calculate_pid()
{
  if(error!=777 && error != 999){
  P = error;
  I = I + prevI;
  D = error - prevError;

  PID_Val = (Kp * P) + (Ki * I) + (Kd * D);

  //prevI = I;
  prevError = error;
  }
}


void motor_control()
{
  //change nothing
  if(error==777 || error == 999)//all black //all white
    return;
if(error<10){
 // Calculating the effective motor speed:
 leftMotorSpeed = initial_motor_speed + PID_Val;
 rightMotorSpeed = initial_motor_speed;
} else if(error>-10)
{
   // Calculating the effective motor speed:
 leftMotorSpeed = initial_motor_speed + PID_Val;
 rightMotorSpeed = initial_motor_speed - PID_Val;
}
  // The motor speed should not exceed the max PWM value
  leftMotorSpeed = constrain(leftMotorSpeed+25, 0, 255);
  rightMotorSpeed = constrain(rightMotorSpeed, 0, 255);

  analogWrite(enb, leftMotorSpeed); //Left Motor Speed
  analogWrite(ena, rightMotorSpeed); //Right Motor Speed



  analogWrite(enb, leftMotorSpeed); //Left Motor Speed
  analogWrite(ena, initial_motor_speed); //Right Motor Speed
  //following lines of code are to make the bot move forward
  turn('F');
}




void printIRDigital()
{
   Serial.print("Digital Reading: ");
   Serial.print(linePositionNum);
   Serial.print("\t");
}

void printErrorVal()
{
     Serial.print("Error Reading: ");
    Serial.println(error);
}

void printMotors()
{
  Serial.print("PID Value:");
    Serial.print(PID_Val);
    Serial.print("\t");
    Serial.print("Left Value:");
    Serial.print(leftMotorSpeed);
    Serial.print("\t");
    Serial.print("Right Value:");
    Serial.print(rightMotorSpeed);
    Serial.print("\t");
}

void printDEBUG()
{
  printMotors();
  printIRDigital();
  printErrorVal();
}

void turn(char dir){
switch(dir)
{
  case 'F'://forward
  {
  //left engine
  digitalWrite(in1, HIGH);//forward
  digitalWrite(in2, LOW);//backwards
  //right engine
  digitalWrite(in3, HIGH);//forward
  digitalWrite(in4, LOW);//backwards
    break;
  }
    case 'B'://backwards
  {
  //left engine
  digitalWrite(in1, LOW);//forward
  digitalWrite(in2, HIGH);//backwards
  //right engine
  digitalWrite(in3, LOW);//forwards
  digitalWrite(in4, HIGH);//backwards
    break;
  }
    case 'L'://left
  {
  //right engine
  digitalWrite(in1, HIGH);//forward
  digitalWrite(in2, LOW);//backwards
  //left engine
  digitalWrite(in3, LOW);//forwards
  digitalWrite(in4, HIGH);//backwards
    break;
  }
    case 'R'://right
  {
  //right engine
  digitalWrite(in1, LOW);//forward
  digitalWrite(in2,HIGH);//backwards
  //left engine
  digitalWrite(in3, HIGH);//forwards
  digitalWrite(in4, LOW);//backwards
    break;
  }
    case 'S'://stop
  {
  //left engine
  digitalWrite(in1, LOW);//forward
  digitalWrite(in2, LOW);//backwards
  //right engine
  digitalWrite(in3, LOW);//forwards
  digitalWrite(in4, LOW);//backwards
    break;
  }
  //     case '<'://sharp left
  // {
  // //left engine
  // digitalWrite(in1, HIGH);//forward
  // digitalWrite(in2, LOW);//backwards
  // //right engine
  // digitalWrite(in3, LOW);//forwards
  // digitalWrite(in4, HIGH);//backwards
  //   break;
  // }
  //     case '>'://sharp right
  // {
  // //left engine
  // digitalWrite(in1, LOW);//forward
  // digitalWrite(in2, HIGH);//backwards
  // //right engine
  // digitalWrite(in3, HIGH);//forwards
  // digitalWrite(in4, LOW);//backwards
  //   break;
  // }

}

}

void sharpTurn() //TO DOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO
{
  if(error>=10) //SHARP RIGHT TURN
  { 
    analogWrite(enb, initial_motor_speed+25+50); //Left Motor Speed
    analogWrite(ena, initial_motor_speed); //Right Motor Speed 
    turn('R');
    getLinePositionNum(); //get the line position from the IR sensors XXX-XXX
    getError(); //get the amount of sway off track
    while(error!=0)
    {
          getLinePositionNum(); //get the line position from the IR sensors XXX-XXX
          getError(); //get the amount of sway off track
    }
  }else if(error <=-10) //SHARP LEFT TURN
  {
    analogWrite(enb, initial_motor_speed+25); //Left Motor Speed
    analogWrite(ena, initial_motor_speed+50); //Right Motor Speed
    turn('L');
    getLinePositionNum(); //get the line position from the IR sensors XXX-XXX
    getError(); //get the amount of sway off track
    while(error!=0)
    {
        getLinePositionNum(); //get the line position from the IR sensors XXX-XXX
        getError(); //get the amount of sway off track
    }
  }
}

void colorSensor(){
  currentMillis=millis();
  while(millis()<currentMillis+interval)//function that replaces delay
  {
    //DO SOMETHING
  }
}

void setup(){
  //Serial.begin(9600); //debug sensors

  // engines
  pinMode(ena, OUTPUT);
  pinMode(enb, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
}


void loop()
{
 
getLinePositionNum(); //get the line position from the IR sensors XXX-XXX
getError(); //get the amount of sway off track

//delay(1000);//debug
sharpTurn();  //check if need to do sharp turns
calculate_pid();//check PID
motor_control();//set speed

//printDEBUG(); //debug
 delay(5);
}
