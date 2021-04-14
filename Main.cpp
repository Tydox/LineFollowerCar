//ENGINE
const int ena = 10; //left motor - set speed
const int in1 = 7;
const int in2 = 6;
const int in3 = 4; 
const int in4 = 5;
const int enb = 9; //right motor - set speed

//controll the engine
int leftMotorSpeed=0;
int rightMotorSpeed=0;

//Initial Speed of Motor
int initial_motor_speed = 150;

//IR SENSOR
const int sensorNum=6;

const int sensorPIN[sensorNum] {12,11,10,7,8,9};
int linePosition;

int linePositionNum=5;//random starting number so i can set 0 as a first digit, 5 will be removed
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
const int Kp=0;
const int Kd=0;
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
    case 5100000:
    {
      error = -5;
      break;
    }
    case 5110000:
    {
      error = -4;
      break;
    }
    case 5010000:
    {
      error = -3;
      break;
    }
    case 5011000:
    {
      error = -2;
      break;
    }
    case 5001000:
    {
      error = -1;
      break;
    }
    case 5001100:
    {
      error = 0;
      break;
    }
    case 5000100:
    {
      error = 1;
      break;
    }
    case 5000110:
    {
      error = 2;
      break;
    }
    case 5000010:
    {
      error = 3;
      break;
    }
    case 5000011:
    {
      error = 4;
      break;
    }
    case 5000001: //turn right
    {
      error = 5;
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
    case 5001111: //turn right
    {
      error = 106;
      break;
    }
    case 5111100: //turn right
    {
      error = 104;
      break;
    }
    default: 
    {
       Serial.println("Something went wrong! check the code!!!!!"); 
       break; 
    }
  }
}



void calculate_pid()
{
  P = error;
  I = I + prevI;
  D = error - prevError;

  PID_Val = (Kp * P) + (Ki * I) + (Kd * D);

  prevI = I;
  prevError = error;
}


void motor_control()
{
  // Calculating the effective motor speed:
 leftMotorSpeed = initial_motor_speed - PID_Val;
 rightMotorSpeed = initial_motor_speed + PID_Val;

  // The motor speed should not exceed the max PWM value
  leftMotorSpeed = constrain(leftMotorSpeed, 0, 255);
  rightMotorSpeed = constrain(rightMotorSpeed, 0, 255);

  /*Serial.print(PID_value);
    Serial.print("\t");
    Serial.print(left_motor_speed);
    Serial.print("\t");
    Serial.println(right_motor_speed);*/

  analogWrite(ena, leftMotorSpeed); //Left Motor Speed
  analogWrite(enb, rightMotorSpeed - 30); //Right Motor Speed

  //following lines of code are to make the bot move forward
  turn('F');
}




void printIRDigital()
{
   Serial.print("Digital Reading=");
   Serial.println(linePositionNum%10);
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
  //left engine
  digitalWrite(in1, HIGH);//forward
  digitalWrite(in2, LOW);//backwards
  //right engine
  digitalWrite(in3, LOW);//forwards
  digitalWrite(in4, LOW);//backwards
    break;
  }
    case 'R'://right
  {
  //left engine
  digitalWrite(in1, LOW);//forward
  digitalWrite(in2,LOW);//backwards
  //right engine
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
      case '<'://sharp left
  {
  //left engine
  digitalWrite(in1, HIGH);//forward
  digitalWrite(in2, LOW);//backwards
  //right engine
  digitalWrite(in3, LOW);//forwards
  digitalWrite(in4, HIGH);//backwards
    break;
  }
      case '>'://sharp right
  {
  //left engine
  digitalWrite(in1, LOW);//forward
  digitalWrite(in2, HIGH);//backwards
  //right engine
  digitalWrite(in3, HIGH);//forwards
  digitalWrite(in4, LOW);//backwards
    break;
  }

}

}

void sharpTurn() //TO DOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO
{
  switch (error)
  {



  }
}

void setup(){
  //sensors
  Serial.begin(9600);
  
  //engines
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  analogWrite(ena, 200);
  analogWrite(enb, 200);

}


void loop()
{
getLinePositionNum(); //get the line position from the IR sensors XXX-XXX
getError(); //get the amount of sway off track

//debug
Serial.println(error);

//check if need to do sharp turns
sharpTurn();

//check PID and set Speed
calculate_pid();
motor_control();
}
