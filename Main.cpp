
#include "Melopero_APDS9960.h"

Melopero_APDS9960 device;

//#define NOTE_B0  31
//#define NOTE_C1  33
//#define NOTE_CS1 35
//#define NOTE_D1  37
//#define NOTE_DS1 39
#define NOTE_E1 41
#define NOTE_F1 44
#define NOTE_FS1 46
#define NOTE_G1 49
#define NOTE_GS1 52
#define NOTE_A1 55
#define NOTE_AS1 58
#define NOTE_B1 62
#define NOTE_C2 65
#define NOTE_CS2 69
#define NOTE_D2 73
#define NOTE_DS2 78
#define NOTE_E2 82
#define NOTE_F2 87
#define NOTE_FS2 93
#define NOTE_G2 98
#define NOTE_GS2 104
#define NOTE_A2 110
#define NOTE_AS2 117
#define NOTE_B2 123
#define NOTE_C3 131
#define NOTE_CS3 139
#define NOTE_D3 147
#define NOTE_DS3 156
#define NOTE_E3 165
#define NOTE_F3 175
#define NOTE_FS3 185
#define NOTE_G3 196
#define NOTE_GS3 208
#define NOTE_A3 220
#define NOTE_AS3 233
#define NOTE_B3 247
#define NOTE_C4 262
#define NOTE_CS4 277
#define NOTE_D4 294
#define NOTE_DS4 311
#define NOTE_E4 330
#define NOTE_F4 349
#define NOTE_FS4 370
#define NOTE_G4 392
#define NOTE_GS4 415
#define NOTE_A4 440
#define NOTE_AS4 466
#define NOTE_B4 494
#define NOTE_C5 523
#define NOTE_CS5 554
#define NOTE_D5 587
#define NOTE_DS5 622
#define NOTE_E5 659
#define NOTE_F5 698
#define NOTE_FS5 740
#define NOTE_G5 784
#define NOTE_GS5 831
#define NOTE_A5 880
#define NOTE_AS5 932
#define NOTE_B5 988
#define NOTE_C6 1047
#define NOTE_CS6 1109
#define NOTE_D6 1175
#define NOTE_DS6 1245
#define NOTE_E6 1319
#define NOTE_F6 1397
//#define NOTE_FS6 1480
//#define NOTE_G6  1568
//#define NOTE_GS6 1661
//#define NOTE_A6  1760
//#define NOTE_AS6 1865
//#define NOTE_B6  1976
//#define NOTE_C7  2093
//#define NOTE_CS7 2217
//#define NOTE_D7  2349
//#define NOTE_DS7 2489
//#define NOTE_E7  2637
//#define NOTE_F7  2794
//#define NOTE_FS7 2960
//#define NOTE_G7  3136
//#define NOTE_GS7 3322
//#define NOTE_A7  3520
//#define NOTE_AS7 3729
//#define NOTE_B7  3951
//#define NOTE_C8  4186
//#define NOTE_CS8 4435
//#define NOTE_D8  4699
//#define NOTE_DS8 4978
#define REST 0

void printColor(uint16_t r, uint16_t g, uint16_t b, uint16_t c)
{
    Serial.print("R: ");
    Serial.print(r);
    Serial.print(" G: ");
    Serial.print(g);
    Serial.print(" B: ");
    Serial.print(b);
    Serial.print(" C: ");
    Serial.println(c);
}

int melodywii[] = {

  NOTE_FS4,8, REST,8, NOTE_A4,8, NOTE_CS5,8, REST,8,NOTE_A4,8, REST,8, NOTE_FS4,8, //1
  NOTE_D4,8, NOTE_D4,8, NOTE_D4,8, REST,8, REST,4, REST,8, NOTE_CS4,8,
  NOTE_D4,8, NOTE_FS4,8, NOTE_A4,8, NOTE_CS5,8, REST,8, NOTE_A4,8, REST,8, NOTE_F4,8,
  NOTE_E5,-4, NOTE_DS5,8, NOTE_D5,8, REST,8, REST,4,
  
  NOTE_GS4,8, REST,8, NOTE_CS5,8, NOTE_FS4,8, REST,8,NOTE_CS5,8, REST,8, NOTE_GS4,8, //5
  REST,8, NOTE_CS5,8, NOTE_G4,8, NOTE_FS4,8, REST,8, NOTE_E4,8, REST,8,
  NOTE_E4,8, NOTE_E4,8, NOTE_E4,8, REST,8, REST,4, NOTE_E4,8, NOTE_E4,8,
  NOTE_E4,8, REST,8, REST,4, NOTE_DS4,8, NOTE_D4,8, 

  NOTE_CS4,8, REST,8, NOTE_A4,8, NOTE_CS5,8, REST,8,NOTE_A4,8, REST,8, NOTE_FS4,8, //9
  NOTE_D4,8, NOTE_D4,8, NOTE_D4,8, REST,8, NOTE_E5,8, NOTE_E5,8, NOTE_E5,8, REST,8,
  REST,8, NOTE_FS4,8, NOTE_A4,8, NOTE_CS5,8, REST,8, NOTE_A4,8, REST,8, NOTE_F4,8,
  NOTE_E5,2, NOTE_D5,8, REST,8, REST,4,

  NOTE_B4,8, NOTE_G4,8, NOTE_D4,8, NOTE_CS4,4, NOTE_B4,8, NOTE_G4,8, NOTE_CS4,8, //13
  NOTE_A4,8, NOTE_FS4,8, NOTE_C4,8, NOTE_B3,4, NOTE_F4,8, NOTE_D4,8, NOTE_B3,8,
  NOTE_E4,8, NOTE_E4,8, NOTE_E4,8, REST,4, REST,4, NOTE_AS4,4,
  NOTE_CS5,8, NOTE_D5,8, NOTE_FS5,8, NOTE_A5,8, REST,8, REST,4, 

};

int melodyharry[] = {
  REST, 2, NOTE_D4, 4,
  NOTE_G4, -4, NOTE_AS4, 8, NOTE_A4, 4,
  NOTE_G4, 2, NOTE_D5, 4,
  NOTE_C5, -2, 
  NOTE_A4, -2,
  NOTE_G4, -4, NOTE_AS4, 8, NOTE_A4, 4,
  NOTE_F4, 2, NOTE_GS4, 4,
  NOTE_D4, -1, 
  NOTE_D4, 4,

  NOTE_G4, -4, NOTE_AS4, 8, NOTE_A4, 4, //10
  NOTE_G4, 2, NOTE_D5, 4,
  NOTE_F5, 2, NOTE_E5, 4,
  NOTE_DS5, 2, NOTE_B4, 4,
  NOTE_DS5, -4, NOTE_D5, 8, NOTE_CS5, 4,
  NOTE_CS4, 2, NOTE_B4, 4,
  NOTE_G4, -1,
  NOTE_AS4, 4,
     
  //  NOTE_D5, 2, NOTE_AS4, 4,//18
  //  NOTE_D5, 2, NOTE_AS4, 4,
  //  NOTE_DS5, 2, NOTE_D5, 4,
  //  NOTE_CS5, 2, NOTE_A4, 4,
  //  NOTE_AS4, -4, NOTE_D5, 8, NOTE_CS5, 4,
  //  NOTE_CS4, 2, NOTE_D4, 4,
  //  NOTE_D5, -1, 
  //  REST,4, NOTE_AS4,4,  

  // NOTE_D5, 2, NOTE_AS4, 4,//26
  // NOTE_D5, 2, NOTE_AS4, 4,
  // NOTE_F5, 2, NOTE_E5, 4,
  // NOTE_DS5, 2, NOTE_B4, 4,
  // NOTE_DS5, -4, NOTE_D5, 8, NOTE_CS5, 4,
  // NOTE_CS4, 2, NOTE_AS4, 4,
  // NOTE_G4, -1, 
  
};
int melody[] = {

  NOTE_A4,-4, NOTE_A4,-4, NOTE_A4,16, NOTE_A4,16, NOTE_A4,16, NOTE_A4,16, NOTE_F4,8, REST,8,
  NOTE_A4,-4, NOTE_A4,-4, NOTE_A4,16, NOTE_A4,16, NOTE_A4,16, NOTE_A4,16, NOTE_F4,8, REST,8,
  NOTE_A4,4, NOTE_A4,4, NOTE_A4,4, NOTE_F4,-8, NOTE_C5,16,

  NOTE_A4,4, NOTE_F4,-8, NOTE_C5,16, NOTE_A4,2,//4
  NOTE_E5,4, NOTE_E5,4, NOTE_E5,4, NOTE_F5,-8, NOTE_C5,16,
  NOTE_A4,4, NOTE_F4,-8, NOTE_C5,16, NOTE_A4,2,
  
};

byte buzzer = A0;
byte tempoharry = 144;
byte tempowii = 114;
byte tempo = 120;

int noteswii = sizeof(melodywii) / sizeof(melodywii[0]) / 2;
int notesharry = sizeof(melodyharry) / sizeof(melodyharry[0]) / 2;
int notes = sizeof(melody) / sizeof(melody[0]) / 2;

int wholenotewii = (60000 * 4) / tempowii;
int wholenoteharry = (60000 * 4) / tempoharry;
int wholenote = (60000 * 4) / tempo;

int divider = 0;
int noteDuration = 0;

void musika()
{
    if (device.red > 200 && device.red > device.blue && device.red > device.green)
    {
        for (int thisNote = 0; thisNote < notes * 2; thisNote = thisNote + 2)
        {
            divider = melody[thisNote + 1];
            if (divider > 0)
            {
                noteDuration = (wholenote) / divider;
            }
            else if (divider < 0)
            {
                noteDuration = (wholenote) / abs(divider);
                noteDuration *= 1.5;
            }
            tone(buzzer, melody[thisNote], noteDuration * 0.9);
            delay(noteDuration);
            noTone(buzzer);
        }
    }

    if (device.blue > 200 && device.blue > device.red && device.blue > device.green)
    {
        for (int thisNote = 0; thisNote < noteswii * 2; thisNote = thisNote + 2)
        {
            divider = melodywii[thisNote + 1];
            if (divider > 0)
            {
                noteDuration = (wholenotewii) / divider;
            }
            else if (divider < 0)
            {
                noteDuration = (wholenotewii) / abs(divider);
                noteDuration *= 1.5;
            }
            tone(buzzer, melodywii[thisNote], noteDuration * 0.9);
            delay(noteDuration);
            noTone(buzzer);
        }
    }

    if (device.green > 200 && device.green > device.red && device.green > device.blue)
    {
        for (int thisNote = 0; thisNote < notesharry * 2; thisNote = thisNote + 2)
        {
            divider = melodyharry[thisNote + 1];
            if (divider > 0)
            {
                noteDuration = (wholenoteharry) / divider;
            }
            else if (divider < 0)
            {
                noteDuration = (wholenoteharry) / abs(divider);
                noteDuration *= 1.5;
            }
            tone(buzzer, melodyharry[thisNote], noteDuration * 0.9);
            delay(noteDuration);
            noTone(buzzer);
        }
    }
}

//time
long previousMillis = 0;         // will store last time
unsigned long currentMillis = 0; //will store current time
long interval = 1000;            // interval/delay at which to operate

//ENGINE
const byte ena = 3; //left motor - set speed
const byte in1 = 6;
const byte in2 = 13;
const byte in3 = 2;
const byte in4 = 4;
const byte enb = 5; //right motor - set speed

//controll the engine
int leftMotorSpeed = 0;
int rightMotorSpeed = 0;

//Initial Speed of Motor
const int initial_motor_speed = 150;

//IR SENSOR
const byte sensorNum = 6;

const byte sensorPIN[sensorNum]{7, 12, 11, 10, 9, 8};
//int linePosition;

long linePositionNum = 5; //random starting number so i can set 0 as a first digit, 5 will be removed
//COLOR & DISTANCE SENSOR

//PID CONTROLLER
//PID VARIABLES
int PID_Val = 0;
int error = 0;
int P = 0;
int I = 0;
int D = 0;
int prevError = 0;
int prevI = 0;
//PID CONSTANT
const byte Kp = 7;
const byte Kd = 2;
const byte Ki = 0;
const byte M0 = 0;

//Iterate over the IR sensors, and create a 6digit num (sensor)*10+new sensor - 000000 001100 010000
void getLinePositionNum()
{
    linePositionNum = 5;
    for (int i = 0; i < sensorNum; ++i)
        linePositionNum = (linePositionNum * 10) + digitalRead(sensorPIN[i]);
}

void getError()
{
    switch (linePositionNum)
    {
    case 5111110:
    {
        error = -15;
        break;
    }
    case 5111100:
    {
        error = -15;
        break;
    }
    case 5111000:
    {
        error = -15;
        break;
    }
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

    case 5000111: //turn right
    {
        error = 15;
        break;
    }
    case 5001111: //turn right
    {
        error = 15;
        break;
    }
    case 5011111: //turn right
    {
        error = 15;
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
        //Serial.println("Unkown Error - Line Following Status");
        break;
    }
    }
}

void calculate_pid()
{
    if (error != 777 && error != 999)
    {
        P = error;
        I = I + prevI;
        D = error - prevError;

        PID_Val = (Kp * P) + (Ki * I) + (Kd * D);

        prevI = I;
        prevError = error;
    }
}

void motor_control()
{
    //change nothing
    if (error == 777 || error == 999) //all black //all white
        return;
    //   if(error<10 && error > 0){
    //    // Calculating the effective motor speed:
    //    leftMotorSpeed = initial_motor_speed + PID_Val;
    //    rightMotorSpeed = initial_motor_speed;
    //   } else if(error>-10 && error < 0)
    //   {
    //      // Calculating the effective motor speed:
    //    leftMotorSpeed = initial_motor_speed;
    //    rightMotorSpeed = initial_motor_speed - PID_Val;
    //  }

    leftMotorSpeed = initial_motor_speed + PID_Val;
    rightMotorSpeed = initial_motor_speed - PID_Val;

    // The motor speed should not exceed the max PWM value
    leftMotorSpeed = constrain(leftMotorSpeed, 0, 255);
    rightMotorSpeed = constrain(rightMotorSpeed, 0, 255);

    analogWrite(enb, leftMotorSpeed);  //Left Motor Speed
    analogWrite(ena, rightMotorSpeed); //Right Motor Speed

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
    Serial.print("PID Value:  ");
    Serial.print(PID_Val);
    Serial.print("\t");
    Serial.print("Left:  ");
    Serial.print(leftMotorSpeed);
    Serial.print("\t");
    Serial.print("Right:  ");
    Serial.print(rightMotorSpeed);
    Serial.print("\t");
}

void printDEBUG()
{
    printMotors();
    printIRDigital();
    printErrorVal();
}

void turn(char dir)
{
    switch (dir)
    {
    case 'F': //forward
    {
        //left engine
        digitalWrite(in1, HIGH); //forward
        digitalWrite(in2, LOW);  //backwards
        //right engine
        digitalWrite(in3, HIGH); //forward
        digitalWrite(in4, LOW);  //backwards
        break;
    }
    case 'B': //backwards
    {
        //left engine
        digitalWrite(in1, LOW);  //forward
        digitalWrite(in2, HIGH); //backwards
        //right engine
        digitalWrite(in3, LOW);  //forwards
        digitalWrite(in4, HIGH); //backwards
        break;
    }
    case 'L': //left
    {
        //right engine
        digitalWrite(in1, HIGH); //forward
        digitalWrite(in2, LOW);  //backwards
        //left engine
        digitalWrite(in3, LOW);  //forwards
        digitalWrite(in4, HIGH); //backwards
        break;
    }
    case 'R': //right
    {
        //right engine
        digitalWrite(in1, LOW);  //forward
        digitalWrite(in2, HIGH); //backwards
        //left engine
        digitalWrite(in3, HIGH); //forwards
        digitalWrite(in4, LOW);  //backwards
        break;
    }
    case 'S': //stop
    {
        //left engine
        digitalWrite(in1, LOW); //forward
        digitalWrite(in2, LOW); //backwards
        //right engine
        digitalWrite(in3, LOW); //forwards
        digitalWrite(in4, LOW); //backwards
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
    if (error == 15) //SHARP RIGHT TURN
    {
        turn('B');
        analogWrite(enb, initial_motor_speed + 25 - 8); //Left Motor Speed
        analogWrite(ena, initial_motor_speed + 8);      //Right Motor Speed

        delay(350);
        analogWrite(enb, initial_motor_speed + 25 + 50 - 8); //Left Motor Speed
        analogWrite(ena, initial_motor_speed + 8);           //Right Motor Speed

        turn('R');
        while (error <= 4 && error >= -4)
        {
            getLinePositionNum(); //get the line position from the IR sensors XXX-XXX
            getError();           //get the amount of sway off track
        }
    }
    else if (error == -15) //SHARP LEFT TURN
    {
        turn('B');
        analogWrite(enb, initial_motor_speed + 25 - 8); //Left Motor Speed
        analogWrite(ena, initial_motor_speed + 8);      //Right Motor Speed

        delay(350);
        analogWrite(enb, initial_motor_speed + 25 - 8); //Left Motor Speed
        analogWrite(ena, initial_motor_speed + 50 + 8); //Right Motor Speed
        turn('L');

        while (error <= 4 && error >= -4)
        {
            getLinePositionNum(); //get the line position from the IR sensors XXX-XXX
            getError();           //get the amount of sway off track
        }
    }
}

int8_t status = NO_ERROR;

void printError(int8_t error_code){
  if (error_code == NO_ERROR)
    Serial.println("No error :)");
  else if (error_code == I2C_ERROR)
    Serial.println("I2C comunication error :(");
  else if (error_code == INVALID_ARGUMENT)
    Serial.println("Invalid argument error :(");
  else 
    Serial.println("Unknown error O.O");
}



void setup()
{
    Serial.begin(9600); // Initialize serial comunication
    while (!Serial);                              // wait for serial to be ready
    device.init();                     // Initialize the comunication library
    device.reset();                    // Reset all interrupt settings and power off the device
    device.enableAlsEngine();          // enable the color/ALS engine
    device.setAlsIntegrationTime(450); // set the color engine integration time
    device.updateSaturation();         // updates the saturation value, stored in device.alsSaturation
    device.wakeUp();                   // wake up the device

    pinMode(ena, OUTPUT);
    pinMode(enb, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);

  // Every function of the library returns a status code (int8_t). 
  // There are 3 possible status codes (more may be added in the future):
  // NO_ERROR, I2C_ERROR, INVALID_ARGUMENT
  status = NO_ERROR;
  status = device.init(); // Initialize the comunication library
  printError(status);
  status = device.reset(); // Reset all interrupt settings and power off the device
  printError(status);

  status = device.enableProximityEngine(); // Enable the proximity engine
  printError(status);
  
  status = device.wakeUp(); // Wake up the device
  printError(status);


    //delay(2000);
}

void loop()
{
    device.updateColorData(); // update the values stored in device.red/green/blue/clear
    Serial.println("Raw color data:");
    printColor(device.red, device.green, device.blue, device.clear); // print raw values
    //musika();
    getLinePositionNum(); //get the line position from the IR sensors XXX-XXX
    getError();           //get the amount of sway off track

  status = device.updateProximityData(); // Update the proximity data and retrieve the status code
  //printError(status); // Examine the status code 
Serial.print("Prox: ");
  Serial.println(device.proximityData); // print the proximity data

    //delay(1000);//debug
    //sharpTurn();  //check if need to do sharp turns
    //calculate_pid();//check PID
    //motor_control();//set speed
    //printDEBUG(); //debug

    //analogWrite(enb, 100+5); //Left Motor Speed
    //analogWrite(ena, 100); //Right Motor Speed
    //turn('F');
        delay(1000);

    delay(5);
}
