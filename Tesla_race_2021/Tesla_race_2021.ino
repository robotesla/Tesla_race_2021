#include <PS2X_lib.h>  //for v1.6
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

/******************************************************************
   set pins connected to PS2 controller:
     - 1e column: original
     - 2e colmun: Stef?
   replace pin numbers by the ones you use
 ******************************************************************/
//PS2手柄引脚；
#define PS2_DAT        13
#define PS2_CMD        11
#define PS2_SEL        10
#define PS2_CLK        12

// 电机控制引脚；
#define PWMD 3
#define DIRD 2
#define PWMC 5
#define DIRC 4
#define PWMB 6
#define DIRB 7
#define PWMA 9
#define DIRA 8

#define USMIN  1000 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2000 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates


char speed;//小车速度

/******************************************************************
   select modes of PS2 controller:
     - pressures = analog reading of push-butttons
     - rumble    = motor rumbling
   uncomment 1 of the lines for each mode selection
 ******************************************************************/
#define pressures   true
//#define pressures   false
#define rumble      true
//#define rumble      false

PS2X ps2x; // create PS2 Controller Class
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

//right now, the library does NOT support hot pluggable controllers, meaning
//you must always either restart your Arduino after you connect the controller,
//or call config_gamepad(pins) again after connecting the controller.

int error = 0;
byte type = 0;
byte vibrate = 0;

//初始化
void setup() {

  pinMode(PWMA, OUTPUT);
  pinMode(DIRA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(DIRB, OUTPUT);
  pinMode(PWMC, OUTPUT);
  pinMode(DIRC, OUTPUT);
  pinMode(PWMD, OUTPUT);
  pinMode(DIRD, OUTPUT);

  Serial.begin(57600);
  delay(500) ; //added delay to give wireless ps2 module some time to startup, before configuring it
  //CHANGES for v1.6 HERE!!! **************PAY ATTENTION*************

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  pwm.writeMicroseconds(0, 1500);
  //setup pins and settings: GamePad(clock, command, attention, data, Pressures?, Rumble?) check for error
  while (true)
  {
    error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);

    if (error == 0) {
      Serial.print("Found Controller, configured successful ");
      Serial.print("pressures = ");
      if (pressures)
        Serial.println("true ");
      else
        Serial.println("false");
      Serial.print("rumble = ");
      if (rumble)
        Serial.println("true)");
      else
        Serial.println("false");
      Serial.println("Try out all the buttons, X will vibrate the controller, faster as you press harder;");
      Serial.println("holding L1 or R1 will print out the analog stick values.");
      Serial.println("Note: Go to www.billporter.info for updates and to report bugs.");
      break;
    }
    else if (error == 1)
      Serial.println("No controller found, check wiring, see readme.txt to enable debug. visit www.billporter.info for troubleshooting tips");

    else if (error == 2)
      Serial.println("Controller found but not accepting commands. see readme.txt to enable debug. Visit www.billporter.info for troubleshooting tips");

    else if (error == 3)
      Serial.println("Controller refusing to enter Pressures mode, may not support it. ");

    //  Serial.print(ps2x.Analog(1), HEX);
  }
  type = ps2x.readType();
  switch (type) {
    case 0:
      Serial.print("Unknown Controller type found ");
      break;
    case 1:
      Serial.print("DualShock Controller found ");
      break;
    case 2:
      Serial.print("GuitarHero Controller found ");
      break;
    case 3:
      Serial.print("Wireless Sony DualShock Controller found ");
      break;
  }
}
//小车运动定义


void forward(int speed, int diff) { //小车前进
  digitalWrite(DIRA, HIGH);
  digitalWrite(DIRB, HIGH);
  analogWrite(PWMA, constrain(speed - diff, 0, 255));
  analogWrite(PWMB, constrain(speed + diff, 0, 255));
  //  analogWrite(PWMC, speed);
  //  analogWrite(PWMD, speed);
}

void back(int speed) { //小车后退
  digitalWrite(DIRA, LOW);
  digitalWrite(DIRB, LOW);

  analogWrite(PWMA, speed);
  analogWrite(PWMB, speed);

}
void stop() // 停止；
{
  digitalWrite(DIRA, LOW);
  digitalWrite(DIRB, LOW);
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
}


void loop() {
  /* You must Read Gamepad to get new values and set vibration values
    ps2x.read_gamepad(small motor on/off, larger motor strenght from 0-255)
    if you don't enable the rumble, use ps2x.read_gamepad(); with no values
    You should call this at least once a second
  */
  if (error == 1) //skip loop if no controller found
    return;

  if (type == 2) { //Guitar Hero Controller
    return;
  }
  else  { //DualShock Controller
    ps2x.read_gamepad(false, vibrate); //read controller and set large motor to spin at 'vibrate' speed
//
//
//    // 电机正转；
//    if (ps2x.Button(PSB_PAD_UP)) {
//      Serial.println("Up held this hard: ");
//      speed = 250;
//      forward(speed, 0);
//    }
//
//    // 电机反转；
//    if (ps2x.Button(PSB_PAD_DOWN)) {
//      Serial.print("Down held this hard: ");
//      speed = 150;
//      back(speed);
//    }
//
//
//    if (ps2x.Button(PSB_SELECT)) {
//      Serial.println("stop");
//      stop();
//    }
    delay(20);




    int LX = ps2x.Analog(PSS_LX);
    int RY = ps2x.Analog(PSS_RY);

    int angle = map(LX, 0, 255, 1000, 2000);
    pwm.writeMicroseconds(0, angle);



    if (ps2x.Button(PSB_L2)) {
      Serial.print("PSB_L2");

      back(100);
    }
    else if (ps2x.Button(PSB_R2)) {
      Serial.print("PSB_R2");

      forward(250, 0);
    }
    else
    {
      Serial.print("RY");
      if (RY < 127) //前进
      {
        forward(map(RY, 127, 0, 0, 255), (LX - 127) / 2);
      }

      else if (RY > 127)
      {
        back(map(RY, 127, 255, 0, 255));
      }
      else
      {
        stop();
      }

    }

  }
}
