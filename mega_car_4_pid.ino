#define M_R_1 22 // 4
#define M_R_2 24 // 2
#define M_L_1 26 // 12
#define M_L_2 28 // 13
#define EN_1 2  // for right wheel
#define EN_2 3 // for left wheel 
#define IR1 23
#define IR2 25
#define IR3 27
#define IR4 29
#define IR5 31
#define IR6 33
#define IR7 35
#define IR8 37
//define frontIR 18 //front ir sensor
//define backIR 20 // back ir sensor

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <SoftwareSerial.h>
SoftwareSerial BT(10, 19); // tx , rx of bluetooth

RF24 radio(7, 8);  // nRF24L01 (CE, CSN)
const byte address[6] = "00001"; // Address
// Max size of this struct is 32 bytes - NRF24L01 buffer limit
struct Data_Package {
  byte j1PotX;
  byte j1PotY;
  byte j1Button;
  byte j2PotX;
  byte j2PotY;
  byte j2Button;
  byte pot1;
  byte pot2;
  byte tSwitch1;
  byte tSwitch2;
  byte tSwitch3;
  byte tSwitch4;
  byte tSwitch5;
  byte tSwitch6;
};
Data_Package datas;


int speed1 = 0; // right wheel speed
int speed2 = 0; // left wheel speed
char PREdata;
int baseSpeed;
char data;

double kp;
double kd;
int error = 0;
int pidValue = 0;
int previousError = 0;

void setup()
{
  // Define the radio communication
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MAX);
  radio.startListening();

  //attachInterrupt(digitalPinToInterrupt(frontIR),stopy,FALLING);
  //attachInterrupt(digitalPinToInterrupt(backIR),stopy,FALLING);
  pinMode(M_R_1, OUTPUT);
  pinMode(M_R_2, OUTPUT);
  pinMode(M_L_1, OUTPUT);
  pinMode(M_L_2, OUTPUT);
  pinMode(IR1, INPUT);
  pinMode(IR2, INPUT);
  pinMode(IR3, INPUT);
  pinMode(IR4, INPUT);
  pinMode(IR5, INPUT);
  pinMode(IR6, INPUT);
  pinMode(IR7, INPUT);
  pinMode(IR8, INPUT);
  //Serial.begin(9600);
  BT.begin(9600);
  BT.println("bluetooth working");
}

void loop()
{

  if (BT.available() > 0)
  {
    data = BT.read();

    BT.println(data);
    if (data == 't')
    {
      bool temp = true;
      while (temp)
      {
        radioControl();
        if (BT.available() > 0)
        { data = BT.read();
          if (data == 'T')
          {
            temp = false;
          }
        }
      }
    }
    else if (data == 'i') // increasing speed of both wheel
    {
      speed1 += 25;
      speed2 += 25;
      BT.print("speed1=");
      BT.println(speed1);
      BT.print("speed2=");
      BT.println(speed2);
      data = PREdata;
    }
    else if (data == 'e') //decreasing speed both wheel
    {
      speed1 -= 25;
      speed2 -= 25;
      BT.print("speed1=");
      BT.println(speed1);
      BT.print("speed2=");
      BT.println(speed2);
      data = PREdata;
    }
    else if (data == 'f') // condition to start line follower
    {
      digitalWrite(M_R_1, 1);
      digitalWrite(M_R_2, 0);
      digitalWrite(M_L_1, 1);
      digitalWrite(M_L_2, 0);
      analogWrite(EN_1, 0);
      analogWrite(EN_2, 0);
      bool temp = true;

      BT.println("follow line condition");
      BT.println("enter baseSpeed"); // receing baseSpeed
      while (BT.available() == 0)
      {
      }
      String norm = BT.readString();
      baseSpeed = norm.toInt();

      BT.println("enter kp"); // receing base kp
      while (BT.available() == 0)
      {
      }
      String str1 = BT.readString();
      kp = str1.toDouble();

      BT.println("enter kd"); // receing base kd
      while (BT.available() == 0)
      {
      }
      String str2 = BT.readString();
      kd = str2.toDouble();

      while (temp)
      {
        calculateError();
        calculatePid();
        drivespd(baseSpeed - pidValue, baseSpeed + pidValue);
        //delay(50);
        if (BT.available() > 0)
        {
          char temp1 = BT.read();
          if (temp1 == 'F') // to stop the line follower
          { temp = false;
            BT.println("not following line condition");
          }
        }
      }

    }
    else if (data == 'c')
    {
      int ir1 = digitalRead(IR1);
      BT.print(" ir1=");
      BT.println(ir1);
      int ir2 = digitalRead(IR2);
      BT.print(" ir2=");
      BT.println(ir2);
      int ir3 = digitalRead(IR3);
      BT.print(" ir3=");
      BT.println(ir3);
      int ir4 = digitalRead(IR4);
      BT.print(" ir4=");
      BT.println(ir4);
      int ir5 = digitalRead(IR5);
      BT.print(" ir5=");
      BT.println(ir5);
      int ir6 = digitalRead(IR6);
      BT.print(" ir6=");
      BT.println(ir6);
      int ir7 = digitalRead(IR7);
      BT.print(" ir7=");
      BT.println(ir7);
      int ir8 = digitalRead(IR8);
      BT.print(" ir8=");
      BT.println(ir8);

    }

    else if (data == 'l') // incresing speed of left side wheel
    { speed2 += 10;
      data = PREdata;
    }
    else if (data == 'r') // incresing speed of right side wheel
    { speed1 += 10;
      data = PREdata;
    }
    else if (speed1 > 255)
    {
      speed1 = 255;
    }
    else if (speed1 <= 0)
    {
      speed1 = 0;
    }
    if (speed2 > 255)
    {
      speed2 = 255;
    }
    else if (speed2 < 0)
    { speed2 = 0;
    }

    else if (data == 'w')
    {
      forward(speed2, speed1);
    }
    else if (data == 'z')
    {
      //backward
      backward(speed2, speed1);
    }
    else if (data == 'a') // right side wheel will only rotate
    {
      //right
      left();
    }
    else if (data == 'd') // left side wheel will only rotate
    {
      //right
      right();
    }
    else if (data == 'o')
    {
      //Serial.println("stop with speed 0");
      BT.println("stop with speed 0");
      digitalWrite(M_R_1, 0);
      digitalWrite(M_R_2, 0);
      digitalWrite(M_L_1, 0);
      digitalWrite(M_L_2, 0);
      speed1 = 0;
      speed2 = 0;
      analogWrite(EN_1, speed1);
      analogWrite(EN_2, speed2);

    }
    else if (data == 's')
    {
      //Serial.println("stop");
      stopy();
    }
    PREdata = data;
  }
}
void forward(int lft, int rgt)
{
  //Serial.println("forward");
  BT.println("forward");
  digitalWrite(M_R_1, 1);
  digitalWrite(M_R_2, 0);
  digitalWrite(M_L_1, 1);
  digitalWrite(M_L_2, 0);
  /*if(rgt>230)
    {rgt=230;}
    if(lft>230)
    {lft=230;}*/
  if (rgt < 0)
  {
    rgt = 0;
  }
  if (lft < 0)
  {
    lft = 0;
  }
  analogWrite(EN_1, rgt);
  analogWrite(EN_2, lft);
}

void backward(int lft, int rgt)
{
  BT.println("backward");
  digitalWrite(M_R_1, 0);
  digitalWrite(M_R_2, 1);
  digitalWrite(M_L_1, 0);
  digitalWrite(M_L_2, 1);
  /*if(rgt>230)
    {rgt=230;}
    if(lft>230)
    {lft=230;}*/
  if (rgt < 0)
  {
    rgt = 0;
  }
  if (lft < 0)
  {
    lft = 0;
  }
  analogWrite(EN_1, rgt);
  analogWrite(EN_2, lft);
}
void right()
{
  //Serial.println("right");
  BT.println("right");
  digitalWrite(M_R_1, 0);
  digitalWrite(M_R_2, 0);
  digitalWrite(M_L_1, 1);
  digitalWrite(M_L_2, 0);
  analogWrite(EN_1, speed1);
  analogWrite(EN_2, speed2);
}
void left()
{
  //Serial.println("left");
  BT.println("left");
  digitalWrite(M_R_1, 1);
  digitalWrite(M_R_2, 0);
  digitalWrite(M_L_1, 0);
  digitalWrite(M_L_2, 0);
  analogWrite(EN_1, speed1);
  analogWrite(EN_2, speed2);
}
void stopy()
{
  BT.println("stop");
  digitalWrite(M_R_1, 0);
  digitalWrite(M_R_2, 0);
  digitalWrite(M_L_1, 0);
  digitalWrite(M_L_2, 0);
  analogWrite(EN_1, speed1);
  analogWrite(EN_2, speed2);
}
void calculateError()
{

  //on white, sensor gives 0 & on black sensor give 1
  int k1 = digitalRead(IR1);
  int k2 = digitalRead(IR2);
  int k3 = digitalRead(IR3);
  int k4 = digitalRead(IR4);
  int k5 = digitalRead(IR5);
  int k6 = digitalRead(IR6);
  int k7 = digitalRead(IR7);
  int k8 = digitalRead(IR8);
  if (k1 == 0)
  {
    error = 4;
  }
  else if (k8 == 0)
  {
    error = -4;
  }
  else if (k2 == 0)
  {
    error = 3;
  }
  else if (k7 == 0)
  {
    error = -3;
  }
  else if (k3 == 0)
  {
    error = 2;
  }
  else if (k6 == 0)
  {
    error = -2;
  }
  else if (k4 == 0)
  {
    error = 1;
  }
  else if (k5 == 0)
  {
    error = -1;
  }
  else if (k4 == 0 && k5 == 0)
  {
    error = 0;
  }
  else if (k1 == 1 && k2 == 1 && k3 == 1 && k4 == 1 && k5 == 1 && k6 == 1 && k7 == 1 && k8 == 1)
  {
    stopy();
  }
}
void calculatePid()
{
  int p = error;
  //BT.print(" error=");
  //BT.println(error);
  int d = error - previousError;
  pidValue = (kp * p) + (kd * d);
  //BT.print("pidValue=");
  //BT.println(pidValue);
  previousError = error;
}

void drivespd(int lftSpd, int rgtSpd)
{
  analogWrite(EN_1, rgtSpd);
  analogWrite(EN_2, lftSpd);
  /*BT.print(" rightspd=");
    BT.println(rgtSpd);
    BT.print("leftspd=");
    BT.println(lftSpd);*/
}
void radioControl()
{
  if (radio.available()) {
    radio.read(&datas, sizeof(Data_Package));
    //   BT.print(" datas.j1Potx=");
    //  BT.println(datas.j1PotX);
    int fullSpd = datas.pot1;
    BT.print(" datas.j1PotY=");
    BT.println(datas.j1PotY);

    BT.print(" datas.j2PotX=");
    BT.println(datas.j2PotX);

    // BT.print(" datas.j2PotY=");
    // BT.println(datas.j2PotY);

    // BT.print(" datas.j1Button=");
    //  BT.println(datas.j1Button);

    //  BT.print(" datas.j2Button=");
    // BT.println(datas.j2Button);

    BT.print(" datas.pot1=");
    BT.println(datas.pot1);

    //BT.print(" datas.pot2=");
    // BT.println(datas.pot2);

    // BT.print(" datas.tSwitch1=");
    // BT.println(datas.tSwitch1);

    //  BT.print(" datas.tSwitch2 =");
    // BT.println(datas.tSwitch2);

    //  BT.print("  datas.tSwitch3=");
    // BT.println(datas.tSwitch3);

    // BT.print(" datas.tSwitch4=");
    //  BT.println(datas.tSwitch4);

    // BT.print("  datas.tSwitch5=");
    // BT.println(datas.tSwitch5);

    // BT.print(" datas.tSwitch6 =");
    //BT.println(datas.tSwitch6);

    if (datas.tSwitch1 == 0)
    {
      //BT.print(" datas.tSwitch1=");
      //BT.println(datas.tSwitch1);
      return ;
    }

    int j2 = map(datas.j2PotX, 0, 255, 255, 0);
    int j1 = datas.j1PotY;
    if (j2 < 130 && j2 > 125 && j1 < 130 && j1 > 125) // stop condition
    {
      //BT.println("stopy by j1 j2");
      stopy();
      return;
    }
    else if (j2 > 130) // forward
    {
      //BT.println("forkward");
      if (j1 > 130) // turning right with forward
      {
        //BT.println("going straight with right");
        int k1 = map(j1, 130, 255, 0, fullSpd);
        int k2 = map(j2, 130, 255, 20, fullSpd);
        forward(k2, k2 - k1);
        return;
      }
      else if (j1 < 125) // turning left with forward
      {
        //BT.println("going straight with left");

        int k1 = map(j1, 125, 0, 0, fullSpd);
        int k2 = map(j2, 130, 255, 20, fullSpd);
        forward(k2 - k1, k2);
        return;
      }
      else   // going straight
      {
        //BT.println("going straight");
        int k2 = map(j2, 130, 255, 20, fullSpd);
        forward(k2, k2);
        return;
      }
    }
    else
    { //BT.println("backward");
      if (j1 > 130) // turning right with backward
      {
        //BT.println("turning right with backward");
        int k1 = map(j1, 130, 255, 0, fullSpd); // direction
        int k2 = map(j2, 130, 0, 20, fullSpd); // accelrator
        backward(k2, k2 - k1);
        return;
      }
      else if (j1 < 125) // turning left with backward
      {
        //BT.println("turning left with backward");
        int k1 = map(j1, 125, 0, 0, fullSpd); // direction
        int k2 = map(j2, 130, 0, 20, fullSpd); // accelrator
        backward(k2 - k1, k2);
        return;
      }
      else   // going straight back
      {
        //BT.println("going backward");
        int k2 = map(j2, 130, 0, 20, fullSpd);
        backward(k2, k2);
        return;
      }
    }

  }
}
