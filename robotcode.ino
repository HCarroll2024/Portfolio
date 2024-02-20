#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <util/atomic.h>
#define PI 3.1415926535897932384626433832795
#define in1 10
#define in2 11
#define in3 6
#define in4 9
#define ENCAL 3
#define ENCBL 13
#define ENCAR 2
#define ENCBR 12

Adafruit_MPU6050 mpu;
double G5;
double G4;
double G3;
double G2;
double G1;
double G;
double Gcal;

volatile int posiL = 0;
volatile int posiR = 0;

void setup() {
  Serial.begin(9600);
  pinMode(ENCAL, INPUT);
  pinMode(ENCBL, INPUT);
  pinMode(ENCAR, INPUT);
  pinMode(ENCBR, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCAL),readEncoder,RISING);
  attachInterrupt(digitalPinToInterrupt(ENCAR),readEncoder,RISING);
  mpu.begin();

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  Gcal = g.gyro.z;
  Gcal += g.gyro.z;
  Gcal += g.gyro.z;
  Gcal += g.gyro.z;
  Gcal += g.gyro.z;
  Gcal /= 5;
  delay(1000);


//**************************Movement Code goes here******************************************
  

}

void forwardRightWheel(int speed) {
  digitalWrite(in4, LOW);
  analogWrite(in3, speed);
}
void forwardLeftWheel(int speed) {
  digitalWrite(in1, LOW);
  analogWrite(in2, speed);
}

void backRightWheel(int speed) {
  analogWrite(in4, speed);
  digitalWrite(in3, LOW);
}

void backLeftWheel(int speed) {
  analogWrite(in1, speed);
  digitalWrite(in2, LOW);
}

void stop() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, HIGH);
}

void forward(int speed, double dist) {
  double gain = 15;
  int vL = speed;
  int vR = speed;
  int max = 255;
  int min = 50;
  int posiLZero = posiL;
  int posiRZero = posiR;
  double d = 0;
  while(d < dist) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    /*G5 = G4;
    G4 = G3;
    G3 = G2;
    G2 = G1;
    G1 = g.gyro.z;
    G = ((G1 + G2 + G3 + G4 + G5) / 5) - Gcal;
    */
    
    G = g.gyro.z - Gcal;
    
    vL += (int)(G * gain);
    if(vL > max) {
      vL = max;
    }
    if(vL < min) {
      vL = min;
    }
    forwardLeftWheel(vL);

    vR -= (int)(G * gain); 
    if(vR > max) {
      vR = max;
    }
    if(vR < min) {
      vR = min;
    }
    forwardRightWheel(vR);

    double deg = (abs(posiL - posiLZero) + abs(posiR - posiRZero)) / 2;
    double rot = deg / 595.23;
    d = rot * PI * 65;
  }
  stop();
}

void backward(int speed, double dist) {
  double gain = 20;
  int vL = speed;
  int vR = speed;
  int max = 255;
  int min = 50;
  int posiLZero = posiL;
  int posiRZero = posiR;
  double d = 0;
  while(d < dist) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    /*G5 = G4;
    G4 = G3;
    G3 = G2;
    G2 = G1;
    G1 = g.gyro.z;
    G = ((G1 + G2 + G3 + G4 + G5) / 5) - Gcal;
    */
    
    G = g.gyro.z - Gcal;
    
    vR += (int)(G * gain);
    if(vL > max) {
      vL = max;
    }
    if(vL < min) {
      vL = min;
    }
    backLeftWheel(vL);

    vL -= (int)(G * gain); 
    if(vR > max) {
      vR = max;
    }
    if(vR < min) {
      vR = min;
    }
    backRightWheel(vR);

    double deg = (abs(posiL - posiLZero) + abs(posiR - posiRZero)) / 2;
    double rot = deg / 595.23;
    d = rot * PI * 65;
  }
  stop();
}

void turnRight() {
  int vL = 75;
  int vR = 75;
  double d = 0;
  int posiLZero = posiL;
  int posiRZero = posiR;
  while((abs(posiR - posiRZero) + abs(posiL - posiLZero)) / 2 <= 379) {
      forwardLeftWheel(vL);
      backRightWheel(vR);
      if(abs(posiL - posiLZero) < abs(posiR - posiRZero)) {
        vL++;
        vR--;
      }
      if(abs(posiR - posiRZero) < abs(posiL - posiLZero)) {
        vR++;
        vL--;
      }
  }
  stop();
}

void turnLeft() {
  int vL = 75;
  int vR = 75;
  double d = 0;
  int posiRZero = posiR;
  int posiLZero = posiL;
  while((abs(posiR - posiRZero) + abs(posiL - posiLZero)) / 2 <= 385) {
      backLeftWheel(vL);
      forwardRightWheel(vR);
      if(abs(posiL - posiLZero) < abs(posiR - posiRZero)) {
        vL++;
        vR--;
      }
      if(abs(posiR - posiRZero) < abs(posiL - posiLZero)) {
        vR++;
        vL--;
      }
  }
  stop();
}

void loop() {
  int posL = 0;
  int posR = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    posL = posiL;
    posR = posiR;
  }

  Serial.print(posL);
  Serial.print(" ");
  Serial.println(posR);

}

void readEncoder(){
  int bL = digitalRead(ENCBL);
  int bR = digitalRead(ENCBR);
  if(bL > 0) {
    posiL++;
  }
  else {
    posiL--;
  }
  if(bR > 0) {
    posiR++;
  }
  else {
    posiR--;
  }
}

