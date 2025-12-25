/* ------ Include Libraries ------ */


#include <LiquidCrystal.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>


/* ------ Defining Motor Driver L298N Pins ------ */


#define ENA 11
#define ENB 3

#define IN1 13
#define IN2 12
#define IN3 A2
#define IN4 1


/* ------ Defining Rotary Encoder Pins ------ */


#define ENCODER_PIN_1 2


/* ------ Defining Wheel Circumference ------ */


#define CIRCUMFERENCE 20.42


/* ------ Defining Objects ------ */

Adafruit_MPU6050 mpu;
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);


/* ------ Declaring Variables ------ */

unsigned long lastTime360;
float gyroZ;
volatile float angleZ;
unsigned long lastTime;
volatile long wheelCounter1;
float distance_1;
bool turn360state = false;
bool ramp = false;


/* ------ Interrupt Service Routine ------ */


void encoder_1_ISR(void) {

    wheelCounter1++;

}


/* ------ Motor Control Function ------ */


void setMotor(int leftSpeed, int rightSpeed) {


/* ------ Left Motor Control ------ */


  if (leftSpeed >= 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }

  else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    leftSpeed = -leftSpeed;
  }

  analogWrite(ENB, constrain(leftSpeed, 0, 255));


/* ------ Right Motor Control ------ */


  if (rightSpeed >= 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }

  else {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    rightSpeed = -rightSpeed;
  }

  analogWrite(ENA, constrain(rightSpeed, 0, 255));

}


void angleCalculate(float z) {

  gyroZ = z * 57.2958;

  unsigned long now = micros();
  float dt = (now - lastTime) / 1e6;
  lastTime = now;

  angleZ += gyroZ * dt;

}



void rampState(float angleZ) {


  volatile unsigned long now360 = micros();

  if (now360 - lastTime360 <= 4e6) {
    setMotor(0,0);
  }

  else {

    if (angleZ < 360) {
      setMotor(-200,200);
    }

    else {
      turn360state = true;
      setMotor(0,0);
      lastTime360 = now360;
    }

  }

}


void setup() {

/* ------ I2C Communication ------ */


  Wire.begin();
  mpu.begin();


/* ------ Motor Driver L298N Pin Setup ------ */


  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);


/* ------ Initializing LCD ------ */


  lcd.begin(16,2);
  lcd.clear();
  lcd.print("Let's drive!");
  delay(1000);


/* ------ Rotary Encoder Pin Setup ------ */


  pinMode(ENCODER_PIN_1, INPUT);


/* ------ Encoder Interrupt ------- */


  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_1), encoder_1_ISR, RISING);


/* ------ Setting Last Time ------ */


  lastTime = micros();


}


void loop() {


  /* ------ Variables ------ */


  sensors_event_t a, g, temp;
  float angle;
  float adjustedAngle;
  float adjustedAngleZ;


  /* ------ MPU Readings =----- */




  mpu.getEvent(&a, &g, &temp);
  angle = atan2(a.acceleration.x, a.acceleration.z);
  adjustedAngle = (angle + 0.065) * 55.8;


  if (adjustedAngle > 20) ramp = true;
    
    if (ramp == true && adjustedAngle < 5){
    lastTime = micros();
    lastTime360 = micros();
    angleZ = 0;


    while(1){
      mpu.getEvent(&a, &g, &temp);
      angleCalculate(g.gyro.z);
      adjustedAngleZ = angleZ * 1.0285;
      rampState(adjustedAngleZ);
    
    volatile float time = (millis())/1000.0 - 1;

    distance_1 = wheelCounter1 / 27.0 * CIRCUMFERENCE;


    lcd.setCursor(0,0);
    lcd.print("d:");

    lcd.setCursor(3, 0);
    lcd.print(distance_1);

    lcd.setCursor(7, 0);
    lcd.print(", t: ");

    lcd.setCursor(12, 0);
    lcd.print(time);
    
    lcd.setCursor(0,1);
    lcd.print("Angle: ");

    lcd.print(adjustedAngleZ);
    delay(100);

      lcd.clear();
      if (turn360state) {
        turn360state = false;
        ramp = false;
        break;
      }
    }


  }


  else {
    volatile float time = (millis())/1000.0 - 1;


  distance_1 = wheelCounter1 / 27.0 * CIRCUMFERENCE;


  lcd.setCursor(0,0);
  lcd.print("d:");

  lcd.setCursor(3, 0);
  lcd.print(distance_1);

  lcd.setCursor(7, 0);
  lcd.print(", t: ");

  lcd.setCursor(12, 0);
  lcd.print(time);
  
  lcd.setCursor(0,1);
  lcd.print("Angle: ");

  lcd.print(adjustedAngle);
  delay(100);
  lcd.clear();

  setMotor(200,200);

  }





}