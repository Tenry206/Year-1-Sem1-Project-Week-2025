/* ----- Include Library -----*/
#include <LiquidCrystal.h>

/* ----- Defining Motor Driver L298N Pins -----*/
#define ENA 11
#define ENB 3

#define IN1 13
#define IN2 12
#define IN3 A2
#define IN4 1

/* ----- Motor Driver: Motor Speed -----*/
#define baseSpeed 240
#define maxSpeed 255

/* ------ Defining LCD Object ------ */
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

/* ----- Defining Ultrasonic sensor ----- */
#define trigPin A1
#define echoPin A3

/* ------ Motor Control Function ------ */


void setMotor(int leftSpeed, int rightSpeed) {


/* ------ Left Motor Control ------ */


  if (leftSpeed >= 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }
  else{
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
  else{
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    rightSpeed = -rightSpeed;
  }

  analogWrite(ENA, constrain(rightSpeed, 0, 255));


}


void setup() {
  // put your setup code here, to run once:
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

  /* ----- Ultrasonic Setup ------ */
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin,HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin,LOW);

  long duration = pulseIn(echoPin,HIGH);
  float distance = duration * 0.0343/2;

  if (distance <=20 && distance >0){
    setMotor(255,-255);
    delay(500);
  }else{
    setMotor(255,255);
    
  }

}