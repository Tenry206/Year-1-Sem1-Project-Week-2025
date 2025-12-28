/* ------ Including Libraries ------ */


#include <LiquidCrystal.h>


/* ------ Defining Motor Driver L298N Pins ------ */


#define ENA 11
#define ENB 3


#define IN1 13
#define IN2 12
#define IN3 2
#define IN4 1


/* ------ Defining Ultrasonic Sensor Pins ------ */


#define trigPin A5
#define echoPin A4


/* ------ Defining Motor Speed ------ */


#define motorSpeedL 170
#define motorSpeedR 194


/* ------ Defining LCD Object ------ */


LiquidCrystal lcd(8, 9, 4, 5, 6, 7);


/* ------ Movement Functions ------ */


void forward() {

    analogWrite(ENA, motorSpeedR);
    analogWrite(ENB, motorSpeedL);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);

}


void stop() {
  
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  
}


void setup() {


    /* ------ Motor Driver L298 Pin Setup ------ */


    pinMode(ENA, OUTPUT);
    pinMode(ENB, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);


    /* ------ Initializing LCD ------ */


    lcd.begin(16, 2);
    lcd.setCursor(0, 0);
    lcd.clear();
    lcd.print("Let's drive");
    delay(1000);
}




void loop() {

    volatile float time = (millis())/1000.0 - 1;
  
    if (time < 10) {
      forward();
    } else {
      stop();
    }

    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Time: ");
    lcd.print(time);
}