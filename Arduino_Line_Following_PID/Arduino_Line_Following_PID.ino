/* ------ Include Libraries ------ */


#include <LiquidCrystal.h>


/* ------ Defining Motor Driver L298N Pins ------ */


#define ENA 11
#define ENB 3

#define IN1 13
#define IN2 12
#define IN3 A2
#define IN4 1


/* ------ Defining Rotary Encoder Pins ------ */


#define ENCODER_PIN_1 2


/* ------ Defining IR Sensor Pins ------ */


#define leftIRPin A5
#define centerIRPin A4
#define rightIRPin A3 


/* ------ Motor Driver: Motor Speed ------ */


#define baseSpeed 240
#define maxSpeed 255


/* ------ PID Constants ------ */


float Kp = 30.0;
float Ki = 0.8;
float Kd = 18.0;

float integral = 0;
float lastError = 0;


/* ------ Defining Wheel Circumference ------ */


#define CIRCUMFERENCE 20.42


/* ------ Defining LCD Object ------ */


LiquidCrystal lcd(8, 9, 4, 5, 6, 7);


/* ------ Declaring Variables ------ */


bool IRSensorLeft;
bool IRSensorRight;

volatile long wheelCounter1;
float distance_1;


/* ------ Interrupt Service Routine ------ */


void encoder_1_ISR(void) {

    wheelCounter1++;

}


/* ------ Error Function for PID ------ */


void errorFunction(int &error, bool &lost, bool &crossing) {
  
  int L = !digitalRead(leftIRPin);
  int C = digitalRead(centerIRPin);
  int R = !digitalRead(rightIRPin);

  lost = false;
  crossing = false;

  int pattern = (L << 2) | (C << 1) | (R);

  switch (pattern) {
    case 0b010:
    error = 0;
    break;

    case 0b110:
    error = -1;
    break;

    case 0b100:
    error = -2;
    break;

    case 0b011:
    error = 1;
    break;

    case 0b001:
    error = 2;
    break;

    case 0b111:
    crossing = true;
    error = 0;
    break;

    case 0b000:
    default:
      lost = true;
      error = lastError;
    break;
  }
}


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


/* ------ Motor Driver L298N Pin Setup ------ */


  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);


/* ------ IR Sensor Array Pin Setup ------ */


  pinMode(leftIRPin, INPUT);
  pinMode(centerIRPin, INPUT);
  pinMode(rightIRPin, INPUT);


/* ------ Initializing LCD ------ */


  lcd.begin(16,2);
  lcd.clear();
  lcd.print("Let's drive!");
  delay(1000);


/* ------ Rotary Encoder Pin Setup ------ */


  pinMode(ENCODER_PIN_1, INPUT);


/* ------ Encoder Interrupt ------- */


  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_1), encoder_1_ISR, RISING);


}


void loop() {


  int error;
  bool lost, crossing;


/* ------ Operational Time ------ */


  volatile float time = (millis())/1000.0 - 1;


/* ------ Error Function ------ */


  errorFunction(error, lost, crossing);


/* ------ Special Cases (Lost or Crossing) ------ */


  if (crossing) {

    setMotor(0,0);
    integral = 0;
    while(1);
    return;

  }

  if (lost) {

    int searchSpeed = 175;
    if (lastError > 0) {
      setMotor(searchSpeed, -searchSpeed);
    }
    else if (lastError < 0) {
      setMotor(-searchSpeed, searchSpeed);
    }
    else {
      setMotor(255,-255);
    }
    return;

  }


/* ------ Dynamic Speed Control ------ */


  int dynamicSpeed = baseSpeed;
  int turn = abs(error);

  dynamicSpeed = baseSpeed - 15 * turn;
  dynamicSpeed = constrain(dynamicSpeed, 95, baseSpeed);


/* ------ PID Calculations ------ */


  integral += error;
  float derivative = error - lastError;


  /* ------ Start of EMA Filter ------ */


  static float dFilter = 0;
  float alpha = 0.55;
  dFilter = alpha * dFilter + (1 - alpha) * derivative;


  /* ------ End of EMA Filter ------ */


  /* ------ Start of Extra Speed Bump ------ */


  float extra = 0;

  if (abs(derivative) > 1.5) {
    extra = 0.5 * derivative;
  }
  

  /* ------ End of Extra Speed Bump ------ */


  float correction = Kp * error + Ki * integral + Kd * dFilter + extra;

  int leftSpeed = dynamicSpeed - (int)correction;
  int rightSpeed = dynamicSpeed + (int)correction;

  leftSpeed = constrain(leftSpeed, -maxSpeed, maxSpeed);
  rightSpeed = constrain(rightSpeed, -maxSpeed, maxSpeed);

  setMotor(leftSpeed, rightSpeed);

  lastError = error;


/* ------ Movement for Line Following Robot ------ */


  lcd.clear();


  distance_1 = wheelCounter1 / 27.0 * CIRCUMFERENCE;


  lcd.setCursor(0,0);
  lcd.print("Distance:");

  lcd.setCursor(10, 0);
  lcd.print(distance_1);

  lcd.setCursor(0,1);
  lcd.print("Time: ");

  lcd.setCursor(6,1);
  lcd.print(time);
  
  delay(100);


}