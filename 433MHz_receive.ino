// BubbleBlast V2.0 / By Niklas Roy / Released under a Beer Ware license 
// library for 433MHz radio: https://www.pjrc.com/teensy/td_libs_VirtualWire.html
// A1 Voltage sensing Battery via 1/3 Voltage divider
// Motor driver on 3,5,6,11

#ifdef __AVR__
#include <avr/power.h>
#endif
#include <VirtualWire.h>

#define LEFT 2
#define RIGHT 1


const int led_pin = 13;
boolean   toggleLED = HIGH;

const int receive_pin = 12;

unsigned long lastMessageTime;
unsigned long displaySignalTimer;

//433 MHz communication
uint8_t buf[VW_MAX_MESSAGE_LEN];
uint8_t buflen = VW_MAX_MESSAGE_LEN;
uint8_t receiveX;
uint8_t receiveY;

float smoothVoltage = 0;


//////////////////////////////////////////////////////////////////////////////////////////////// SETUP
void setup()
{
  Serial.begin(9600);  // Debugging only
  Serial.println("setup");


  //initialise the IO and ISR for 433 MHz communication
  vw_set_rx_pin(receive_pin);
  vw_setup(2000);  // Bits per sec
  vw_rx_start();   // Start the receiver PLL running

  //set default value for received message
  buf[0] = 136;
  buf[1] = 136;
  receiveX = 8;
  receiveY = 8;

  //motordriver
  pinMode(3, OUTPUT);  // 3:  right - forward
  pinMode(5, OUTPUT);  // 5:  right - back
  pinMode(6, OUTPUT);  // 6:  left - forward
  pinMode(11, OUTPUT); // 11: left - back

  //battery voltage indicator
  pinMode(A3, OUTPUT);
  pinMode(A4, OUTPUT);
  pinMode(A5, OUTPUT);

  int d = 70;
  for (int i = 0; i < 2; i++) {
    digitalWrite(A3, HIGH);
    delay(d);
    digitalWrite(A4, HIGH);
    delay(d);
    digitalWrite(A5, HIGH);
    delay(d);
    digitalWrite(A3, LOW);
    delay(d);
    digitalWrite(A4, LOW);
    delay(d);
    digitalWrite(A5, LOW);
    delay(d);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////// LOOP
void loop()
{

  //receive direction via 433MHz and write into variables 'receiveX' and 'receiveY'
  receiveMsg(1); //0 = blue car / 1 = red car

  //check how old the last correct message is and calculate 'signalQuality' value
  float messageAge = millis() - lastMessageTime;
  messageAge = constrain(messageAge, 500, 1000);
  float signalQuality = map(messageAge, 500, 1000, 100, 0);
  signalQuality = signalQuality / 100; //0=no signal (not received a signal for a while) ; 1=good signal (recently received a good signal)

  //calculate motor movement from X & Y
  float rX = receiveX;
  float rY = receiveY;

  //center values around 0
  rX = rX - 8;
  rY = rY - 8;

  //scale the values to range from -1 to 1
  rX = rX / 7;
  rY = rY / 7;

  //calculate motor movements
  float motorL = rX + rY;
  float motorR = -(rX - rY);

  //read battery voltage
  float voltage = readVoltage(A1);

  //move motors
  moveMotor(LEFT,  constrain(motorL * 100 * signalQuality, -100, 100), voltage);
  moveMotor(RIGHT, constrain(motorR * 100 * signalQuality, -100, 100), voltage);

  //indicate signal quality via onboard LED
  if (signalQuality > 0) { 
    if (displaySignalTimer +  150 - signalQuality * 100 < millis()) { //blink fast for good signal; slow for bad signal
      toggleLED = !toggleLED;
      digitalWrite(led_pin, toggleLED);
      displaySignalTimer = millis();
    }
  } else {
    digitalWrite(led_pin, LOW); //turn LED off for no signal
  }
}



//////////////////////////////////////////////////////////////////////////////////////////////// Move motors
void moveMotor(uint8_t side, float tempo, float volt) {

  float pwm = tempo * 2.55;
  pwm = pwm / volt * 8;
  pwm = constrain(pwm, -255, 255);

  //3:right - forward
  //5:right - back


  if (side == RIGHT) {
    if (pwm < 1 && pwm > -1) {
      analogWrite(3, 0);  //stop right motor
      analogWrite(5, 0);
    }
    if (pwm > 0) {
      analogWrite(3, int(pwm)); //right-forward
      analogWrite(5, 0);
    }
    if (pwm < 0) {
      pwm = -pwm;
      analogWrite(5, int(pwm)); //right-Backward
      analogWrite(3, 0);
    }
  }


  //6:left - forward
  //11:left - back

  if (side == LEFT) {
    if (pwm < 1 && pwm > -1) {
      analogWrite(6, 0);  //stop left motor
      analogWrite(11, 0);
    }
    if (pwm > 0) {
      analogWrite(6, int(pwm)); //left-Forward
      analogWrite(11, 0);
    }
    if (pwm < 0) {
      pwm = -pwm;
      analogWrite(11, int(pwm)); //left-Backward
      analogWrite(6, 0);
    }

  }
}

//////////////////////////////////////////////////////////////////////////////////////////////// Receive two bytes
void receiveMsg(int carNumber) {

  //receive two bytes via 433MHz communication and write them into buf[0] and buf[1]
  // Non-blocking, only get messages w/ good checksum
  if (vw_get_message(buf, &buflen))
  {
    //buf[0] contains values for car1 ; buf[1] contains values for car2 ;
    receiveX = buf[carNumber] & 0b00001111;        //decode X from byte
    receiveY = (buf[carNumber] >> 4) & 0b00001111; //decode Y from byte
    lastMessageTime = millis();
  }
}


//////////////////////////////////////////////////////////////////////////////////////////////// Read battery voltage and return value as float
float readVoltage(int inputPin) {
  float readVoltage = analogRead(inputPin);
  readVoltage = readVoltage / 1023 * 15; //take 1/3 voltage divider into account
  if (smoothVoltage == 0) {
    smoothVoltage = readVoltage; //store voltage in 'smoothVoltage' if this code is executed for 1st time after startup
  }
  smoothVoltage = (smoothVoltage * 9 + readVoltage) / 10; //smoothen out the actual read voltage

  digitalWrite(A3, LOW);
  digitalWrite(A4, LOW);
  digitalWrite(A5, LOW);

  if (smoothVoltage <= 9.4) { //battery empty --> switch motors off, stall & blink
    //stop motors
    analogWrite(3, 0);
    analogWrite(5, 0);
    analogWrite(6, 0);
    analogWrite(11, 0);
    //stall & blink LED
    while (1) {
      digitalWrite(A3, HIGH);
      delay(50);
      digitalWrite(A3, LOW);
      delay(50);
      digitalWrite(A3, HIGH);
      delay(50);
      digitalWrite(A3, LOW);
      delay(50);
      digitalWrite(A3, HIGH);
      delay(50);
      digitalWrite(A3, LOW);
      delay(950);
    }
  }
  //display battery state via 3 LED's
  if (smoothVoltage >= 9.8)  {
    digitalWrite(A3, HIGH);
  }
  if (smoothVoltage >= 10.2) {
    digitalWrite(A4, HIGH);
  }
  if (smoothVoltage >= 10.6) {
    digitalWrite(A5, HIGH);
  }
  return smoothVoltage;
}

