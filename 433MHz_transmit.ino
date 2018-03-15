/*
  PIN I/O

  Pin 12 Output: to 433MHz transmitter data input
  Pin 11 Input: Start/Stop button w/ external pull-up resistor
  Pin 8  Output: bubble machine & beacon

  For all encoders:
  Photodiode bpw42 is connected with (-) pin to +5V and with (+)pin to Arduino INPUT and ARDUINO INPUT PIN is connected via 30KOhm to GND.
*/
//  Trackball blue:

#define X1b A0
#define X2b A1
#define Y1b A2
#define Y2b A3

// Trackball red:

#define X1r A7
#define X2r A6
#define Y1r 6
#define Y2r 7

// LED's in trackballs
#define BLUEON 9 /*Blue LED's in Blue trackball*/
#define BLUEWHITE 3 /* other LED's in Blue trackball (so they become white when turned on together w/ blue)*/
#define REDON 10/*Red LED's in Red trackball*/
#define REDWHITE 5/* other LED's in Red trackball (so they become white when turned on together w/ red)*/

// button & beacon
#define BUTTON 11
#define BEACON 8

#include <VirtualWire.h>

const int transmit_pin = 12;
unsigned long sendTimer;
int loopCount = 0;
signed long xPosb, yPosb;//current posisiotn blue trackball
boolean pX1b, pX2b, nX1b, nX2b, pY1b, pY2b, nY1b, nY2b;

signed long xPosr, yPosr;//current position red trackball
boolean pX1r, pX2r, nX1r, nX2r, pY1r, pY2r, nY1r, nY2r;

signed long pxPosr = 0; //previous positions
signed long pyPosr = 0;
signed long pxPosb = 0;
signed long pyPosb = 0;

float xMover = 0;//position change (movement)
float yMover = 0;
float xMoveb = 0;
float yMoveb = 0;

boolean start, pStart; // start-stop state
boolean pButtonState, buttonState;
unsigned long lastButtonTime = 0; //last time button was pushed

int buttonCount;

//------------------------------------------------------------------------------------------------------- SETUP
void setup()
{
  Serial.begin(9600);
  Serial.println("let's go!");

  //trackballs inputs
  pinMode(X1b, INPUT);
  pinMode(X2b, INPUT);
  pinMode(Y1b, INPUT);
  pinMode(Y2b, INPUT);
  pinMode(X1r, INPUT);
  pinMode(X2r, INPUT);
  pinMode(Y1r, INPUT);
  pinMode(Y2r, INPUT);

  //led lights output
  pinMode(BLUEON, OUTPUT);
  pinMode(BLUEWHITE, OUTPUT);
  pinMode(REDON, OUTPUT);
  pinMode(REDWHITE, OUTPUT);

  //button input
  pinMode(BUTTON, INPUT);

  //beacon & bubble machine output
  pinMode(BEACON, OUTPUT);


  pX1b = digitalRead(X1b); //read & remember states of encoder photodiodes
  pX2b = digitalRead(X2b);

  pY1b = digitalRead(X1b); //read & remember states of encoder photodiodes
  pY2b = digitalRead(X2b);

  pX1r = digitalRead(X1r); //read & remember states of encoder photodiodes
  pX2r = digitalRead(X2r);

  pY1r = digitalRead(X1r); //read & remember states of encoder photodiodes
  pY2r = digitalRead(X2r);

  xPosb = 0; //reset position values
  yPosb = 0;
  yPosr = 0;
  xPosr = 0;

  // Initialise the IO and ISR
  vw_set_tx_pin(transmit_pin);
  vw_setup(2000);   // Bits per sec

  // reset send Timer
  sendTimer = millis();
}

//------------------------------------------------------------------------------------------------------- LOOP
void loop()
{
  pStart = start;
  buttonState = !digitalRead(BUTTON);
  if(buttonState){buttonCount++;}else{buttonCount=0;}
  /*
  if ( buttonState && pButtonState != buttonState) {
    //button is pushed
    delay(100);//debounce
    start = !start;
  }
  
  pButtonState = buttonState;
  */
  if (buttonCount==25){start=!start;}

  if (pStart != start && start) {
      //when game starts to run: switch trackball LED's to red & blue
      digitalWrite(BLUEON, HIGH);
      digitalWrite(BLUEWHITE, LOW);
      digitalWrite(REDON, HIGH);
      digitalWrite(REDWHITE, LOW);
      //turn beacon & bubble machine on
      digitalWrite(BEACON,HIGH);
  }
  if (start) {//game is running

    readTrackballs(); //read the encoders of both trackballs and write new position values in long variables xPosr, yPosr, xPosb and yPosb (r=red; b=blue)

    if (sendTimer < millis()) { //every 15ms:
      sendValues(); //calculate speeds and send the values via 433MHz transmitter
      sendTimer = millis() + 15; // calculate new timer value
    }
  }else{
      //trackball LED's go white when game is not running
      digitalWrite(BLUEON, 1);
      digitalWrite(BLUEWHITE, 1);
      digitalWrite(REDON,1);
      digitalWrite(REDWHITE,1);
      //turn beacon & bubble machine off
      digitalWrite(BEACON,LOW);
    }
}



//------------------------------------------------------------------------------------------------------- FUNCTIONS

void readTrackballs() { //reads the encoders of both trackballs and writes new position in xPosr, yPosr, xPosb and yPosb

  //-------------------------------------------------------- blue trackball
  //read X encoder
  nX1b = digitalRead(X1b); // read encoder states
  nX2b = digitalRead(X2b);

  if (nX1b && nX1b != pX1b) {
    if (!nX2b) {
      xPosb++;
    } else {
      xPosb--;
    }
  }
  if (!nX1b && nX1b != pX1b) {
    if (nX2b) {
      xPosb++;
    } else {
      xPosb--;
    }
  }
  if (nX2b && nX2b != pX2b) {
    if (nX1b) {
      xPosb++;
    } else {
      xPosb--;
    }
  }
  if (!nX2b && nX2b != pX2b) {
    if (!nX1b) {
      xPosb++;
    } else {
      xPosb--;
    }
  }

  pX1b = nX1b;
  pX2b = nX2b;

  //read Y encoder

  nY1b = digitalRead(Y1b); // read encoder states
  nY2b = digitalRead(Y2b);

  if (nY1b && nY1b != pY1b) {
    if (!nY2b) {
      yPosb++;
    } else {
      yPosb--;
    }
  }
  if (!nY1b && nY1b != pY1b) {
    if (nY2b) {
      yPosb++;
    } else {
      yPosb--;
    }
  }
  if (nY2b && nY2b != pY2b) {
    if (nY1b) {
      yPosb++;
    } else {
      yPosb--;
    }
  }
  if (!nY2b && nY2b != pY2b) {
    if (nY1b) {
      yPosb--;
    } else {
      yPosb++;
    }
  }

  pY1b = nY1b;
  pY2b = nY2b;

  //-------------------------------------------------------- red trackball
  //read X encoder
  nX1r = (analogRead(X1r) > 250); // read encoder states
  nX2r = (analogRead(X2r) > 400);


  if (nX1r && nX1r != pX1r) {
    if (!nX2r) {
      xPosr++;
    } else {
      xPosr--;
    }
  }
  if (!nX1r && nX1r != pX1r) {
    if (nX2r) {
      xPosr++;
    } else {
      xPosr--;
    }
  }
  if (nX2r && nX2r != pX2r) {
    if (nX1r) {
      xPosr++;
    } else {
      xPosr--;
    }
  }
  if (!nX2r && nX2r != pX2r) {
    if (!nX1r) {
      xPosr++;
    } else {
      xPosr--;
    }
  }

  pX1r = nX1r;
  pX2r = nX2r;

  //read Y encoder

  nY1r = digitalRead(Y1r); // read encoder states
  nY2r = digitalRead(Y2r);

  if (nY1r && nY1r != pY1r) {
    if (!nY2r) {
      yPosr++;
    } else {
      yPosr--;
    }
  }
  if (!nY1r && nY1r != pY1r) {
    if (nY2r) {
      yPosr++;
    } else {
      yPosr--;
    }
  }

  if (nY2r && nY2r != pY2r) {
    if (nY1r) {
      yPosr++;
    } else {
      yPosr--;
    }
  }

  if (!nY2r && nY2r != pY2r) {
    if (nY1r) {
      yPosr--;
    } else {
      yPosr++;
    }
  }

  pY1r = nY1r;
  pY2r = nY2r;
}


//------------------------------------------------------------------------------------------------------- calculate spedd values and send them out via 433MHz
void sendValues() {
  int xSmooth = 1;
  int ySmooth = 3;

  xMover = (xMover * xSmooth + (xPosr - pxPosr)) / (xSmooth + 1); //calculate trackball speeds from actual and previous positions; smoothen them
  yMover = (yMover * ySmooth + (yPosr - pyPosr)) / (ySmooth + 1);
  xMoveb = (xMoveb * xSmooth + (xPosb - pxPosb)) / (xSmooth + 1);
  yMoveb = (yMoveb * ySmooth + (yPosb - pyPosb)) / (ySmooth + 1);

  pxPosr = xPosr; //save current positions as "previous positions"
  pyPosr = yPosr;
  pxPosb = xPosb;
  pyPosb = yPosb;

  int xMr = int(-xMover * .8) + 8; //center speed values around 8
  int yMr = int(-yMover * 2.6) + 8;
  int xMb = int(-xMoveb * .8) + 8;
  int yMb = int(-yMoveb * 2.6) + 8;

  uint8_t rBytex = constrain(xMr, 0, 15);   //limit speeds to values from 1 to 15 (8=center)
  uint8_t rBytey = constrain(yMr, 0, 15);
  uint8_t bBytex = constrain(xMb, 0, 15);
  uint8_t bBytey = constrain(yMb, 0, 15);

  uint8_t rByte = rBytex | (rBytey << 4);     //redByte: encode X and Y in one Byte (1st Nibble is Y-value; last Nibble is X-value)
  uint8_t bByte = bBytex | (bBytey << 4);     //blueByte: encode X and Y in one Byte

  while (vw_tx_active());      // Wait until the whole (previous) message is gone
  uint8_t buf[2];              // buffer which will be transmitted
  buf[1] = rByte;              // write red joystick byte value in buffer
  buf[0] = bByte;              // write blue joystick byte value in buffer
  vw_send((uint8_t *)buf, 2);  // send values
}
