#include <Adafruit_NeoPixel.h>
#include <math.h>

#define DIAL_L_PIN 4
#define DIAL_R_PIN 5
#define OUT_N_PIN1 2 // OUT negative signal to other board
#define OUT_N_PIN2 9 // OUT negative LED
#define OUT_P_PIN1 3 // OUT positive signal to other board
#define OUT_P_PIN2 7 // OUT positive LED
#define NEXT_PIN   6 // IN signal from the other board
#define BUFF_SIZE 25 // actually one less than this
#define NEXT_PIN_IS_BUTTON true // do we need to debounce the "next" signal?

enum DIAL_T { dNEITHER, dL, dLB, dLR, dR, dRB, dRL, dUNKNOWN };

enum DIAL_T dial_prev = dNEITHER, dial_curr;

bool lastLHigh = false;
bool lastRHigh = false;
int8_t dialIncDec[BUFF_SIZE];
uint8_t dialReadIdx = 0;
uint8_t dialWriteIdx = 0;
bool alreadyWroteAvail = false;
uint8_t idx = 0;

void setup() {
//  Serial.begin(9600);
//  while (!Serial) { }

  pinMode(DIAL_L_PIN, INPUT);
  pinMode(DIAL_R_PIN, INPUT);
  pinMode(NEXT_PIN,   INPUT);
  pinMode(OUT_N_PIN1, OUTPUT);
  pinMode(OUT_N_PIN2, OUTPUT);
  pinMode(OUT_P_PIN1, OUTPUT);
  pinMode(OUT_P_PIN2, OUTPUT);

//  attachInterrupt(digitalPinToInterrupt(DIAL_L_PIN), dialCheckL, CHANGE);
//  attachInterrupt(digitalPinToInterrupt(DIAL_R_PIN), dialCheckR, CHANGE);
  
  memset(dialIncDec, 0, BUFF_SIZE * sizeof(int8_t));
}

void loop()
{
  uint8_t availReadCnt;
  bool nxt = false;
  bool newLHigh = false, newRHigh = false;

  if (idx % 128 == 0)
  {
    // check the next pin
    nxt = digitalRead(NEXT_PIN);
  }

  // update dial values with exact result
  newLHigh = digitalRead(DIAL_L_PIN);
  newRHigh = digitalRead(DIAL_R_PIN);
  dialCheck(lastLHigh, lastRHigh);
  lastLHigh = newLHigh;
  lastRHigh = newRHigh;
  availReadCnt = getAvailReadCnt();

//  Serial.print(nxt);
//  Serial.print(" ");
//  Serial.print(lastLHigh + (lastRHigh << 1));
//  Serial.print(" ");
//  Serial.println(availReadCnt);

  // check for a read signal
  if (nxt && idx % 40 == 0)
  {
    if (availReadCnt == 0)
      return;
#ifdef NEXT_PIN_IS_BUTTON
    delay(20);
    if (!digitalRead(NEXT_PIN))
      return;
#endif
    alreadyWroteAvail = false;
    
    if (dialIncDec[dialReadIdx] < 0)
    {
      digitalWrite(OUT_N_PIN1, HIGH);
      digitalWrite(OUT_P_PIN1, LOW);
#ifdef OUT_N_PIN2
      digitalWrite(OUT_N_PIN2, HIGH);
      digitalWrite(OUT_P_PIN2, LOW);
#endif
    }
    else
    {
      digitalWrite(OUT_N_PIN1, LOW);
      digitalWrite(OUT_P_PIN1, HIGH);
#ifdef OUT_N_PIN2
      digitalWrite(OUT_N_PIN2, LOW);
      digitalWrite(OUT_P_PIN2, HIGH);
#endif
    }
    while (digitalRead(NEXT_PIN)) { }
    availReadCnt--;
    dialReadIdx = (dialReadIdx + 1) % BUFF_SIZE;
    
    if (availReadCnt == 0)
    {
      digitalWrite(OUT_N_PIN1, LOW);
      digitalWrite(OUT_P_PIN1, LOW);
#ifdef OUT_N_PIN2
      digitalWrite(OUT_N_PIN2, LOW);
      digitalWrite(OUT_P_PIN2, LOW);
#endif
    }
  } // if (nxt)
  else if (!alreadyWroteAvail && availReadCnt > 0 && idx % 40 == 0)
  {
    digitalWrite(OUT_N_PIN1, HIGH);
    digitalWrite(OUT_P_PIN1, HIGH);
#ifdef OUT_N_PIN2
    digitalWrite(OUT_N_PIN2, HIGH);
    digitalWrite(OUT_P_PIN2, HIGH);
#endif
    alreadyWroteAvail = true;
  }

  idx++;
}

uint8_t getAvailReadCnt()
{
  if (dialWriteIdx >= dialReadIdx)
    return dialWriteIdx - dialReadIdx;
  else
    return dialWriteIdx + BUFF_SIZE - dialReadIdx;
}

void dialCheck(bool lHigh, bool rHigh)
{
  // check if the buffer has space available
  uint8_t availReadCnt = getAvailReadCnt();
  if (availReadCnt + 1 >= BUFF_SIZE)
    return;
  
  // all the condition checking ever
  if (lHigh)
  {
    if (rHigh)
    { // L and R
      switch (dial_prev) {
      case dLB:
      case dRB:
      case dLR:
      case dRL:
        dial_curr = dial_prev;
        break;
      case dL:
//        Serial.println("dLB");
        dial_curr = dLB;
        break;
      case dR:
//        Serial.println("dRB");
        dial_curr = dRB;
        break;
      default:
//        Serial.println("dUNKNOWN");
        dial_curr = dUNKNOWN;
      }
    }
    else
    { // L, no R
      switch (dial_prev) {
      case dL:
      case dLB:
      case dRL:
        dial_curr = dial_prev;
        break;
      case dRB:
//        Serial.println("dRL");
        dial_curr = dRL;
        break;
      case dNEITHER:
//        Serial.println("dL");
        dial_curr = dL;
        break;
      default:
//        Serial.println("dUNKNOWN");
        dial_curr = dUNKNOWN;
      }
    }
  }
  else
  {
    if (rHigh)
    { // R, no L
      switch (dial_prev) {
      case dR:
      case dRB:
      case dLR:
        dial_curr = dial_prev;
        break;
      case dLB:
//        Serial.println("dLR");
        dial_curr = dLR;
        break;
      case dNEITHER:
//        Serial.println("dR");
        dial_curr = dR;
        break;
      default:
//        Serial.println("dUNKNOWN");
        dial_curr = dUNKNOWN;
      }
    }
    else
    { // no L, no R
      switch (dial_prev) {
      case dNEITHER:
        dial_curr = dial_prev;
        break;
      case dRL:
//        Serial.println("decrement");
        dialIncDec[dialWriteIdx] = -1;
        dialWriteIdx = (dialWriteIdx + 1) % BUFF_SIZE;
        dial_curr = dNEITHER;
        break;
      case dLR:
//        Serial.println("increment");
        dialIncDec[dialWriteIdx] = 1;
        dialWriteIdx = (dialWriteIdx + 1) % BUFF_SIZE;
        dial_curr = dNEITHER;
        break;
      default:
//        Serial.println("dNEITHER");
        dial_curr = dNEITHER;
      }
    }
  }
  dial_prev = dial_curr;
}

