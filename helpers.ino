
#include "helpers.h"
#include "workitem.h"

void enableMotors() {
  digitalWrite(enPin1, LOW);
  digitalWrite(enPin2, LOW);
  delayMicroseconds(100);
}

void disableMotors() {
  digitalWrite(enPin1, HIGH);
  digitalWrite(enPin2, HIGH);
}

void setDirection(workItem item) {
  if (!dryRun) {
    if (item.x > 0) {
      digitalWrite(dirPins[0], HIGH);
    } else {
      digitalWrite(dirPins[0], LOW);
    }
    if (item.y > 0) {
      digitalWrite(dirPins[1], LOW);
    } else {
      digitalWrite(dirPins[1], HIGH);
    }
    delayMicroseconds(150);
  }
}

void servoMove(int servo, int pos) {
  if (dryRun) {
    return;
  }
  
  int pwm;
  // Create millis for HIGH duration
  pwm = (pos * 11) + 544;

  // Iterate a few times to give the servo a chance
  // for its arm to move
  for (int i = 0; i < 35 ; i++) {
    // One signal round is 20ms
    digitalWrite(servo, HIGH);
    delayMicroseconds(pwm);
    digitalWrite(servo, LOW);
    delayMicroseconds(20000);
  }
  penIsUp = (pos == PEN_UP);
}

void println(String text) {
#ifdef DEBUG  
  Serial.println(text);
  Serial.flush();
#endif  
}

void ISR_dummy() {
  // Empty on purpose
}

void enableEndSwitches() {
  // We need to attach to a dummy to capture an initial interrupt 
  // that my stil linger around
  for (int i = 18; i <= 21; i++) {
    attachInterrupt(digitalPinToInterrupt(i), ISR_dummy , CHANGE);
  }

  // Now use the real interrup service routines
  attachInterrupt(digitalPinToInterrupt(18), interruptOnX1 ,CHANGE);
  attachInterrupt(digitalPinToInterrupt(19), interruptOnX2, CHANGE);  
  attachInterrupt(digitalPinToInterrupt(20), interruptOnY2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(21), interruptOnY1, CHANGE);

  xHit = yHit = false;
  println("D Interrupts attached"); 
}

void disableXInterrupts() {
  detachInterrupt(digitalPinToInterrupt(18));
  detachInterrupt(digitalPinToInterrupt(19));
}

void disableYInterrupts() {
  detachInterrupt(digitalPinToInterrupt(20));
  detachInterrupt(digitalPinToInterrupt(21));
}

void enableXInterrupts(int mode) {
  attachInterrupt(digitalPinToInterrupt(19), interruptOnX2, mode);
  attachInterrupt(digitalPinToInterrupt(18), interruptOnX1, mode);
  
}

void enableYInterrupts(int mode) {
  attachInterrupt(digitalPinToInterrupt(20), interruptOnY2, mode);
  attachInterrupt(digitalPinToInterrupt(21), interruptOnY1, mode);

}

void printAllWorkItems(workItem *items) {
#ifdef DEBUG
  int count = 0;
  boolean done = false;
  do {
    workItem item = items[count];
    printWorkItem(item);
    count++;
    if (item.steps < 0 ) {
      done = true;
    }
  } while (!done);

#endif
}

void printWorkItem(workItem wItem) {
#ifdef DEBUG
  Serial.print("D workItem{steps=");
  Serial.print(wItem.steps, DEC);
  Serial.print(", x=");
  Serial.print(wItem.x, DEC);
  Serial.print(", y=");
  Serial.print(wItem.y, DEC);
  Serial.print(", task=");
  Serial.print(wItem.task);
  Serial.println("}");

  Serial.flush();
#endif
}
