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

void servoMove(int servo, int pos){
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
}

void printWorkItem(workItem wItem) {
#ifdef DEBUG  
  Serial.print("D steps=");
  Serial.print(wItem.steps, DEC);
  Serial.print(", x=");
  Serial.print(wItem.x, DEC);
  Serial.print(", y=");
  Serial.print(wItem.y, DEC);
  Serial.print(", ox=");
  Serial.print(wItem.ox, DEC);
  Serial.print(", oy=");
  Serial.println(wItem.oy, DEC);

  Serial.flush();
#endif  
}
