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
    delayMicroseconds(100);
  }
}

void printWorkItem(workItem wItem) {
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
}
