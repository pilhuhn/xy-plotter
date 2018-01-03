void enableMotors() {
  digitalWrite(enPin1, LOW);
  digitalWrite(enPin2, LOW);
  delayMicroseconds(100);
}

void disableMotors() {
  digitalWrite(enPin1, HIGH);
  digitalWrite(enPin2, HIGH);
}


