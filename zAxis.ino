
#include <TMC2130Stepper.h>
#include "helpers.h"

// Stepper 1 = z-axis = Motor 1 on shield
#define enPinZ 2
#define stepPinZ 3
#define dirPinZ 4

#define CS_PIN 48
#define MOSI_PIN 51 //SDI/MOSI (ICSP: 4, Uno: 11, Mega: 51)
#define MISO_PIN 50 //SDO/MISO (ICSP: 1, Uno: 12, Mega: 50)
#define SCK_PIN  52 //CLK/SCK  (ICSP: 3, Uno: 13, Mega: 52)



TMC2130Stepper TMC = TMC2130Stepper(enPinZ, dirPinZ, stepPinZ, CS_PIN);

long pauseMS = 60; // pause between steps

void setupZAxis() {
  //  pinMode(enPinZ, OUTPUT);
  //  pinMode(stepPinZ, OUTPUT);
  //  pinMode(dirPinZ, OUTPUT);
  //  digitalWrite(enPinZ, LOW);

  TMC.begin();
//  TMC.hysteresis_start(4);
//  TMC.hysteresis_end(-2);
  TMC.SilentStepStick2130(500);
  TMC.microsteps(16);
  TMC.interpolate(1);
//  TMC.stealthChop(1);  // Silent stepping, not compatible with stall guard (?)
  TMC.diag0_stall(1);
  TMC.diag1_stall(1);
  TMC.diag1_active_high(1);
  
  TMC.coolstep_min_speed(25000);
//  TMC.THIGH(0);
//  TMC.semin(5);
//  TMC.semax(2);
//  TMC.sedn(0b01);
//  TMC.sg_min(0); // Needed to parametrize
  TMC.sg_stall_value(14);
//  TMC.sg_filter(0);

  Serial.println(TMC.stealthChop());
  Serial.println(TMC.shaft_dir());
  Serial.println(TMC.microsteps());
  Serial.println(TMC.getCurrent());

  //  digitalWrite(dirPinZ, HIGH);
  digitalWrite(enPinZ, HIGH); // Disable stepper

}

void handleZ(args arg) {
  switch (arg.code) {
    case '+':
      zStep(HIGH, arg.value);
      break;
    case '-':
      zStep(LOW, arg.value);
      break;
    case 'v':
      TMC.sg_stall_value(arg.value);
      break;
    case 'p':
      pauseMS = arg.value;
      break;
    case 'f':
      TMC.sg_filter( arg.value );
      break;
    case 's':
      TMC.microsteps(arg.value);
      break;
    case 'e':
      enableZMotor();
      break;
    case 'd':
      disableZMotor();
      break;    
    case '?':
    default:
      Serial.println("- =up, + =down, v =stall_value, p =pauseMS, f =sg_filter, v = stall_value, s = microsteps, e = enable, d = disable");
      Serial.print("stall_value= ");
      Serial.print(TMC.sg_stall_value());
      Serial.print(", pauseMS= ");
      Serial.print(pauseMS);
      Serial.print( ", filter= ");
      Serial.print(TMC.sg_filter());
      Serial.print(", microsteps= ");
      Serial.print(TMC.microsteps());
      Serial.println();
      break;
  }
}

void enableZMotor() {
  digitalWrite(enPinZ, LOW);
  delayMicroseconds(20);
  Serial.println(TMC.isEnabled());
}

void disableZMotor() {
  digitalWrite(enPinZ, HIGH);
}

void zStep(char dir, long steps) {

  int code ;
//  enableZMotor();
  TMC.shaft_dir(dir);
  for (long i = 0; i < steps ; i++) {  // 800Steps = 1mm
    for (int j = 0; j < 1 ; j++) {
      delayMicroseconds(pauseMS);
      digitalWrite(stepPinZ, HIGH);
      delayMicroseconds(30);
      digitalWrite(stepPinZ, LOW);
      code = TMC.sg_result();
      Serial.println(code);
      if (code == 0) {
        Serial.print("!!! stalled +++  j=");
        Serial.println(j);
        j = 200;
        i = steps;
        break;
      }

    }
    Serial.println(TMC.sg_result());
    if (TMC.sg_result() == 0) {
      Serial.print("+++ stalled +++  i=");
      Serial.println(i);
      i = steps;
      break;
    }
  }
//  disableZMotor();

}

