/*
   3200 steps = 2cm
   1600 steps = 1cm
   160 steps  = 1mm
*/
#define DEFAULT_STEPS_PER_MM 160

#define DEBUG

#include "TimerOne.h"
#include "helpers.h"
#include "sin_table.h"
#include "workitem.h"

// Stepper 1 = z-axis = Motor 1 on shield
// See zAxis.ino

// Stepper 2 = x-axis = Motor 2 on shield
#define enPin1 5
#define stepPin1 6
#define dirPin1 7

// Stepper 3 = y-axis = motor 3 on shield
#define enPin2 11
#define stepPin2 12
#define dirPin2 13

// Servo
#define SERVO_PIN PIN_A1 // Logical pin , Servo port on motor 4 of Fabscan shield, pin 55 on mega
#define PEN_UP 10
#define PEN_DOWN 70

// LED pin
#define LED_PIN PIN_A3

// Tasks
#define TASK_MOVE 1
#define TASK_PEN_UP 2
#define TASK_PEN_DOWN 3

// bit mask is   ....YYXX where x and y have swapped
// bits, as the direction setting is different for them.
#define X_LEFT_HIT 1
#define X_RIGHT_HIT 2
#define Y_RIGHT_HIT 4
#define Y_LEFT_HIT 8


workItem *workItems = new workItem[361];
int currentItem = 0;

#define END_MARKER { -1, -1, -1 , TASK_MOVE}

int stepsPerMM = DEFAULT_STEPS_PER_MM;
String command;
long stepCount;
int motor;
long stepsDone = 0 ;
boolean done;
long e2, err;
int pathPointer;
boolean continuousMode = false;
boolean compensating = false;
boolean homing = false;
int tokenCount;
args a; // for z-Axis
boolean dryRun = false;
volatile boolean xHit = false;
volatile boolean yHit = false;
volatile unsigned char hitmask = 0x0;
volatile char hitMsg = '\0';
boolean verbose = true;

char dirPins[] = {7, 13};
char stepPins[] = {6, 12};

/*
 * One time setup routine called from the Arduino framework
 */ 
void setup() {

  Serial.begin(115200);

  // setup Motor 2 (x-axis)
  pinMode(enPin1, OUTPUT);
  pinMode(stepPin1, OUTPUT);
  pinMode(dirPin1, OUTPUT);
  digitalWrite(enPin1, LOW);

  // setup Motor 3 (z-axis)
  pinMode(enPin2, OUTPUT);
  pinMode(stepPin2, OUTPUT);
  pinMode(dirPin2, OUTPUT);
  digitalWrite(enPin2, LOW);

//  setupZAxis();

  digitalWrite(dirPin1, HIGH);
  digitalWrite(enPin1, HIGH); // Disable stepper

  digitalWrite(dirPin2, HIGH);
  digitalWrite(enPin2, HIGH); // Disable stepper


  // Interrupt if the pin goes to GND
  for (int i = 18 ; i <= 21 ; i++) {
    pinMode(i, INPUT_PULLUP);
  }

  // TODO move Panic button to other pin as it clashes
  // with stepper motor 1 (z-Axis)
    pinMode(2, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(2), panic, CHANGE);


  // Attach to servo and raise pen
  // TODO rewrite with z-Stepper
  pinMode(SERVO_PIN, OUTPUT);
  servoMove(SERVO_PIN, PEN_UP);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.println();
  Serial.flush();
  Serial.println("OK Setup done");
  Serial.flush();


}

/*
 * Interrupt callback for the 'panic' button.
 * Stops the activity of the motors
 */ 
void panic() {
  Timer1.stop();
  println("STOPPED");
  done = true;
  xHit = true;
  yHit = true;
}

long dx, dy;

/*
 * Does the work and is driven from the
 * timer interrrupts.
 * This function is called each time the Timer1
 * fires.
 * See #startWork() which initialises motors+interrupts.
 */ 
void oneStep() {
  boolean doX, doY;

  if (xHit || yHit)  {
    println(F("D One step, hit a switch"));
    servoMove(SERVO_PIN, PEN_UP);
    Timer1.stop();
    Timer1.detachInterrupt();
    // delete[] workItems;
    done = true;
    return;
  }

  workItem wItem = workItems[currentItem];

  if (wItem.task == TASK_PEN_UP) {
    servoMove(SERVO_PIN, PEN_UP);
    currentItem++;
    return;
  }
  if (wItem.task == TASK_PEN_DOWN) {
    servoMove(SERVO_PIN, PEN_DOWN);
    currentItem++;
    return;
  }

  // From here on:  TASK_MOVE

  // wItem.steps <= 0 means END_MARKER, so we can stop with this
  // set of work items
  if (wItem.steps <= 0 ) {
    Timer1.stop();
    Timer1.detachInterrupt();
    done = true;
    return;
  }

  // No end maker? Then let's move
  if (stepsDone == 0) {
    if (verbose) {
      printWorkItem(wItem);
    }
    setDirection(wItem);
    dx = abs(wItem.x);
    dy = -abs(wItem.y);
    err = dx + dy;
  }

  if (!dryRun) {
    // Bessenham algorithm from wikipedia
    e2 = 2 * err;
    if (e2 > dy && !xHit) {
      digitalWrite(stepPins[0], HIGH);
      err += dy;
      doX = true;
    }
    if (e2 < dx && !yHit) {
      digitalWrite(stepPins[1], HIGH);
      err += dx;
      doY = true;
    }
    delayMicroseconds(30);
    if (doX) {
      digitalWrite(stepPins[0], LOW);
    }
    if (doY) {
      digitalWrite(stepPins[1], LOW);
    }
  }

  stepsDone++;
  if (stepsDone > abs(wItem.steps)) {
    currentItem++;
    stepsDone = 0;
  }
}

void interruptOnX1() {
  disableXInterrupts();
  xHit = true;
  hitMsg = 'A';
  hitmask |= X_LEFT_HIT;
}

void interruptOnX2() {
  disableXInterrupts();
  xHit = true;
  hitMsg = 'B';
  hitmask |= X_RIGHT_HIT;
}

void interruptOnY1() {
  disableYInterrupts();
  yHit = true;
  hitMsg = 'C';
  hitmask |= Y_LEFT_HIT;
}

void interruptOnY2() {
  disableYInterrupts();
  yHit = true;
  hitMsg = ('D');
  hitmask |= Y_RIGHT_HIT;
}

void startWork() {
  currentItem = 0;
  xHit = yHit = false;
  stepsDone = 0;
  if (!dryRun) {
    enableMotors();
    enableEndSwitches();
  }
  done = false;

  Serial.println("D Starting...");
  Serial.flush();

  // Start interrupts. Values are in micro-seconds
  if (dryRun) {
    Timer1.initialize(100); // No motor movement, so we can speed up
  } else {
    Timer1.initialize(200);
  }
  Timer1.attachInterrupt(oneStep);
}



/*
 * Main loop as defined by Arduino. This
 * basically reads input commands over serial
 * and executes them in a row.
 */ 
void loop() {

  if (Serial.available() > 0) {
    command = Serial.readStringUntil('\n');
    Serial.print("D ");
    Serial.println(command.c_str());
    Serial.flush();

    if (continuousMode) {
      // we read line by line from serial input and feed it to parser+worker
      // until the line starts with -END
      if (command.startsWith("-END")) {
        done = true;
        continuousMode = false;
        dryRun = false;
        Serial.println("D END found, continuous mode off");
        Serial.print("D Tokens processed: ");
        Serial.println(tokenCount, DEC);
        Serial.flush();
      }
      else {
        pathPointer = 0;
        String path = preParse();
        parsePath(path);
        startWork();
      }
    }
    else {
      // we get commands on the command line - possibly with parameters
      char c = command.charAt(0);
      switch (c) {
        case 'E':
          enableMotors();
          break;
        case 'D':
          disableMotors();
          break;
        case 's':
          {
            String in = command.substring(1);
            in.replace('\n', '|');
            pathPointer = 1; // comand[0] is 's', path starts at 1
            String path = preParse();
            parsePath(path);
            startWork();
          }
          break;
        case 'Q':
          // Emergency shutdown for manual operation
          panic();
          break;
        case 'C': {
            continuousMode = true;
            Serial.print("OK Continuous mode is on");
            if (command.length() > 2) {
              dryRun = true;
              Serial.print(", dryRun is on ");
            }
            Serial.println();
            Serial.flush();
            break;
          }
        case 'H': {
            // Home, requires interrupts and end-switches
            println("D Homing");
            homing = true;
            String path = "X-300 Y-300";
            parsePath(path);
            startWork();
          }
          break;
        case 'u': {
            // Pen up
            //          penServo.write(PEN_UP);
            servoMove(SERVO_PIN, PEN_UP);
            Serial.println("OK pen up");
            Serial.flush();
          }
          break;
        case 'd': {
            // Pen down
            //          penServo.write(PEN_DOWN);
            servoMove(SERVO_PIN, PEN_DOWN);
            Serial.println("OK pen down");
            Serial.flush();

          }
          break;
        case 'v': {
            int val = command.substring(1).toInt();
            servoMove(SERVO_PIN, val);
            break;
          }
        case 'i':
          enableEndSwitches();
          break;
        case 'V':
          verbose = !verbose;
          Serial.print(F("Verbose is "));
          Serial.println(verbose ? "on" : "off");
          Serial.flush();
          break;
        case 'I': {
            disableXInterrupts();
            disableYInterrupts();
            Serial.println("Interrupts detached");
            Serial.flush();
            break;
          }

        case 'r': { // resolution
            int val = command.substring(1).toInt();
            int tmp = stepsPerMM;
            stepsPerMM = val;
            Serial.print("OK set steps perMM to ");
            Serial.print(val);
            Serial.print(" previous value was ");
            Serial.println(tmp);
            Serial.flush();
            break;
          }
        case 'z': { 
          args arg = optarg(command);
          handleZ(arg);
          break;
          }
        case 'a':
          // Fall through intentional
        case 'A': 
          handleArc(c,command);
          break;
        default: {
            Serial.print("Unknown command >>");
            Serial.print(c);
            Serial.println("<<");
            Serial.flush();
          }
      } // switch
    } // if (!continuous mode)
  } // if serial

  if (done) {
    done = false;
    boolean oldX = xHit;
    boolean oldY = yHit;
    println("D .. done reached ..");

    if (xHit || yHit) {

      // determine which was hit and then move into the
      // opposite direction until we hit again
      println("D.. x or y hit, compensating");
      delay(5);
      if (xHit) {
        xHit = false;
        compensate_move(0);
      }
      if (yHit) {
        yHit = false;
        compensate_move(1);
      }
      println("D.. compensating done");
      pathPointer = 0; // Ignore remainder of input

      if (homing) {
        delay(5);
        println("Homing now");
        if (oldX) {
          println("  on Y");
          parsePath("Y-200");
          startWork();
        }
        if (oldY) {
          println("  on X ");
          parsePath("X-200");
          startWork();
        }
        homing = false;
      }

    } // xHit || yHit


    // Works with the T or C command where the
    // command string is very long, but we can only
    // do so much at a time.
    if (pathPointer > 1) {
      String path = preParse();
      Serial.println(path);
      parsePath(path);
      startWork();
    }
    else {
      if (!continuousMode) {
        Serial.println("D DONE, disabling motors");
        disableMotors();
      }
      Serial.println("OK");
      Serial.flush();
    }
  } // if done

  if (xHit || yHit) {  // TODO
    if (hitMsg != '\0') {
      Serial.print("D ");
      Serial.println(hitMsg);
      Serial.flush();
      hitMsg = '\0';
    }
  }
} // loop()

void compensate_move(int motor) {
  Serial.print("D motor = ");
  Serial.print(motor, DEC);
  Serial.print(" mask = ");
  Serial.print(hitmask, BIN);
  println(" Start crawling");

  unsigned char mask = hitmask;
  if (motor == 1) {
    mask >>= 2;
  }
  mask &= 3;

  if (mask & 1) {
    digitalWrite(dirPins[motor], HIGH);
  }
  else {
    digitalWrite(dirPins[motor], LOW);
  }
  delayMicroseconds(150);

  if (motor == 1) {
    enableYInterrupts(CHANGE);
  } else {
    enableXInterrupts(CHANGE);
  }
  delay(1);

  // Now we move until the switch is released, which is signalled by an interrupt.
  while (!xHit && !yHit) {
    digitalWrite(stepPins[motor], HIGH);
    delayMicroseconds(30);
    digitalWrite(stepPins[motor], LOW);
    delayMicroseconds(150);
  }
  println("D End crawling");
}

args optarg (String text) {
  
  a.code = text.charAt(1);
  a.value = text.substring(2).toInt();

  return a;
}

