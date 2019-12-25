/*
   3200 steps = 2cm
   1600 steps = 1cm
   160 steps  = 1mm
*/
#define DEFAULT_STEPS_PER_MM 160

#define DEBUG

#include "TimerOne.h"
#include "helpers.h"

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
#define SERVO_PIN PIN_A1
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


// A work item is a single "relative" line, resolved into steps
struct workItem {
  long steps;  // total number of steps for this item
  long x;      // steps in x direction
  long y;      // steps in y direction
  int ox; // original x from command
  int oy; // original y from command
  byte task;
};

workItem *workItems;
int currentItem = 0;

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

char dirPins[] = {7, 13};
char stepPins[] = {6, 12};

void setup() {

  Serial.begin(115200);

  pinMode(enPin1, OUTPUT);
  pinMode(stepPin1, OUTPUT);
  pinMode(dirPin1, OUTPUT);
  digitalWrite(enPin1, LOW);

  pinMode(enPin2, OUTPUT);
  pinMode(stepPin2, OUTPUT);
  pinMode(dirPin2, OUTPUT);
  digitalWrite(enPin2, LOW);

  setupZAxis();

  digitalWrite(dirPin1, HIGH);
  digitalWrite(enPin1, HIGH); // Disable stepper

  digitalWrite(dirPin2, HIGH);
  digitalWrite(enPin2, HIGH); // Disable stepper


  // Interrupt if the pin goes to GND
  for (int i = 18 ; i <= 21 ; i++) {
    pinMode(i, INPUT_PULLUP);
  }

  // TODO move Panic button to other pin
    pinMode(2, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(2), panic, CHANGE);


  // Attach to servo and raise pen
  // TODO rewrite with z-Stepper
  //  pinMode(SERVO_PIN, OUTPUT);
  //  servoMove(SERVO_PIN, PEN_UP);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.println();
  Serial.flush();
  Serial.println("OK Setup done");
  Serial.flush();


}

void panic() {
  Timer1.stop();
  println("STOPPED");
  done = true;
  xHit = true;
  yHit = true;
}

long dx, dy;

// Does the work and is driven from the
// timer interrrupts.
// This function is called each time the Time1
// fires.
void oneStep() {
  boolean doX, doY;

  if (xHit || yHit)  {
    println("One step, hit a switch");
    servoMove(SERVO_PIN, PEN_UP);
    Timer1.stop();
    Timer1.detachInterrupt();
    delete[] workItems;
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

  // This must be TASK_MOVE
  long totalSteps = wItem.steps;
  if (totalSteps <= 0 ) {
    Timer1.stop();
    Timer1.detachInterrupt();
    delete[] workItems;
    done = true;
    return;
  }

  if (stepsDone == 0) {
    printWorkItem(wItem);
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





/* Parse a path that is in the form of
    Xnn
    Ynn
    Xnn Ynn
    where items are separated by newline.
    nn is a relative distance in millimeter
    E.g. the following to build a triangle
   "X50\nY50\nX-50 Y-50";
*/
void parsePath(String path) {

#ifdef DEBUG
  long t1 = millis();
#endif
  path.trim();
  String sub = path;

  int segments = 0;
  int curPos = 0;
  int pos;

  // count segments separated by |
  do {
    pos = path.indexOf('|', curPos);
    segments++;
    curPos = pos + 1;
  } while (pos > 0);

  Serial.print("D Segments: ");
  Serial.println( segments, DEC);
  // allocate memory
  workItems = new workItem[segments + 1];

  // Now parse the segments and create work items
  curPos = 0;
  int count = 0;
  if (segments > 0) {
    do {
      String token;
      pos = path.indexOf('|', curPos);
      if (pos != -1) {
        token = sub.substring(curPos, pos);
      } else {
        token = sub.substring(curPos);
      }
#ifdef DEBUG
      Serial.print("D Found token >>");
      Serial.print( token.c_str());
      Serial.println("<<");
      Serial.flush();
#endif
      parseToken(token, &workItems[count++]);
      tokenCount++;
      curPos = pos + 1; // skip over |
    } while (pos > 0);
  }

  workItems[count] = { -1, -1, 0 , 0, 0, TASK_MOVE};

#ifdef DEBUG
  long t2 = millis();
  Serial.print("D Parsing took " );
  Serial.print(t2 - t1, DEC);
  Serial.println(" ms");
  Serial.flush();
#endif

}

void parseToken(String token, workItem *wItem) {
  long x = 0;
  long y = 0;

  int pos ;
  int curPos = 0;
  String sub = token;

#ifdef DEBUG
  unsigned long t1 = micros();
#endif

  do {
    char v = sub.charAt(curPos);
    curPos++;
    long val;
    pos = token.indexOf(' ', curPos);
    if (pos > 0) {
      val = token.substring(curPos, pos).toInt();
    } else {
      val = token.substring(curPos).toInt();
    }
    if (v == 'X') {
      x = val;
      wItem->task = TASK_MOVE;
    } else if (v == 'Y') {
      y = val;
      wItem->task = TASK_MOVE;
    } else if (v == 'U') {
      wItem->task = TASK_PEN_UP;
    } else if (v == 'D') {
      wItem->task = TASK_PEN_DOWN;
    } else {
      Serial.print("E  unknown code ");
      Serial.println(v);
      Serial.flush();
    }
    curPos = pos + 1;
  } while (pos > 0);

  wItem->ox = x;
  wItem->oy = y;
  wItem->x = x * stepsPerMM;
  wItem->y = y * stepsPerMM;

  wItem->steps = max(abs(wItem->x), abs(wItem->y));

#ifdef DEBUG
  unsigned long t2 = micros();

  Serial.print("D     parseToken: ");
  Serial.println(t2 - t1, DEC);
#endif

  printWorkItem(*wItem);
}

String findTen() {
  int curr = pathPointer;
  int count = 0;
  for (unsigned int i = pathPointer; i < command.length(); i++) {
    if (command.charAt(i) == '|') {
      count++;
    }
    if (count == 10) {
      Serial.print("D f10: pp: ");
      Serial.println(i, DEC);
      Serial.flush();
      pathPointer = i + 1;
      return command.substring(curr, i);
    }
  }
  Serial.print("D f10: count: ");
  Serial.println(count, DEC);
  Serial.flush();
  // We did not hit 10, so return the whole string's end
  pathPointer = -1; // Mark end
  return command.substring(curr);
}

#ifdef DEBUG
void d(String path) {
  Serial.println(" ---> " + path);
  Serial.flush();
  parsePath(path);
  //  startWork();
  int i = 0;
  while (workItems[i].steps > 0) {
    printWorkItem(workItems[i]);
    i++;
  }
  delete[] workItems;
  Serial.println("---------------------");
  Serial.flush();
}
#else
void d(String path)
{
}
#endif

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
        String path = findTen();
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
            parsePath(in);
            startWork();
          }
          break;
        case 'X':
          {
            // TEST to produce a square with 2cm edge length
            workItems = new workItem[5];
            workItems[0] = {3200,  3200, 0     , 0, 0, TASK_MOVE} ;
            workItems[1] = {3200,     0, 3200  , 0, 0, TASK_MOVE} ;
            workItems[2] = {3200, -3200, 0     , 0, 0, TASK_MOVE} ;
            workItems[3] = {3200,     0, -3200 , 0, 0, TASK_MOVE} ;
            workItems[4] = {  -1, -1, -1       , 0, 0, TASK_MOVE} ; // guard
            startWork();
            break;
          }
        case 'Q':
          // Emergency shutdown for manual operation
          panic();
          break;
        case 'T': {
            // Split the path in smaller items and handle them one at a time
            command.replace('\n', '|');
            pathPointer = 1; // comand[0] is 'T, path starts at 1
            String path = findTen();
            Serial.println(path);
            parsePath(path);
            startWork();
          }
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
            //          penServo.write(val);
            servoMove(SERVO_PIN, val);
            break;
          }
        case 'i':
          enableEndSwitches();
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
      String path = findTen();
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
