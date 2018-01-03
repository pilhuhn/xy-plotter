/*
 * 3200 steps = 4cm
 * 1600 steps = 2cm
 * 160 steps = 2mm
 * 80 steps = 1mm
 */
#define STEPS_PER_MM 80

#include "TimerOne.h"

// Stepper 1 = x-axis
#define enPin1 2
#define stepPin1 3
#define dirPin1 4

// Stepper 2 = y-axis
#define enPin2 5
#define stepPin2 6
#define dirPin2 7

#define MS_PIN PIN_A5

struct work {
  int steps;  // total number of steps for this item
  int x;      // steps in x direction
  int y;      // steps in y direction
} workItem;

work *workItems;
int currentItem = 0;

String command;
int stepCount;
char dir;
int motor;
int stepsDone =0 ;
boolean done;
long e2, err;
int pathPointer;

char dirPins[] = {4,7};
char stepPins[] = {3,6};

void printWorkItem(work workItem) {
  Serial.print("steps=");
  Serial.print(workItem.steps, DEC);
  Serial.print(", x=");
  Serial.print(workItem.x, DEC);
  Serial.print(", y=");
  Serial.println( workItem.y, DEC);
  Serial.flush();
}

void setup() {

  Serial.begin(115200);

  pinMode(MS_PIN, OUTPUT);


  pinMode(enPin1, OUTPUT);
  pinMode(stepPin1, OUTPUT);
  pinMode(dirPin1, OUTPUT);
  digitalWrite(enPin1, LOW);


  pinMode(enPin2, OUTPUT);
  pinMode(stepPin2, OUTPUT);
  pinMode(dirPin2, OUTPUT);
  digitalWrite(enPin2, LOW);

  digitalWrite(dirPin1, HIGH); 
  digitalWrite(enPin1, HIGH); // Disable stepper

  digitalWrite(dirPin2, HIGH); 
  digitalWrite(enPin2, HIGH); // Disable stepper

  Serial.println("Setup done");
}

int dx, dy;

void oneStep() {
  boolean doX, doY;

  work workItem = workItems[currentItem];
  
  int totalSteps = workItem.steps;
  if (totalSteps <=0 ) {
    Timer1.stop();
    delete[] workItems;
    done=true;
    return;
  }
  if (stepsDone == 0) {
    printWorkItem(workItem);
    setDirection(workItem);
    dx = abs(workItem.x);
    dy = -abs(workItem.y);
    err = dx+dy;
  }

  // Bessenham algorithm from wikipedia
  e2 = 2*err;
  if (e2 > dy) {
    digitalWrite(stepPins[0],HIGH);
    err += dy;
    doX=true;
  }
  if (e2 < dx) {
    digitalWrite(stepPins[1],HIGH);
    err += dx;
    doY=true;
  }
  delayMicroseconds(30);
  if (doX) {
    digitalWrite(stepPins[0],LOW);
  }
  if (doY) {
    digitalWrite(stepPins[1],LOW);
  }

  stepsDone++;
  if (stepsDone > abs(workItem.steps)) {
    currentItem++;
    stepsDone = 0;
  }
}

void setDirection(work item) {
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

void startWork() {
  currentItem=0;
  stepsDone = 0;
  enableMotors();
  done = false;

  Serial.println("Starting...");
  Serial.flush();

  Timer1.initialize(500); // 500us
  Timer1.attachInterrupt(oneStep);
}





/* Parse a path that is in the form of
 *  Xnn
 *  Ynn
 *  Xnn Ynn
 *  where items are separated by newline.
 *  nn is a relative distance in millimeter
 *  E.g. the following to build a triangle
 * "X50\nY50\nX-50 Y-50"; 
 */
void parsePath(String path) {
  long t1 = millis();
  path.trim();
  String sub = path;

  int segments = 0;
  int curPos = 0;
  int pos;

  // count segments separated by |
  do {
    pos = path.indexOf('|', curPos);
    segments++;
    curPos = pos+1;
  } while (pos > 0);
  
  Serial.print("Segments: ");
  Serial.println( segments, DEC);
  // allocate memory
  workItems = new work[segments +1 ];

  // Now parse the segments and create work items
  curPos = 0;
  int count = 0;
  do {
    pos = path.indexOf('|', curPos);
    String token = sub.substring(curPos,pos);
    Serial.print("Found token ");
    Serial.println( token.c_str());
    Serial.flush();
    work *workItem = parseToken(token);
    workItems[count++]=*workItem;
    curPos = pos+1;
  } while(pos >0);

  workItems[count] = { -1, -1, 0};
  long t2 = millis();
  Serial.print("Parsing took " );
  Serial.print(t2-t1, DEC);
  Serial.println(" ms");
  Serial.flush();
}

work *parseToken(String token) {
  int x = 0;
  int y = 0;

  int pos ;
  int curPos = 0;
  String sub = token;
  do {
    char v = sub.charAt(curPos);
    curPos++;
    int val;
    pos = token.indexOf(' ',curPos);
    if (pos > 0) {
      val = token.substring(curPos,pos).toInt();
    } else {
      val = token.substring(curPos).toInt();
    }
    if (v == 'X') {
      x = val;
    } else {
      y = val;
    }
    curPos = pos+1;
  } while(pos>0);

  work *workItem = new work;
  workItem->x = x * STEPS_PER_MM;
  workItem->y = y * STEPS_PER_MM;

  workItem->steps = max(abs(workItem->x), abs(workItem->y));

//  printWorkItem(*workItem);
  return workItem;
}

String findTen() {
  int curr = pathPointer;
  int count=0;
  for (int i = pathPointer; i < command.length(); i++) {
    if (command.charAt(i) == '|') {
      count++;
    }
    if (count==5) {
      Serial.print("f10: pp: ");
      Serial.println(i, DEC);
      pathPointer = i+1;
      return command.substring(curr,i);
    }
  }
  Serial.print("f10: count: ");
  Serial.println(count, DEC);
  // We did not hit 10, so return the whole string's end
  pathPointer = -1; // Mark end
  return command.substring(curr);
}

void loop() {

  if (Serial.available() > 0) {
    command = Serial.readStringUntil('\n');
    Serial.println(command.c_str());    
    Serial.flush();
//    if (command.length() < 1) {
//      continue;
//    }
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
        in.replace('\n','|');
        parsePath(in);
        startWork();
      }
        break;      
      case 'X':
        // TEST to produce a square with 2cm edge length
        workItems = new work [5];
        workItems[0] = {3200,  3200, 0    } ;
        workItems[1] = {3200,     0, 3200 } ;
        workItems[2] = {3200, -3200, 0    } ;
        workItems[3] = {3200,     0, -3200} ;
        workItems[4] = {  -1, -1, -1      } ; // guard
        startWork();
        break;

      case 'G':
        // Test for a path with square and triangles
      {
        String path = "X30|Y30|X-30 Y-30|X-20|Y-20|X20|Y20|X-40|Y-25|X40 Y25";
        
        parsePath(path);
        
        startWork();
      }
      break; 
      case 'Q':
        // Emergency shutdown
        Timer1.stop();
        Serial.println("STOPPED");
        done=true;
      break;  
      case 'T': {
        // Split the path in smaller items and handle them one at a time
        command.replace('\n','|');
        pathPointer = 1; // comand[0] is 'T, path starts at 1
        String path = findTen();
        Serial.println(path);
        parsePath(path);
        startWork();
        
      }
      break;
        
    } // switch
  } // if serial

  if (done) {
    done=false;

    // Works with the T command where the 
    // command string is very long, but we can only
    // do so much at a time.
    if (pathPointer > 1) {
      String path = findTen();
      Serial.println(path);
      parsePath(path);
      startWork();
    }
    else {
    
      Serial.println("DONE, disabling motors");
      Serial.flush();
      disableMotors();
    }
  }
}
