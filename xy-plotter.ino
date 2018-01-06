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

// A work item is a single "relative" line, resolved into steps
struct workItem{
  int steps;  // total number of steps for this item
  int x;      // steps in x direction
  int y;      // steps in y direction
  int ox; // original x from command
  int oy; // original y from command
};

workItem *workItems;
int currentItem = 0;

String command;
int stepCount;
char dir;
int motor;
int stepsDone =0 ;
boolean done;
long e2, err;
int pathPointer;
boolean continuousMode=false;
int tokenCount;
boolean dryRun = false;

char dirPins[] = {4,7};
char stepPins[] = {3,6};


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

  digitalWrite(dirPin1, HIGH); 
  digitalWrite(enPin1, HIGH); // Disable stepper

  digitalWrite(dirPin2, HIGH); 
  digitalWrite(enPin2, HIGH); // Disable stepper

  Serial.println();
  Serial.flush();
  Serial.println("OK Setup done");
  Serial.flush();
}

int dx, dy;

// Does the work and is driven from the
// timer interrrupts. 
// This function is called each time the Time1
// fires.
void oneStep() {
  boolean doX, doY;

  workItem wItem= workItems[currentItem];
  
  int totalSteps = wItem.steps;
  if (totalSteps <=0 ) {
    Timer1.stop();
    delete[] workItems;
    done=true;
    return;
  }
  
  if (stepsDone == 0) {
    printWorkItem(wItem);
    setDirection(wItem);
    dx = abs(wItem.x);
    dy = -abs(wItem.y);
    err = dx+dy;
  }

  if (!dryRun) {
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
  }

  stepsDone++;
  if (stepsDone > abs(wItem.steps)) {
    currentItem++;
    stepsDone = 0;
  }
}


void startWork() {
  currentItem=0;
  stepsDone = 0;
  if(!dryRun) {
    enableMotors();
  }
  done = false;

  Serial.println("D Starting...");
  Serial.flush();

  if (dryRun) {
    Timer1.initialize(100); // No motor movement, so we can speed up
  } else {
    Timer1.initialize(500); // 500us
  }
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
    curPos = pos+1;
  } while (pos > 0);
  
  Serial.print("D Segments: ");
  Serial.println( segments, DEC);
  // allocate memory
  workItems = new workItem[segments +1];

  // Now parse the segments and create work items
  curPos = 0;
  int count = 0;
  if (segments>0) {
    do {
      String token;
      pos = path.indexOf('|', curPos);
      if (pos != -1) {
        token = sub.substring(curPos,pos);
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
      curPos = pos+1; // skip over |
    } while(pos >0);
  }

  workItems[count] = { -1, -1, 0};

#ifdef DEBUG  
  long t2 = millis();
  Serial.print("D Parsing took " );
  Serial.print(t2-t1, DEC);
  Serial.println(" ms");
  Serial.flush();
#endif  

}

void parseToken(String token, workItem *wItem) {
  int x = 0;
  int y = 0;

  int pos ;
  int curPos = 0;
  String sub = token;

#ifdef DEBUG
  unsigned long t1 = micros();
#endif  
  
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

  wItem->ox = x;
  wItem->oy = y;
  wItem->x = x * STEPS_PER_MM;
  wItem->y = y * STEPS_PER_MM;

  wItem->steps = max(abs(wItem->x), abs(wItem->y));

#ifdef DEBUG 
  unsigned long t2 = micros();
 
  Serial.print("D     parseToken: ");
  Serial.println(t2-t1, DEC);
#endif  
  
  printWorkItem(*wItem);
}

String findTen() {
  int curr = pathPointer;
  int count=0;
  for (unsigned int i = pathPointer; i < command.length(); i++) {
    if (command.charAt(i) == '|') {
      count++;
    }
    if (count==8) {
      Serial.print("D f10: pp: ");
      Serial.println(i, DEC);
      Serial.flush();
      pathPointer = i+1;
      return command.substring(curr,i);
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
  while(workItems[i].steps > 0) {
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
        done=true;
        continuousMode=false;
        dryRun=false;
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
          in.replace('\n','|');
          parsePath(in);
          startWork();
        }
          break;      
        case 'X':
        {
          // TEST to produce a square with 2cm edge length
          workItems = new workItem[5];
          workItems[0] = {3200,  3200, 0    } ;
          workItems[1] = {3200,     0, 3200 } ;
          workItems[2] = {3200, -3200, 0    } ;
          workItems[3] = {3200,     0, -3200} ;
          workItems[4] = {  -1, -1, -1      } ; // guard
          startWork();
          break;
        }
        case 'G':
          // Test for a path with square and triangles
        {
          String path; 
#ifdef DEBUG          
          dryRun = true;
          path = "X30";
          d(path);
          
          path = "X30|Y30";
          d(path);
          
          path = "X30|Y30|";
          d(path);
          
          path = "Y30|X30";
d(path);

          path = "Y30 X30";
d(path);

          path = "Y-30 X-30";
d(path);
          
          path = "X30|Y30|X-30 Y-30|X-20";
d(path);          
          
          path = "X30|Y30|X-30 Y-30|X-20|Y-20|X20|Y20|X-40|Y-25";
d(path);

          path = "X30|Y30|X-30 Y-30|X-20|Y-20|X20|Y20|X-40|Y-25|X40 Y25";
d(path);
          path = "X30|Y30|X-30 Y-30|X-20|Y-20|X20|Y20|X-40|Y-25|X40 Y25";
d(path);
          path = "X30|Y30|X-30 Y-30|X-20|Y-20|X20|Y20|X-40|Y-25|X40 Y25";
d(path);
          path = "X30|Y30|X-30 Y-30|X-20|Y-20|X20|Y20|X-40|Y-25|X40 Y25";
d(path);
#endif
          path = "X30|Y30|X-30 Y-30|X-20|Y-20|X20|Y20|X-40|Y-25|X40 Y25|X3|X-3|Y3|Y-3";
d(path);

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
        case 'C': {
          continuousMode = true;
          Serial.print("OK Continuous mode is on");
          if (command.length() >1) {
            dryRun = true;
            Serial.print(", dryRun is on ");
          }
          Serial.println();
          Serial.flush();
          break;
        }
      } // switch
    } // if (!continuous mode)
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
      if (!continuousMode) {
        Serial.println("D DONE, disabling motors");
        disableMotors();
      }
      Serial.println("OK");
      Serial.flush();
    }
  }
}
