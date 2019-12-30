

/* Parse a path that is in the form of
    Xnn
    Ynn
    Xnn Ynn
    where items are separated by pipe ( | ).
    nn is a relative distance in millimeter
    E.g. the following to build a triangle
   "X50|Y50|X-50 Y-50";
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
