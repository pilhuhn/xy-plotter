

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
  Serial.print("Parse Path input ");
  Serial.println(path);
  Serial.flush();
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

  // Now parse the segments and create work items
  curPos = 0;
  int itemCount = 0;
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
      if (token.startsWith("a") || token.startsWith("A") ) {
        handleArc(token);
        return; // we get this as single token, so we are done here.
      } else {
        parseToken(token, &workItems[itemCount]);
        itemCount++;
      }
      tokenCount++;
      curPos = pos + 1; // skip over |
    } while (pos > 0);
  }

  workItems[itemCount] = END_MARKER;

#ifdef DEBUG
  long t2 = millis();
  Serial.print("D Parsing took " );
  Serial.print(t2 - t1, DEC);
  Serial.println(" ms");
  Serial.flush();
#endif

}

/*
 * Parse a single token of the path and put it into the passed
 * workItem
 */
void parseToken(String token, workItem *wItem) {
  long x = 0;
  long y = 0;

  int pos ;
  int curPos = 0;

#ifdef DEBUG
  unsigned long t1 = micros();
#endif

  do {
    char v = token.charAt(curPos);
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
      Serial.print(F("E  unknown code "));
      Serial.println(v);
      Serial.flush();
    }
    curPos = pos + 1;
  } while (pos > 0);

  wItem->x = x * stepsPerMM;
  wItem->y = y * stepsPerMM;

  wItem->steps = max(abs(wItem->x), abs(wItem->y));

#ifdef DEBUG
  unsigned long t2 = micros();

  Serial.print(F("D     parseToken: (us) "));
  Serial.println(t2 - t1, DEC);
#endif

  if (verbose) {
    printWorkItem(*wItem);
  }
}

/*
 * Find up to 50 path segments and pass them to the 
 * path parser. 
 * The pointer into the input is then forwarded.
 */ 
String preParse() {

  int curr = pathPointer;
  int itemCount = 0;
  boolean aFound = false;
  
  for (unsigned int i = pathPointer; i < command.length(); i++) {
    if (command.charAt(i) == '|') {
      itemCount++;
    }
    if (command.charAt(i) == 'a' || command.charAt(i) == 'A') {
      // a/A is for arc/circles and creates its own workItems, so 
      // we need to stop here, let work be done, then do the a command
      // and then continue
      if (itemCount != 0) {
        aFound = true;        
      } else {
        int ePos = i;
        while (command.charAt(ePos) != '|' && ePos < command.length()) {
          ePos++;
        }
        String aCommand = command.substring(i,ePos);
        Serial.print("D Found A >>" );
        Serial.println(aCommand);
        Serial.flush();
        pathPointer = ePos +1 ;
        return aCommand;
      }
    }
    
    if (itemCount == 50) {
      Serial.print(F("D preParse: path pointer:"));
      Serial.println(i, DEC);
      Serial.flush();
      return command.substring(curr, i);
    }
    if (aFound) {
      Serial.print("aFound, returning ");      
      String tmp = command.substring(curr,i-1);
      pathPointer = i;
      Serial.println(tmp);
      Serial.flush();
      return tmp;

    }
  }
  Serial.print("D preParse: itemCount: ");
  Serial.println(itemCount, DEC);
  Serial.flush();
  // We did not hit 10, so return the whole string's end
  pathPointer = -1; // Mark end
  return command.substring(curr);
}

