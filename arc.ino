
#include "workitem.h"

void handleArc(String command) {
    return handleArc(command.charAt(0),command);
}


/*
 * Deal with arcs/circles
*/
void handleArc(char c, String command)
{
    boolean moveTo = false;
    boolean oldPenStateIsUp = penIsUp; // remember old pen position for 'A' mode.

    if (c == 'A') {
        moveTo = true;
    }

    // arc radius[,start,end]  start,end in degree (0-360)
    int idx = command.indexOf(',',1); // Check for 1st ,
    int radius;
    int start = 0;
    int end= 360;
    if (idx == -1) {
        radius = command.substring(1).toInt();
    } else {
        radius = command.substring(1,idx).toInt();
    
    int idx2 = command.indexOf(',',idx+1);
    if (idx2 == -1) {
        Serial.println(F("E start and/or end degrees missing"));
        return;
    } else {
        start = command.substring(idx+1,idx2).toInt();
        end = command.substring(idx2+1).toInt();
    }
    }
    // Normalize to 0..360 deg
    start = start %360;
    if (end>360) {
        end = end %360;  // TODO full circle needs the last step, that gets removed here
    }   
    Serial.print("D arc, radius= ");
    Serial.print(radius);
    Serial.print(", start= ");
    Serial.print(start);
    Serial.print(", end= ");
    Serial.println(end);

    signed long ox, oy;
    signed long save_x, save_y;
    int count =0;
    int stepsToExecute = 0;
    short dir = (start < end ) ? 1 : -1 ;
    int deg = start;
    
    //  with 'A' reserve for penup/down + initial move
    stepsToExecute=abs(end-start) + (moveTo ? 4 :1 ); 
    Serial.print(F("D arc, steps to execute "));
    Serial.print(stepsToExecute, DEC);
    Serial.print(F(" direction="));
    Serial.print(dir, DEC);
    Serial.print(F(", startDeg ="));
    Serial.println(deg, DEC);
    
    // We start the arc at the current position
    // TODO: decide (via param) if previous point is middle point
    //    or start point

    // Loop. Run variable is the number of the current step.
    for (int i = 0; i < stepsToExecute ; i++) { 

        // determine x and y depending on quadrant
        signed long msin, mcos;
        switch (deg / 90) {        
        case 0: // x>0, y>0
            mcos = my_cos (deg);
            msin = my_sin (deg);
            break;
        case 1: // x<0, y>0
            mcos = -my_cos (-deg + 180);
            msin = my_sin (-deg + 180);              
            break;
        case 2: // x<0,y<0
            msin = -my_sin ( deg % 90);
            mcos = -my_cos ( deg % 90);
            break;
        case 3: // x>0, y<0
            msin = -my_sin (90 - (deg % 90));
            mcos = my_cos (90 - (deg % 90));              
            break;              
        default:
            Serial.print(F("E rror, unknwon quadrant "));
            Serial.print(deg/90);
            Serial.print(" for " );
            Serial.println(deg);
            Serial.flush();
            break;
        }
        
#ifdef DEBUG            
    if (verbose) {
            Serial.flush();
            Serial.println();
            Serial.print("D     deg= ");
            Serial.print(deg, DEC);
            Serial.print("  deg/90= ");
            Serial.print(deg/90, DEC);
            Serial.print("   msin= ");
            Serial.print(msin,DEC);
            Serial.print("   mcos= ");
            Serial.print(mcos,DEC);
        }
#endif

        // Creates the absolute x/y position
        // Sin and cos values from table are multiplied by 10k
        signed long x = ( radius * mcos * stepsPerMM) / 10000;
        signed long y = ( radius * msin * stepsPerMM) / 10000;

        if (i==0 ) { // Start, initialise oldxy
            // For 'A' we first move to the stating point
            if (moveTo) {
                ox = 0; 
                oy = 0;
                // penup
                workItem *item = &workItems[count];
                item->task = TASK_PEN_UP;
                count++;
                
                // move from middle point to radius
                item = &workItems[count];
                item->task = TASK_MOVE;
                item->x = x;
                save_x = x;
                item->y = y;
                save_y = y;   
                item->steps = max(abs(x),abs(y));    
                count++;

                // pen down
                item = &workItems[count];
                item->task = TASK_PEN_DOWN;
                count++;
            
            }
                ox=x;
                oy=y;
            // Nothing to do
        } 

#ifdef DEBUG                                
        if (verbose) {
            Serial.print("     x=");
            Serial.print(x);
            Serial.print(",  y=");
            Serial.print(y);
            Serial.print("     ox=");
            Serial.print(ox);
            Serial.print(",  oy=");
            Serial.println(oy);
        }
#endif        

        if (i != 0) {
            workItem *item = &workItems[count];
            item->x = (x - ox) ;
            item->y = (y - oy) ;

            item->steps = max(abs(item->x),abs(item->y));
            item->task = TASK_MOVE;
            
            ox = x;
            oy = y;
#ifdef DEBUG        
            if (verbose) {
                printWorkItem(workItems[count]);
            }
#endif        
            count++;
        }
        deg = deg + dir;
    }
    
    // if A then move back to middle point
    if (moveTo) {
        workItem *item = &workItems[count];
        item->task = TASK_PEN_UP;
        item->x = 0;
        item->y = 0;
        item->steps = 0;
        count++;
        
        // move from middle point to radius
        item = &workItems[count];
        item->task = TASK_MOVE;
        item->x = -save_x;
        item->y = -save_y;
        item->steps = max(abs(save_x),abs(save_y));    
        count++;

        // pen down - we only lower when the pen was
        // down before doing the arc command in moveTo mode
        if (!oldPenStateIsUp) {
            item = &workItems[count];
            item->task = TASK_PEN_DOWN;
            item->x = 0;
            item->y = 0;
            item->steps = 0;

            count++;
        }

    }

    // Attach 'end of input' element
    workItems[count] = END_MARKER;

    if (verbose) {
        Serial.println("D arc -- workitems --");
        printAllWorkItems(workItems);
    }
        
    startWork();          
}

void waitABit() {
    
    while (!done) { // global var that the oneStep sets to true once done.
        delay(2);    
    }
}

// Return sinus value (multiplied by 10000) from progmem
unsigned long my_sin(int deg) {
//  Serial.print("my_sin,deg="); Serial.println(deg);
  unsigned int val = pgm_read_word_near(sin_table + deg);  
  return val;
}

// Return cosinus value (multiplied by 10000) from progmem
unsigned long my_cos(int deg) {
//  Serial.print("my_cos,deg="); Serial.println(deg);
  unsigned int val = pgm_read_word_near(sin_table + (90-deg));
  return val;
}
