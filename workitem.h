#ifndef WORKITEM_H
#define WORKITEM_H

// A work item is a single "relative" line, resolved into steps
struct workItem {
  long steps;  // total number of steps for this item
  long x;      // steps in x direction
  long y;      // steps in y direction
  byte task;
};

#endif