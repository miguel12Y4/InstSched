#ifndef UTIL_H
#define UTIL_H

#include <iostream>
#include <vector>

#include "Base.h"


enum class Priority { 
  CP, // Critical Path
  SU, // Sethi-Ullman

};
class PriorityQueue {
public:
  std::vector<preNode *> nodes;
  unsigned int index;
  bool stall;
  Priority priority;

// public:
  PriorityQueue(std::vector<preNode *> nodes, Priority priority=Priority::CP) {
    this->nodes = nodes;
    this->index = 0;
    this->stall = false;
    this->priority = priority;
  }

  PriorityQueue(Priority priority=Priority::CP) {
    this->nodes = std::vector<preNode *>();
    this->index = 0;
    this->stall = false;
    this->priority = priority;
  }

  bool empty();
  void push(preNode *node);
  preNode *top();
  void pop();
  void reset();
  void next();
  PriorityQueue newQueue();
  PriorityQueue copy();
  void setStall(bool stall);
  bool getStall();
  void remove(preNode *node);
};


#endif // UTIL_H