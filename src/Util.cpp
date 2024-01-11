#include "Util.h"

bool PriorityQueue::empty() { return nodes.empty() || index >= nodes.size(); }

void PriorityQueue::push(
    preNode *node) { // Insertar según prioridad, mayor a menor
  for (int i = index; i < nodes.size(); i++) {
    if (priority == Priority::CP && node->getCriticalPath() > nodes[i]->getCriticalPath()) {
      nodes.insert(nodes.begin() + i,node); 
      // if (index >= i && updateIndex) {
      //   index++;
      // }
      return;
    }else if (priority == Priority::SU && (node->maxRP-node->destSize) < (nodes[i]->maxRP-nodes[i]->destSize)) { //TODO: verficar ya que es bottom up pero se usa top down
      nodes.insert(nodes.begin() + i,node);
      // if (index >= i && updateIndex) {
      //   index++;
      // }
      return;
    }
  }
  nodes.push_back(node);
}

preNode *PriorityQueue::top() { return nodes[index]; }

void PriorityQueue::pop() {
  nodes.erase(nodes.begin() + index);
} // TODO: no usarlo ya que se usa next

void PriorityQueue::reset() { index = 0; }

void PriorityQueue::next() { index++; }

// Copiar cola de prioridad sin el nodo actual
PriorityQueue PriorityQueue::newQueue() {
  PriorityQueue pq;
  std::vector<preNode *> nodes = std::vector<preNode *>(this->nodes.begin(), this->nodes.end());
  if (index < nodes.size()) {
    nodes.erase(nodes.begin() + index);
  }
  pq.nodes = nodes;
  pq.index = 0;
  return pq;
}

// Copiar cola de prioridad
PriorityQueue PriorityQueue::copy() {
  PriorityQueue pq;
  std::vector<preNode *> nodes =
      std::vector<preNode *>(this->nodes.begin(), this->nodes.end());
  pq.nodes = nodes;
  pq.index = 0;
  return pq;
}

void PriorityQueue::setStall(bool stall) { this->stall = stall; }

bool PriorityQueue::getStall() { return this->stall; }

void PriorityQueue::remove(preNode *node) {
  for (int i = 0; i < nodes.size(); i++) {
    if (nodes[i]->getID() == node->getID()) {
      nodes.erase(nodes.begin() + i);
      if (index > i) {
        index--;
      }
      return;
    }
  }
}