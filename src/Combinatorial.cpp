#include <chrono>
#include <cmath>
#include <map>
#include <thread>

#include "Combinatorial.h"
#include "Util.h"
#include "pruebas.h"
#include <iostream>

// EnumNode methods
int EnumNode::getCycle() { return cycle; }
PriorityQueue *EnumNode::getReadyNodes() { return &pq; }
EnumNode *EnumNode::getFather() { return father; }
preNode *EnumNode::getNode() { return node; }
int EnumNode::getMaxRP() { return maxRP; }
void EnumNode::setMaxRP(unsigned int maxRP) { this->maxRP = maxRP; }

// Sched methods
bool Sched::wasObjectiveMet() { return false; }
bool Sched::examineNode(preNode *inst) {
  unsigned int rp = pressure.size() > 0 ? pressure.back() : 0;
  if (priority == Priority::SU && rp >= this->bestSolution) {
    // std::cout << "**" << std::endl;
    this->pruningBestSolution++;
    return false;
  }

  if (priority == Priority::CP &&
      this->currentEnumNode->getCycle() >= this->bestSolution) {
    // std::cout << "**" << std::endl;
    this->pruningBestSolution++;
    return false;
  }

  return true;
}
void Sched::restoreState() {}

bool Sched::findFeasibleNode() {
  // std::cout << "Find feasible node" << std::endl;
  // int sol = this->currentEnumNode->getCycle();
  // if (sol > this->bestSolution) {
  //   // std::cout << "**" << std::endl;
  //   return false;
  // }


  auto pq = this->currentEnumNode->getReadyNodes();
  // std::cout << "size pq: " << pq->nodes.size() << std::endl;
  // std::cout << "index pq: " << pq->index << std::endl;
  if (!pq->empty()) {
    preNode *inst = pq->top();
    if (!examineNode(inst))
      return false;
  } else {
    // std::cout << "pq empty" << std::endl;
    return false;
  }

  // TODO: Averrigar else restoreState();

  return true; // Instruccion
}

EnumNode *Sched::stepForward() {

  int cycle = this->currentEnumNode->getCycle();
  cycle++;

  auto pq = this->currentEnumNode->getReadyNodes();

  // Seleccionar el siguiente nodo
  EnumNode *enumNode;
  if (!pq->empty()) {
    preNode *node = pq->top();
    node->timeStep = cycle;
    // std::cout << node->getID() << ":" << node->timeStep << " " << std::endl;
    enumNode = new EnumNode(node, cycle, this->currentEnumNode, pq->newQueue());
    pq->next(); // en vez de pop, para que en la siguiente iteración se escoga
                // el siguiente nodo
  }

  // Declara que el nodo está vivo
  leftNodes[enumNode->getNode()] = enumNode->getNode()->successors.size();
  // Verificar si el nodo actual es el ultimo uso de sus predecesores
  for (preEdge *edge : enumNode->getNode()->predecessors) {
    preNode *parent = edge->getNode();
    leftNodes[parent]--;
  }

  unsigned int rp = enumNode->getNode()->successors.size() > 0
                        ? 0
                        : enumNode->getNode()->getDestSize();
  for (preNode *node : graph->getNodes()) {
    if (leftNodes[node] > 0) {
      rp += node->getDestSize();
    }
  }
  pressure.push_back(rp);

  if (enumNode->getMaxRP() < rp) {
    enumNode->setMaxRP(rp);
  }

  //si no está en la tabla, agregarlo
  // if (historyTable.find(enumNode->getKey()) == historyTable.end()) {
  //   historyTable[enumNode->getKey()] = enumNode->getMaxRP();
  // } else {
  //   //si está en la tabla, verificar si el rp es menor
  //   if (historyTable[enumNode->getKey()] < enumNode->getMaxRP()) {
  //     //si es menor, no continuar
  //     std::cout << "poda" << std::endl;
  //     delete enumNode;
  //     return nullptr;
  //   } else {
  //     //si es mayor, actualizar el rp y agregarlo
  //     historyTable[enumNode->getKey()] = enumNode->getMaxRP();
  //   }
  // }

  // std::cout << "enum node: " << enumNode->getNode()->getID() << std::endl;
  pq = enumNode->getReadyNodes();
  preNode *node = enumNode->getNode();
  for (preEdge *edge : node->getSuccessors()) {
    preNode *child = edge->getNode();
    child->decrementNumPredLeft();
    if (child->getNumPredLeft() == 0) {
      pq->push(child);
    }
  }

  return enumNode;
}

EnumNode *Sched::backtrack() {
  // Prev EnumNode
  EnumNode *prevEnumNode = this->currentEnumNode->getFather();
  // delete this->currentEnumNode;
  // this->currentEnumNode = nullptr;
  for (preEdge *edge : this->currentEnumNode->getNode()->getSuccessors()) {
    preNode *child = edge->getNode();
    child->incrementNumPredLeft();
  }
  auto pq = prevEnumNode->getReadyNodes();

  leftNodes[this->currentEnumNode->getNode()] = 0;
  for (preEdge *edge : this->currentEnumNode->getNode()->predecessors) {
    preNode *parent = edge->getNode();
    leftNodes[parent]++;
  }
  pressure.pop_back();

  delete this->currentEnumNode;
  this->currentEnumNode = nullptr;
  return prevEnumNode;
}

std::vector<preNode *> Sched::enumerate(int targetLength, int &cycles) {
  PriorityQueue pq = PriorityQueue(priority);
  // Resetear numPredLeft y numSucLeft
  for (preNode *node : graph->getNodes()) {
    node->resetNumPredLeft();
    // node->resetNumSucLeft();
    node->setNumNodesLeft(node->getNumPredLeft()); // Nodes who use this node
    if (node->getNumPredLeft() == 0) {             // Top down
      pq.push(node);
    }
  }

  int cycle = 0;
  // std::cout << "pq size: " << pq.nodes.size() << std::endl;
  // std::cout << "pq index: " << pq.index << std::endl;
  this->rootEnumNode = new EnumNode(nullptr, cycle, nullptr, pq);

  this->currentEnumNode = rootEnumNode;
  int n = graph->getNodes().size();

  this->result = std::vector<preNode *>();

  int backtracking = 0;
  int poda = 0;

  // Set all nodes as not live
  for (preNode *node : graph->getNodes()) {
    leftNodes[node] = 0;
  }

  // std::cout << "----------------" << std::endl;

  this->bestSolution = 1 << 30; // un numero muy grande
  unsigned int enumNodes = 0;
  unsigned int results = 0;
  this->pruningBestSolution = 0;
  while (!allNodesExplored && !wasObjectiveMet()) {
    // std::cout << "Enum nodes: " << enumNodes << std::endl;

    bool foundFeasibleNode = findFeasibleNode();
    if (foundFeasibleNode) {
      auto node = stepForward();
      if (node == nullptr) {
        backtracking++;
        poda++;
        this->currentEnumNode = backtrack();
        continue;
      }

      this->currentEnumNode = node;
      enumNodes++;
      if (this->currentEnumNode->getNode() != nullptr) {
        result.push_back(this->currentEnumNode->getNode());
      }

    } else {
      if (this->currentEnumNode == rootEnumNode) {
        allNodesExplored = true;
      } else {
        backtracking++;
        if (this->currentEnumNode->getNode() != nullptr) {
          // std::cout << "Nodo sacado: " <<
          result.pop_back();
        }
        this->currentEnumNode = backtrack();
      }
    }

    if (result.size() == n) { // se encontró una solución completa
      results++;
      // for (preNode *node : result) {
      //   std::cout << node->getID() << ":" << node->timeStep << " ";
      // }

      // std::cout << std::endl;
      // std::cout << "Pressure: ";
      // for (unsigned int i = 0; i < pressure.size(); i++) {
      //   std::cout << pressure[i] << " ";
      // }
      
      unsigned int maxRP =
          std::max_element(pressure.begin(), pressure.end())[0];
      // std::cout << " -> " << maxRP << std::endl;
      // unsigned int maxRP = this->currentEnumNode->getMaxRP();
      if (priority == Priority::SU && maxRP < this->bestSolution) {
        this->bestSolution = maxRP;
        this->bestResult = std::vector<std::pair<int, int>>();
        this->bestPressure = std::vector<int>();
        //Insertar presiones a bestPressure
        this->bestPressure.insert(this->bestPressure.end(), pressure.begin(),
                                  pressure.end());
        for (preNode *node : result) {
          this->bestResult.push_back(std::pair<int, int>(node->getID(),
                                                          node->timeStep));
        }
      } else if (priority == Priority::CP &&
                 result.back()->timeStep < this->bestSolution) {
        this->bestSolution = result.back()->timeStep;
        //Insertar presiones a bestPressure
        this->bestPressure = std::vector<int>();
        this->bestPressure.insert(this->bestPressure.end(), pressure.begin(),
                                  pressure.end());
        this->bestResult = std::vector<std::pair<int, int>>();
        for (preNode *node : result) {
          this->bestResult.push_back(std::pair<int, int>(node->getID(),
                                                          node->timeStep));
        }

      }
    }
  }

  // best
  // std::cout <<  std::endl << "Max RP Enum: " << this->bestSolution << std::endl;
  // std::cout << "Enum nodes: " << enumNodes << std::endl;
  // std::cout << "Results: " << results << std::endl;
  // std::cout << "Pruning best solution: " << this->pruningBestSolution <<
  // std::endl;
  std::vector<preNode *> bestResult = std::vector<preNode *>();
  for (std::pair<int, int> pair : this->bestResult) {
    for (preNode *node : graph->getNodes()) {
        if (node->getID() == pair.first) {
          node->timeStep = pair.second;
          bestResult.push_back(node);
        }
    }
  }
  // std::cout << "Backtracking: " << backtracking << std::endl;
  // std::cout << "Poda: " << poda << std::endl;
  return bestResult;
}