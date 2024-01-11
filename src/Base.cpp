
#include "Base.h"
#include <algorithm>

int preNode::calculateCriticalPath(Direction direction) {
  if (isCriticalPathCalculated) {
    return criticalPath;
  }
  if (getSuccessors().size() == 0) {
    criticalPath = getLatency();
    isCriticalPathCalculated = true;
    return getLatency();
  }

  int max = 0;
  for (preEdge *edge :
       direction == Direction::TOPDOWN ? getSuccessors() : getPredecessors()) {
    int path =
        edge->getLatency() + edge->getNode()->calculateCriticalPath(direction);
    if (path > max) {
      max = path;
    }
  }
  criticalPath = max;
  isCriticalPathCalculated = true;
  return max;
}

// calcular ruta del camino crítico
std::vector<int> preNode::getCriticalPath() {
  std::vector<int> path;
  if (getSuccessors().size() == 0) {
    path.push_back(getID());
    return path;
  }
  int max = 0;
  preNode *maxNode = nullptr;
  for (preEdge *edge : getSuccessors()) {
    int path = edge->getLatency() + edge->getNode()->calculateCriticalPath();
    if (path > max) {
      max = path;
      maxNode = edge->getNode();
    }
  }
  path = maxNode->getCriticalPath();
  // path.push_back(getID());
  path.insert(path.begin(), getID());
  return path;
}

void calculateSethiUllmanNumber(preNode *node) {
  // condicion de salida ya que el nodo ya fué calculado si tiene un valor
  // distinto de 0 en MaxRegs o DstSizes
  auto CurNum = node->maxRP;
  if (CurNum != 0 || node->destSize != 0) {
    // std::cout << "continue (valores calculados)" << std::endl;
    return;
  }

  // std::cout << "Calculando Sethi-Ullman number para nodo " << node->getID()
  // << std::endl;

  std::vector<std::pair<preNode *, int>> Preds;
  // Process the predecessors first
  for (auto I = node->pred_begin(), E = node->pred_end(); I != E; ++I) {
    auto &edge = *I;
    if (edge->getDepType() != RAW && edge->getDepType() != WAW) {
      // std::cout << "Error tipo de dependencia" << std::endl;
      continue;
    }
    calculateSethiUllmanNumber(edge->getNode());
    auto MaxReg = node->maxRP;
    auto DstSize = node->destSize;
    Preds.emplace_back(edge->getNode(), MaxReg - DstSize);
  }

  if (node->getNumPredLeft() != 0) {
    // std::cout << "ERROR: Algoritmo Sethi-Ullman no se esta aplicando a un
    // nodo " << node->getID() << " con todos sus descendientes calculados" <<
    // std::endl;
  }

  // Decrement the number of descendants left for each successor
  for (preEdge *edge : node->getSuccessors()) {
    edge->getNode()->decrementNumPredLeft();
  }

  if (Preds.size() > 0) {
    std::sort(Preds.begin(), Preds.end(),
              [](std::pair<preNode *, int> lhs, std::pair<preNode *, int> rhs) {
                return lhs.second < rhs.second;
              });
    // Calcula valor para curNum que es el maximo (max RP) (PRP) de los valores
    // de los predecesores
    for (unsigned i = 0, e = (unsigned)Preds.size(); i < e; ++i) {
      auto PN = Preds[i].first;
      auto DstSize = PN->destSize;
      auto MaxReg = PN->maxRP;
      CurNum = std::max(MaxReg, CurNum + DstSize);
    }
  }

  node->maxRP = CurNum;
}


void DDG::calculateSethiUllman() {
  auto &Nodes = getNodes();
  unsigned N = (unsigned)Nodes.size();
  // resetear el numero de predecesores de cada nodo
  for (preNode *node : Nodes) {
    node->resetNumPredLeft();
  }

  for (auto I = Nodes.rbegin(); I != Nodes.rend();
       ++I) { // calcula sethi ullman number para todos los nodos del grafo en
              // orden inverso
    calculateSethiUllmanNumber((*I));
  }
}




std::vector<unsigned> DDG::computeRP() {
  std::vector<preNode *> &nodes = getNodes();
  // tiempo de última hoja
  unsigned max = 0;
  for (preNode *node : getLeafNodes()) {
    if (node->timeStep > max) {
      max = node->timeStep + node->getLatency();
    }
  }
  std::vector<unsigned> RP(max + 1, 0);
  // ordenar por tiempo de ejecución
  std::sort(nodes.begin(), nodes.end(),
            [](preNode *a, preNode *b) { return a->timeStep < b->timeStep; });

  for (int i = 0; i < (int)nodes.size(); i++) {
    preNode *node = nodes[i];
    // obtener id del maximo sucesor
    unsigned max = 0;
    for (preEdge *edge : node->getSuccessors()) {
      unsigned time = edge->getNode()->timeStep;
      if (time > max) {
        max = time;
      }
    }
    if (max == 0) {
      // std::cout << "Node " << node->getID() << " has no successors" <<
      // std::endl;
      max = node->timeStep + node->getLatency();
    }
    for (int j = node->timeStep - 1; j < max; j++) {
      RP[j] += node->getDestSize();
    }
  }
  return RP;
}

DDG DDG::generarArbol(int n, int d, bool invertido) {
  if (n < 1 || d < 1) {
    throw std::invalid_argument(
        "Número de nodos y grado deben ser mayores o iguales a 1.");
  }

  DDG G;

  // Crear nodos
  for (int i = 0; i < n; i++) {
    G.addNode(new preNode(i, 1, rand() % 10 + 1));
  }

  std::vector<preNode *> nodes = G.getNodes();

  // Conectar nodos para formar un árbol
  for (int i = 0; i < n; i++) {
    for (int j = 1; j <= d; j++) {
      int childIndex = i * d + j;
      if (childIndex < n) {
        if (invertido) {
          G.addEdge(nodes[childIndex], nodes[i],
                    nodes[childIndex]->getLatency(), DepType::RAW);
        } else {
          G.addEdge(nodes[i], nodes[childIndex], nodes[i]->getLatency(),
                    DepType::RAW);
        }
      }
    }
  }

  return G;
}

std::vector<preNode *> DDG::getSubGraph(std::vector<preNode *> hojas,
                                        Direction direction) {
  std::vector<preNode *> nodes;
  std::vector<preNode *> nodesToVisit = hojas;
  while (!nodesToVisit.empty()) {
    preNode *node = nodesToVisit.back();
    nodesToVisit.pop_back();
    if (std::find(nodes.begin(), nodes.end(), node) == nodes.end()) { // not found
      nodes.push_back(node);
    }

    if (direction == Direction::BOTTOMUP) {
      for (preEdge *edge : node->getPredecessors()) {
        if (std::find(nodes.begin(), nodes.end(), edge->getNode()) ==
            nodes.end()) {
          nodesToVisit.push_back(edge->getNode());
        }
      }
    } else {
      for (preEdge *edge : node->getSuccessors()) {
        if (std::find(nodes.begin(), nodes.end(), edge->getNode()) ==
            nodes.end()) {
          nodesToVisit.push_back(edge->getNode());
        }
      }
    }

  }
  //TODO: Ajustar predecesores y sucesores
  return nodes;
}
