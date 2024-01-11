#include <algorithm>
#include <map>
#include <queue>

#include "Base.h"
#include "IGC_Sched.h"

// Solo computa MaxRegs y DstSizes de todos los nodos del grafo
void SethiUllmanQueue::init() {
  auto &Nodes = graph->getNodes();
  unsigned N = (unsigned)Nodes.size();
  MaxRegs.resize(N, 0);
  DstSizes.resize(N, 0);
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

void SethiUllmanQueue::calculateSethiUllmanNumber(preNode *node) {
  // condicion de salida ya que el nodo ya fué calculado si tiene un valor
  // distinto de 0 en MaxRegs o DstSizes
  auto CurNum = MaxRegs[node->getID()];
  if (CurNum != 0 || DstSizes[node->getID()] != 0) {
    // std::cout << "continue (valores calculados)" << std::endl;
    return;
  }

  // std::cout << "Calculando Sethi-Ullman number para nodo " << node->getID()
  // << std::endl;

  DstSizes[node->getID()] = node->getDestSize();
  std::vector<std::pair<preNode *, int>> Preds;
  // Process the predecessors first
  for (auto I = node->pred_begin(), E = node->pred_end(); I != E; ++I) {
    auto &edge = *I;
    if (edge->getDepType() != RAW && edge->getDepType() != WAW) {
      // std::cout << "Error tipo de dependencia" << std::endl;
      continue;
    }
    calculateSethiUllmanNumber(edge->getNode());
    auto MaxReg = MaxRegs[edge->getNode()->getID()];
    auto DstSize = DstSizes[edge->getNode()->getID()];
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
      auto DstSize = DstSizes[PN->getID()];
      auto MaxReg = MaxRegs[PN->getID()];
      CurNum = std::max(MaxReg, CurNum + DstSize);
    }
  }

  MaxRegs[node->getID()] = CurNum;
}

// list scheduling with Sethi-Ullman
std::vector<preNode *> SethiUllmanQueue::listScheduling() {

  // cola de prioridad por segunda componente de un pair
  std::priority_queue<std::pair<preNode *, int>,
                      std::vector<std::pair<preNode *, int>>, ComparadorPair>
      pq;
  std::vector<preNode *> nodes =
      std::vector<preNode *>(graph->getNodes().size());
  for (preNode *node : graph->getLeafNodes()) {
    pq.push(
        std::make_pair(node, MaxRegs[node->getID()] - DstSizes[node->getID()]));
  }
  for (preNode *node : graph->getNodes()) {
    node->resetNumSucLeft();
    node->resetNumPredLeft();
  }

  int t = graph->getNodes().size() - 1;
  while (!pq.empty()) {
    preNode *node = pq.top().first;
    pq.pop();
    node->timeStep = t + 1;
    nodes[t] = node;
    // Bottom-up scheduling
    for (preEdge *edge : node->getPredecessors()) {
      edge->getNode()->decrementNumSucLeft();
      if (edge->getNode()->getNumSucLeft() == 0) {
        pq.push(std::make_pair(edge->getNode(),
                               MaxRegs[edge->getNode()->getID()] -
                                   DstSizes[edge->getNode()->getID()]));
      }
    }
    t--;
  }
  return nodes;
}

// list scheduling with SU and RP-reduction
std::vector<preNode *> SethiUllmanQueue::listSchedulingRP() {

  for (preNode *node : graph->getNodes()) {
    node->resetNumSucLeft();
    node->resetNumPredLeft();
  }

  // cola de prioridad por segunda componente de un pair
  std::priority_queue<std::pair<preNode *, int>,
                      std::vector<std::pair<preNode *, int>>, ComparadorPair>
      Q;
  std::priority_queue<std::pair<preNode *, int>,
                      std::vector<std::pair<preNode *, int>>, ComparadorPair>
      W;

  std::vector<preNode *> nodes(graph->getNodes().size());
  std::vector<int> liveTS(graph->getNodes().size(), -1);
  for (preNode *node : graph->getLeafNodes()) {
    Q.push(std::make_pair(node,
                          MaxRegs[node->getID()] -
                              DstSizes[node->getID()])); // Sethi-Ullman number
  }

  int t = graph->getNodes().size() - 1;
  while (!Q.empty()) {
    preNode *node = Q.top().first;
    Q.pop();

    W.push(
        std::make_pair(node, MaxRegs[node->getID()] - DstSizes[node->getID()]));
    while (!W.empty()) {
      node = W.top().first;
      W.pop();
      nodes[t] = node;
      node->timeStep = t + 1;

      // compute live de nodo
      for (preEdge *edge : node->getPredecessors()) {
        liveTS[edge->getNode()->getID()] =
            std::max(t, liveTS[edge->getNode()->getID()]);
      }
      // Bottom-up scheduling
      for (preEdge *edge : node->getPredecessors()) {
        edge->getNode()->decrementNumSucLeft();
        if (edge->getNode()->getNumSucLeft() == 0) {
          int d = liveTS[edge->getNode()->getID()] > 0
                      ? DstSizes[edge->getNode()->getID()]
                      : 0;
          for (preEdge *edge2 : edge->getNode()->getPredecessors()) {
            d = liveTS[edge2->getNode()->getID()] < 0
                    ? (d - DstSizes[edge2->getNode()->getID()])
                    : d;
          }
          if (d >= 0) {
            W.push(std::make_pair(edge->getNode(),
                                  MaxRegs[edge->getNode()->getID()] -
                                      DstSizes[edge->getNode()->getID()]));
          } else {
            Q.push(std::make_pair(edge->getNode(),
                                  MaxRegs[edge->getNode()->getID()] -
                                      DstSizes[edge->getNode()->getID()]));
          }
        }
      }
      t--;
    }
  }

  return nodes;
}

class Cluster {
public:
  std::vector<preNode *> nodes;
  std::vector<unsigned int> operands;
  Cluster() {
    this->nodes = std::vector<preNode *>();
    this->operands = std::vector<unsigned int>();
  }
  Cluster(std::vector<preNode *> nodes) {
    this->nodes = nodes;
    operands = std::vector<unsigned int>();
    // Compute operands
    for (preNode *node : nodes) {
      for (preEdge *edge : node->getPredecessors()) {
        if (std::find(operands.begin(), operands.end(),
                      edge->getNode()->getID()) == operands.end()) {
          operands.push_back(edge->getNode()->getID());
        }
      }
    }
  }

  void addNode(preNode *node) {
    nodes.push_back(node);

    // Update operands
    updateOperands();
  }

  void updateOperands() {
    this->operands = std::vector<unsigned int>();
    for (preNode *node : nodes) {
      for (preEdge *edge : node->getPredecessors()) {
        if (std::find(operands.begin(), operands.end(),
                      edge->getNode()->getID()) == operands.end()) {
          operands.push_back(edge->getNode()->getID());
        }
      }
    }
  }

  bool shareOperands(preNode *node) {
    for (preEdge *edge : node->getPredecessors()) {
      if (std::find(operands.begin(), operands.end(),
                    edge->getNode()->getID()) != operands.end()) {
        return true;
      }
    }
    return false;
  }

  bool allready(std::vector<bool> ready) {
    for (preNode *node : nodes) {
      if (!ready[node->getID()]) {
        return false;
      }
    }
    return true;
  }

  void deleteNode(preNode *node) {
    nodes.erase(std::remove(nodes.begin(), nodes.end(), node), nodes.end());
    updateOperands();
  }

  void clear() {
    nodes.clear();
    operands.clear();
  }

  // Retornar nodos en ready list que tenga dependencias un nodo
  // TODO: hacerlo recursivo?
  std::vector<preNode *>
  getNodesReadyWithDependencies(preNode *node, std::vector<bool> ready,
                                std::vector<bool> scheduled) {
    std::vector<preNode *> nodesWithDependencies = std::vector<preNode *>();
    if (node->getSuccessors().size() == 0) {
      // nodesWithDependencies.push_back(node);
      return nodesWithDependencies;
    }

    for (preEdge *edge : node->getSuccessors()) {
      if (!scheduled[edge->getNode()->getID()] &&
          !ready[edge->getNode()->getID()]) {
        std::vector<preNode *> nodesWithDependenciesAux =
            getNodesReadyWithDependencies(edge->getNode(), ready, scheduled);
        for (preNode *node : nodesWithDependenciesAux) {
          // si no esta en la lista de nodos con dependencias, agregarlo
          if (std::find(nodesWithDependencies.begin(),
                        nodesWithDependencies.end(),
                        node) == nodesWithDependencies.end()) {
            nodesWithDependencies.push_back(node);
          }
        }
      }
    }
    return nodesWithDependencies;
  }
};

class PriorityPair {
private:
  std::vector<std::pair<preNode *, int>> elementos;

public:
  // Inserta un elemento en la cola
  void push(preNode *nodo, int prioridad) {
    elementos.emplace_back(nodo, prioridad);
    std::push_heap(elementos.begin(), elementos.end(), ComparadorPair());
  }

  // Elimina y devuelve el elemento con la mayor prioridad
  std::pair<preNode *, int> pop() {
    if (elementos.empty()) {
      throw std::runtime_error("La cola de prioridad está vacía");
    }

    std::pop_heap(elementos.begin(), elementos.end(), ComparadorPair());
    std::pair<preNode *, int> top_element = elementos.back();
    elementos.pop_back();

    return top_element;
  }

  // Devuelve el elemento con la mayor prioridad
  std::pair<preNode *, int> top() const {
    if (elementos.empty()) {
      throw std::runtime_error("La cola de prioridad está vacía");
    }

    return elementos.front();
  }

  // Eliminar nodos de la cola de prioridad
  void remove(std::vector<preNode *> nodes) {
    for (preNode *node : nodes) {
      elementos.erase(std::remove_if(elementos.begin(), elementos.end(),
                                     [node](std::pair<preNode *, int> pair) {
                                       return pair.first->getID() ==
                                              node->getID();
                                     }),
                      elementos.end());
    }
    std::make_heap(elementos.begin(), elementos.end(), ComparadorPair());
  }

  // Verifica si la cola de prioridad está vacía
  bool empty() const { return elementos.empty(); }

  // Obtiene el número de elementos en la cola de prioridad
  size_t tamano() const { return elementos.size(); }
};

// list scheduling with SU and RP-reduction and clustering
std::vector<preNode *> SethiUllmanQueue::listSchedulingRPCL() {

  for (preNode *node : graph->getNodes()) {
    node->resetNumSucLeft();
    node->resetNumPredLeft();
  }

  // cola de prioridad por segunda componente de un pair
  PriorityPair Q;
  PriorityPair W;

  int n = graph->getNodes().size();
  std::vector<bool> ready = std::vector<bool>(n, false);
  std::vector<bool> scheduled = std::vector<bool>(n, false);

  std::vector<preNode *> nodes = std::vector<preNode *>(n);
  std::vector<int> liveTS = std::vector<int>(n, -1);

  for (preNode *node : graph->getLeafNodes()) {
    Q.push(node, MaxRegs[node->getID()] -
                     DstSizes[node->getID()]); // Sethi-Ullman number
    ready[node->getID()] = true;
  }

  int t = n - 1;
  Cluster cluster = Cluster();
  while (!Q.empty()) {
    preNode *node = Q.top().first;

    cluster.addNode(node);

    // buscar nodes no scheduled y comparte operandos con cluster
    for (preNode *n : graph->getNodes()) {
      if (!scheduled[n->getID()] && cluster.shareOperands(n)) {
        if (node->getID() != n->getID()) {
          // if (node->getNumSucLeft() == 0) {
          //   ready[n->getID()] = true;
          // }
          cluster.addNode(n);
        }
      }
    }
    if (cluster.allready(ready)) {
      for (preNode *n : cluster.nodes) {
        W.push(n, MaxRegs[n->getID()] - DstSizes[n->getID()]);
      }
      // for (preNode *n : cluster.nodes) { //TODO: revisar
      //   ready[n->getID()] = false;
      // }
      Q.remove(cluster.nodes);
      cluster.clear();
    } else {
      // k = nodos en ready list que tienen dependencias en cluster sin cumplir
      std::vector<preNode *> js = std::vector<preNode *>();
      std::vector<preNode *> k = std::vector<preNode *>();
      for (preNode *n : cluster.nodes) {
        if (n->getNumSucLeft() > 0 && !scheduled[n->getID()]) {
          js.push_back(n);
          // break;
        }else if(n->getNumSucLeft() == 0 && !scheduled[n->getID()]){
          k.push_back(n);
        }
      }
      if (js.size() == 0) {
        std::cout << "ERROR: js vacio" << std::endl;
      }

      std::vector<preNode *> nodesGrafo = graph->getNodes();
      for (preNode *n : nodesGrafo) {
        if (n->getNumSucLeft() == 0 && !scheduled[n->getID()]) {
          std::cout << "nodo ready: " << n->getID() << std::endl;
          for (preNode *j : js) {
            // si es descendiente de j y no esta en k
            if (j->isDecendent(n) &&
                std::find(k.begin(), k.end(), n) == k.end()) {
              k.push_back(n);
              // break;
            }
          }
        }
      }
      if (k.size() == 0) {
        std::cout << "ERROR: k vacio" << std::endl;
        for(preNode* j : js){
          std::cout << "j: " << j->getID() << "-:-" << j->getNumSucLeft() << std::endl;
        }
        //lazar error
        // return nodes;
      }else{
        std::cout << "k: ";
        for (preNode *n : k) {
          std::cout << n->getID() << " ";
        }

        std::cout << "j: ";
        for (preNode *n : js) {
          std::cout << n->getID() << "-:-" << n->getNumSucLeft() << " ";
        }

        std::cout << std::endl;
      }
      // remove k from Q que bloquea a j
      Q.remove(k);
      // add k to W
      for (preNode *n : k) {
        W.push(n, MaxRegs[n->getID()] - DstSizes[n->getID()]);
      }
    }

    while (!W.empty()) {
      node = W.top().first;
      W.pop();
      nodes[t] = node;
      node->timeStep = t + 1;
      scheduled[node->getID()] = true;
      ready[node->getID()] = false;
      // compute live de nodo
      for (preEdge *edge : node->getPredecessors()) {
        liveTS[edge->getNode()->getID()] =
            std::max(t, liveTS[edge->getNode()->getID()]);
      }
      // Bottom-up scheduling
      for (preEdge *edge : node->getPredecessors()) {
        edge->getNode()->decrementNumSucLeft();
        if (edge->getNode()->getNumSucLeft() == 0) {
          ready[edge->getNode()->getID()] = true;
          int d = liveTS[edge->getNode()->getID()] > 0
                      ? DstSizes[edge->getNode()->getID()]
                      : 0;
          for (preEdge *edge2 : edge->getNode()->getPredecessors()) {
            d = liveTS[edge2->getNode()->getID()] < 0
                    ? (d - DstSizes[edge2->getNode()->getID()])
                    : d;
          }
          if (d >= 0) {
            W.push(edge->getNode(), MaxRegs[edge->getNode()->getID()] -
                                        DstSizes[edge->getNode()->getID()]);
          } else {
            Q.push(edge->getNode(), MaxRegs[edge->getNode()->getID()] -
                                        DstSizes[edge->getNode()->getID()]);
          }
        }
      }
      // if (t <= graph->getNodes().size() - 13) {
      //   break;
      // }
      t--;
    }
  }
  std::cout << "Fin del algoritmo" << std::endl;

  return nodes;
}

// list scheduling for latency
std::vector<preNode *> LatencyListScheduler::listScheduling(int &cycles) {
  std::priority_queue<preNode *, std::vector<preNode *>, ComparadorCriticalPath>
      ReadyList;
  std::priority_queue<std::pair<preNode *, int>,
                      std::vector<std::pair<preNode *, int>>, ComparadorPair>
      ActiveList;
  // arreglo para ver nodos ya procesados
  std::vector<bool> visited(graph->getNodes().size(), false);

  for (preNode *node : graph->getNodes()) {
    node->resetNumPredLeft();
    node->resetNumSucLeft();
    if (node->getNumPredLeft() == 0) {
      ReadyList.push(node);
    }
    visited[node->getID()] = false;
  }

  std::vector<preNode *> nodes;

  int cycle = 1;
  while (nodes.size() < graph->getNodes().size()) {
    // imprimir tamaño de ready list
    if (!ReadyList.empty()) {
      preNode *node = ReadyList.top();
      node->timeStep = cycle;
      ReadyList.pop();
      nodes.push_back(node);
      visited[node->getID()] = true;
      // std::cout << node->getID() << "-" << cycle << "-" << cycle +
      // node->getLatency() << " ";
      ActiveList.push(std::make_pair(node, cycle + node->getLatency() - 1));
    }

    bool exit = false;
    // verificar si en active list hay algun nodo que ya no tenga dependencias
    while (!ActiveList.empty() && !exit) {
      preNode *node = ActiveList.top().first;
      int cycleFinish = ActiveList.top().second;

      if (cycleFinish <= cycle) {
        ActiveList.pop();
        for (preEdge *edge : node->getSuccessors()) {
          if (visited[edge->getNode()->getID()]) { // si ya fue procesado
            continue;
          }
          edge->getNode()->decrementNumPredLeft();
          if (edge->getNode()->getNumPredLeft() == 0) {
            ReadyList.push(edge->getNode());
          }
        }
      } else {
        exit = true;
      }
    }
    cycle++;
  }

  cycles =
      cycle - 1 +
      nodes.back()
          ->getLatency(); // restar 1 porque se incrementa al final del while
  // std::cout << "Ciclos: " << cycles << std::endl;

  return nodes;
}
