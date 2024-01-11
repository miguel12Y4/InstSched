#ifndef COMBINATORIAL_H
#define COMBINATORIAL_H

#include <algorithm>
#include <iostream>
#include <list>
#include <queue>
#include <map>


#include "Base.h"
#include "Util.h"

#include <sstream>
#include <vector>

enum InstType { MEMORY, ALU, OTHER_INST };

enum RegType { INT, FLOAT, OTHER_REG };

// Declaraciones de clases
class EnumNode;
class preNode;
class preEdge;

class EnumNode {
private:
  int cycle;
  EnumNode *father;
  preNode *node;
  std::string key = ""; // key del nodo para dominacion basada en la historia

  PriorityQueue pq;
  unsigned int maxRP;
  unsigned int rp;

public:
  EnumNode(preNode *node, int cycle, EnumNode *father, PriorityQueue pq) {
    // std::cout << "EnumNode constructor" << std::endl;
    this->node = node;
    this->cycle = cycle;
    this->father = father;
    this->pq = pq;
    this->key = getKey();
    // std::cout << "key: " << key << std::endl;
  }

  std::string getKey() {
    // std::cout << "getKey" << std::endl;
    if (key != "") {
      return key;
    }

    if(node == NULL || node == nullptr){
      // std::cout << "node is null " << "" << std::endl;
      return "";
    }

    if(EnumNode::getFather() == NULL || EnumNode::getFather() == nullptr){
      // std::cout << "father is null " << ""+node->ID << std::endl;
      return node->ID+"";
    }

    auto split = [](const std::string &s, char delim) {
      std::vector<int> result;
      std::stringstream ss(s);
      std::string item;
      while (getline(ss, item, delim)) {
        result.push_back(std::stoi(item));
      }
      return result;
    };

    auto join = [](const std::vector<int> &v, char delim) {
      std::stringstream ss;
      for (int i = 0; i < v.size(); i++) {
        ss << v[i];
        if (i < v.size() - 1) {
          ss << delim;
        }
      }
      return ss.str();
    };


    std::string keyFather = EnumNode::getFather()->getKey();
    //split keyFather by ',', convert to int, sort, convert to string, join by ','
    std::vector<int> keyFatherSplit = split(keyFather, ',');

    keyFatherSplit.push_back(node->ID);
    std::sort(keyFatherSplit.begin(), keyFatherSplit.end());
    std::string newKey = join(keyFatherSplit, ',');
    return newKey;
  }

  int getCycle();
  PriorityQueue *getReadyNodes();
  EnumNode *getFather();
  preNode *getNode();
  int getMaxRP();
  void setMaxRP(unsigned int maxRP);
};

class Sched {
  public:
  DDG *graph;
  EnumNode *rootEnumNode;
  EnumNode *currentEnumNode;
  int n;
  bool allNodesExplored;

  std::map<preNode *, int> leftNodes;
  std::vector<unsigned int> pressure;
  std::vector<preNode *> result;

  std::vector<std::pair<int, int>> bestResult; // (id, cycle)
  std::vector<int> bestPressure;

  unsigned int rp;
  unsigned int maxRP;
  unsigned int bestSolution;
  unsigned int pruningBestSolution;

  Priority priority;


  //History Table: {key: "", maxRP: 0}
  std::map<std::string, unsigned int> historyTable;
// public:
  Sched(DDG *graph, Priority priority = Priority::CP) {
    this->graph = graph;
    this->n = graph->getNodes().size();
    this->allNodesExplored = false;
    this->priority = priority;
  }

  bool wasObjectiveMet();
  bool examineNode(preNode *inst);
  void restoreState();
  bool findFeasibleNode();
  EnumNode *stepForward();
  EnumNode *backtrack();
  std::vector<preNode *> enumerate(int targetLength, int &cycles);
};

#endif