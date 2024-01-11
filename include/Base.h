#ifndef BASE_H
#define BASE_H

#include <iostream>
#include <vector>
#include <memory>
#include <unordered_map>
#include <algorithm> 



class preNode;

enum DepType {
  RAW,
  WAW,
  OTHER
};

enum Direction {
  TOPDOWN,
  BOTTOMUP
};


struct config{
    bool SkipHoldList;
    bool DoNotIterate;
    bool UseLatency;
};

class preEdge {
private:
    preNode* destination;
    unsigned int latency;
    DepType depType;


public:
    preEdge(preNode* destination, unsigned int latency, DepType depType)
        : destination(destination), latency(latency), depType(depType) {}


    preNode* getNode() {
        return destination;
    }

    int getLatency() {
        return latency;
    }

    DepType getDepType() {
        return depType;
    }

    friend class OptimaInstructionScheduling;
};


class preNode {
public: 
    int ID;
    int latency;
    int criticalPath;

    int destSize; // SU number maxRP - destSize
    int maxRP;
    
    int numPredecessorsLeft;
    int numSuccessorsLeft;
    int numNodesLeft; // Number of nodes left who used this node

    //Combinatorial approach release time (ASAP As Son As Possible) and deadline (ALAP As Late As Possible)
    int releaseTime;
    int deadline;

    //Combinatorial approach (Lower Bound Under Constraint)
    int ASAPUC;
    int ALAPUC;

    //Time step in which the node is scheduled
    int timeStep;

    bool isScheduled;
    bool isCriticalPathCalculated;
    bool isSethiUllmanCalculated;


    std::vector<preEdge*> successors;
    std::vector<preEdge*> predecessors;

    bool stall;
// public:
    preNode(int ID, int destSize, int latency = 0, bool stall=false)
        : ID(ID), destSize(destSize), numPredecessorsLeft(0), numSuccessorsLeft(0), releaseTime(0), deadline(0), ASAPUC(0), ALAPUC(0), timeStep(-1), isScheduled(false), isCriticalPathCalculated(false), criticalPath(0) {
            this->latency = latency;
            //random 2 to 4
            // this->latency = rand() % 300 + 2;
            this->stall = stall;
        }

    int getID() {
        return ID;
    }

    //crear nodo vacio
    static preNode* createEmptyNode() {
        return new preNode(-1, 0, 1, true);
    }

    bool isStall() {
        return stall;
    }

    //Cycle in which the node is scheduled TODO: es correcto el nombre ready?
    int getScheduledCycle() {
        if (!isScheduled) {
            return -1;
        }
        return timeStep;
    }

    //Cycle in which the node is scheduled
    void setScheduledCycle(int timeStep) {
        this->timeStep = timeStep;
    }

    void setIsScheduled(bool isScheduled) {
        this->isScheduled = isScheduled;
    }

    void addSuccessor(preEdge* edge) {
        successors.push_back(edge);
        preEdge *e = new preEdge(this, edge->getLatency(), edge->getDepType());
        edge->getNode()->addPredecessor(e);
    }

    void addNodeSuccessor(preNode* node, unsigned int latency, DepType depType) {
        preEdge* edge = new preEdge(node, latency, depType);
        successors.push_back(edge);
        preEdge *e = new preEdge(this, latency, depType);
        node->addPredecessor(e);
    }

    void addPredecessor(preEdge* edge) {
        predecessors.push_back(edge);
    }

    int getLatency() {
        return latency;
    }

    int getDestSize() {
        return destSize;
    }

    std::vector<preEdge*> getSuccessors() {
        return successors;
    }

    std::vector<preEdge*> getPredecessors() {
        return predecessors;
    }

    bool isDecendent(preNode* node, bool byPredecessors = false) {
        for (preEdge* edge : (byPredecessors ? predecessors : successors)) {
            if (edge->getNode() == node) {
                return true;
            }
            if (edge->getNode()->isDecendent(node, byPredecessors)) {
                return true;
            }
        }
        return false;
    }

    void incrementNumPredLeft() {
        numPredecessorsLeft++;
    }

    void incrementNumSucLeft() {
        numSuccessorsLeft++;
    }

    void decrementNumPredLeft() {
        numPredecessorsLeft--;
    }

    void decrementNumSucLeft() {
        numSuccessorsLeft--;
    }

    void decrementNumNodesLeft() {
        numNodesLeft--;
    }

    void incrementNumNodesLeft() {
        numNodesLeft++;
    }

    void setNumNodesLeft(int numNodesLeft) {
        numNodesLeft = numNodesLeft;
    }

    int getNumNodesLeft() {
        return numNodesLeft;
    }

    int getNumPredLeft() {
        return numPredecessorsLeft;
    }

    int getNumSucLeft() {
        return numSuccessorsLeft;
    }

    void resetNumPredLeft() {
        numPredecessorsLeft = predecessors.size();
    }

    void resetNumSucLeft() {
        numSuccessorsLeft = successors.size();
    }

    std::vector<preEdge*>::iterator pred_begin() {
        return predecessors.begin();
    }

    std::vector<preEdge*>::iterator pred_end() {
        return predecessors.end();
    }

    std::vector<preEdge*>::iterator succ_begin() {
        return successors.begin();
    }

    std::vector<preEdge*>::iterator succ_end() {
        return successors.end();
    }

    bool isSuccessor(preNode* node) {
        for (preEdge* edge : successors) {
            if (edge->getNode() == node) {
                return true;
            }
        }
        return false;
    }

    bool isPredecessor(preNode* node) {
        for (preEdge* edge : predecessors) {
            if (edge->getNode() == node) {
                return true;
            }
        }
        return false;
    }

    int calculateCriticalPath(Direction direction = Direction::TOPDOWN);
    std::vector<int> getCriticalPath();
    
    void calculateSethiUllmanNumber(preNode *node);
    int calculateSethiUllman();
    int calculateSethiUllmanNumber();

    friend class CombinatorialScheduler;
    friend class DDG;
};


class DDG {
private:
    //Nodes to be scheduled
    std::vector<preNode*> nodes;


public:
    DDG() {}

    DDG(std::vector<preNode*> n) : nodes(n){}

    //duplicar el grafo junto con los nodos
    DDG duplicate() {
        DDG graph;
        for (preNode* node : nodes) {
            preNode* newNode = new preNode(node->getID(), node->getDestSize(), node->getLatency());
            graph.addNode(newNode);
        }
        for (preNode* node : nodes) {
            for (preEdge* edge : node->getSuccessors()) {
                graph.addEdge(graph.getNodes()[node->getID()], graph.getNodes()[edge->getNode()->getID()], edge->getLatency(), edge->getDepType());
            }
        }
        return graph;
    }

    //get key 01010|00001|00001|00001|00000|
    std::string getKey() {
        std::string key = "";
        for (int i = 0; i < (int)nodes.size(); i++) {
            preNode* node = findIndex(i);
            for (int j = 0; j < (int)nodes.size(); j++) {
                //Si j es id de algun sucesor
                if (node->isSuccessor(findIndex(j))) {
                    key += "1";
                } else {
                    key += "0";
                }
            }
            key += "|";
        }
        return key;
    }

    void calculateSethiUllman();

    void addNode(preNode* node, bool inicio = false) {
        if(inicio){
            nodes.insert(nodes.begin(), node);
        }else{
            nodes.push_back(node);
        }
    }

    void setNodes(std::vector<preNode*> nodes) {
        this->nodes = nodes;
        resetIDs();
    }

 
    std::vector<preNode*>& getNodes() {
        return nodes;
    }


    //Reset IDs
    void resetIDs(bool ReassignIDs = true) {
        if(!ReassignIDs) return;

        //asignar nuevos ids
        for (int i = 0; i < (int)nodes.size(); i++) {
            nodes[i]->ID = i;
        }
    }

    //Find index
    int findIndex(preNode* node) {
        for (int i = 0; i < (int)nodes.size(); i++) {
            if (nodes[i] == node) {
                return i;
            }
        }
        return -1;
    }

    preNode* findIndex(int ID) {
        for (int i = 0; i < (int)nodes.size(); i++) {
            if (nodes[i]->ID == ID) {
                return nodes[i];
            }
        }
        return preNode::createEmptyNode();
    }

    //Print the DAG
    void print() {
        std::cout << "digraph DAG {" << std::endl;
        for (preNode* node : nodes) {
            for (preEdge* edge : node->getSuccessors()) {
                std::cout << node->getID() << " -> " << edge->getNode()->getID() <<  std::endl;
            }
        }
        std::cout << "}" << std::endl;
    }


    //Print the DAG by predecessors
    void printByPredecessors() {
        std::cout << "digraph DAG {" << std::endl;
        for (preNode* node : nodes) {
            for (preEdge* edge : node->getPredecessors()) {
                std::cout << edge->getNode()->getID() << " -> " << node->getID() << " [latency=\"" << edge->getLatency() << "\"]" << std::endl;
            }
        }
        std::cout << "}" << std::endl;
    }

    //Add an edge between two nodes
    void addEdge(preNode* source, preNode* destination, unsigned int latency, DepType depType) {
        preEdge* edge = new preEdge(destination, latency, depType);
        source->addSuccessor(edge);
    }

    bool edgeExists(preNode* source, preNode* destination) {
        for (preEdge* edge : source->getSuccessors()) {
            if (edge->getNode() == destination) {
                return true;
            }
        }
        return false;
    }

    std::vector<preNode*> getRootNodes() {
            std::vector<preNode*> rootNodes;
            for (preNode* node : getNodes()) {
                if (node->getPredecessors().size() == 0) {
                    rootNodes.push_back(node);
                }
            }
            return rootNodes;
        }
    
    //Nodos que pertenecen al subgrafo de subNodes
    std::vector<preNode*> getRootNodes(std::vector<bool>& subNodes) {
        //hash map nodo id, padres
        std::unordered_map<int, int> parents;
        for (preNode* node : getNodes()) {
            parents[node->getID()] = 0;
        }
        //Contando los padres de cada nodo
        for (preNode* node : getNodes()) {
            for (preEdge* edge : node->getSuccessors()) {
                parents[edge->getNode()->getID()]++;
            }
        }
        std::vector<preNode*> roots;
        //encontrando los nodos sin padres
        for (preNode* node : getNodes()) {
            if (parents[node->getID()] == 0) {
                roots.push_back(node);
            }
        }
        return roots;
    }
    std::vector<preNode*> getLeafNodes() {
        std::vector<preNode*> leafNodes;
        for (preNode* node : getNodes()) {
            if (node->getSuccessors().size() == 0) {
                leafNodes.push_back(node);
            }
        }
        return leafNodes;
    }

    std::vector<preNode*> getLeafNodes(std::vector<bool>& subNodes) {
        //hash map nodo id, hijos
        std::unordered_map<int, int> children;
        for (preNode* node : getNodes()) {
            children[node->getID()] = 0;
        }
        //Contando los hijos de cada nodo
        for (preNode* node : getNodes()) {
            for (preEdge* edge : node->getPredecessors()) {
                children[edge->getNode()->getID()]++;
            }
        }
        std::vector<preNode*> leafs;
        //encontrando los nodos sin hijos
        for (preNode* node : getNodes()) {
            if (children[node->getID()] == 0) {
                leafs.push_back(node);
            }
        }
        return leafs;
    }

    
    std::vector<unsigned> computeRP();

    static DDG generarArbol(int k, int d, bool invertido);
    static std::vector<preNode*>  getSubGraph(std::vector<preNode*> nodes, Direction direction);
};

class ComparadorPair{
public:
    bool operator() (std::pair<preNode*, int> obj1, std::pair<preNode*, int> obj2) {
        return obj1.second > obj2.second; 
    }
};


class ComparadorCriticalPath {
public:
    bool operator() (preNode* obj1, preNode* obj2) {
        return obj1->calculateCriticalPath() < obj2->calculateCriticalPath();
    }
};

#endif