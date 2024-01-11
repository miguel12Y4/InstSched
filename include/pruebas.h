#ifndef PRUEBAS_H
#define PRUEBAS_H

#include "Base.h"
#include "IGC_Sched.h"
#include <fstream>
#include <vector>
#include <ctime>
#include <string>

bool isAcyclicUtil(preNode *v, bool visited[], bool *recStack);

DDG generateAcyclicGraph(int numNodes, int numEdges);

DDG generarGrafo(int n, int k, int d);

void save(std::string filename, DDG graph, std::vector<std::vector<std::pair<int, int>>> nodesOrdered, std::vector<std::string> names, std::vector<int> maxRP);

DDG loadGraph(std::string filename);

std::vector<DDG> generarGrafosAciclicos(int r1, int r2, int n_iter, int nro_grafos);

std::vector<unsigned char> calculatePressure(std::vector<preNode*> nodes, bool includeLatency = false, int ciclos = 0);

std::vector<unsigned char> calculateOccupation(std::vector<preNode*> nodes, bool includeLatency = false, int ciclos = 0);

#endif // PRUEBAS_H