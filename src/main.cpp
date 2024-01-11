#include "Combinatorial.h"
#include "IGC_Sched.h"
#include "pruebas.h"

#include <filesystem>
#include <iostream>
namespace fs = std::filesystem;

// Guardar archivo .json con datos: [key="", results=[{algorithm:"",
// planification:[{node:"", cycle:""}], maxRP:""}]]
void saveResults(std::string filename, std::vector<DDG> graphs,
                 std::vector<std::vector<std::vector<std::pair<int, int>>>>
                     planificationsVector,
                 std::vector<std::string> algorithms,
                 std::vector<std::vector<int>> maxRPsVector,
                 std::vector<std::string> files, std::vector<int> optimalRPs,
                 std::vector<std::vector<int>> timesVector) {
  std::ofstream file(filename);
  file << "[";
  for (int i = 0; i < graphs.size(); i++) {
    file << "{";
    file << "\"file\":\"" << files[i] << "\",";
    file << "\"key\":\"" << graphs[i].getKey() << "\",";
    file << "\"optimo\":" << optimalRPs[i] << ",";
    file << "\"results\":[";
    for (int j = 0; j < algorithms.size(); j++) {
      file << "{";
      file << "\"algorithm\":\"" << algorithms[j] << "\",";
      file << "\"maxRP\":" << maxRPsVector[i][j] << ",";
      file << "\"time\":" << int(timesVector[i][j]) << ",";
      file << "\"planification\":[";
      for (int k = 0; k < planificationsVector[i][j].size(); k++) {
        file << "[" << planificationsVector[i][j][k].first << ","
             << planificationsVector[i][j][k].second << "]";
        if (k < planificationsVector[i][j].size() - 1) {
          file << ",";
        }
      }
      file << "],";
      file << "}";
      if (j < algorithms.size() - 1) {
        file << ",";
      }
    }
    file << "]";
    file << "}";
    if (i < graphs.size() - 1) {
      file << ",";
    }
  }
  file << "]";
  file.close();
}

int main() {
  // random Seed
  srand(time(NULL));

  // std::vector<DDG> graphs = generarGrafosAciclicos(7, 8, 1, 1);
  // std::cout << "Grafo generado" << std::endl;
  // DDG graphAux = graphs[0];

  std::vector<std::string> path_dags;
  std::vector<std::string> algorithms_names = {
      "B&B RP", "List SU",
      "List SU RP"}; // IMPORTANT: the order of names must be the same as the
                     // order of the execution of the algorithms in the main
                     // function
  std::vector<DDG> graphs = std::vector<DDG>();

  // Listar archivos dado el path
  std::string path = "./grafos";
  std::vector<std::string> files = std::vector<std::string>();
  try {
    for (const auto &entry : fs::directory_iterator(path)) {
      // std::cout << entry.path() << std::endl; // Muestra la ruta de cada
      // archivo
      DDG graphAux = loadGraph("./grafos/" + entry.path().filename().string());
      files.push_back(entry.path().filename().string());
      graphs.push_back(graphAux);
    }
  } catch (fs::filesystem_error &e) {
    std::cerr << e.what() << std::endl; // Manejo de errores en caso de que la
                                        // ruta no exista o sea inaccesible
  }

  // graphs = std::vector<DDG>();
  // DDG graphAux = loadGraph("./grafos/g408.txt");
  // graphs.push_back(graphAux);
  // save("graph.json", graphAux, {}, {},
  //        {});

  std::vector<std::vector<std::vector<std::pair<int, int>>>>
      planificationsVector;
  std::vector<std::vector<int>> maxRPsVector;
  std::vector<std::vector<int>> timesVector;
  std::vector<int> optimalRPs;

  int index = 0;
  for (DDG graphAux : graphs) {
    std::vector<std::vector<std::pair<int, int>>> planifications;
    std::vector<int> maxRPs;
    std::vector<int> times;

    // B&B for Register Pressure. SI "B&B RP" esta en algorithms_names, se
    // ejecuta
    if (std::find(algorithms_names.begin(), algorithms_names.end(), "B&B RP") !=
        algorithms_names.end()) {
      // B&B for Register Pressure
      Priority priority = Priority::SU;
      Sched sched = Sched(&graphAux, Priority::SU);
      int ciclos1 = 0;

      // Execution of the algorithm
      auto start = std::chrono::high_resolution_clock::now();
      std::vector<preNode *> nodes = sched.enumerate(0, ciclos1);
      auto finish = std::chrono::high_resolution_clock::now();

      std::vector<std::pair<int, int>> s = std::vector<std::pair<int, int>>();
      for (preNode *node : nodes) {
        //   std::cout << node->getID() << ":" << node->timeStep << " ";
        s.push_back(std::pair<int, int>(node->getID(), node->timeStep));
      }
      // std::cout << std::endl;
      planifications.push_back(s);

      int maxRP = 0;
      std::vector<int> pressure = sched.bestPressure;

      for (unsigned int i = 0; i < pressure.size(); i++) {
        // std::cout << pressure[i] << " ";
        if (pressure[i] > maxRP) {
          maxRP = pressure[i];
        }
      }
      // std::cout << std::endl;

      // std::cout << "Max RP B&B: " << maxRP << std::endl;
      optimalRPs.push_back(maxRP);
      maxRPs.push_back(maxRP);
      times.push_back(
          std::chrono::duration_cast<std::chrono::nanoseconds>(finish - start)
              .count());
    }
    //--------------------------------------------------------------------------------

    // List Scheduling with Sethi Ullman heuristic. Si "List SU" esta en
    // algorithms_names, se ejecuta
    if (std::find(algorithms_names.begin(), algorithms_names.end(),
                  "List SU") != algorithms_names.end()) {

      // List Scheduling with Sethi Ullman heuristic
      SethiUllmanQueue suq = SethiUllmanQueue(&graphAux);

      // Execution of the algorithm
      auto start = std::chrono::high_resolution_clock::now();
      std::vector<preNode *> nodes = suq.listScheduling();
      auto finish = std::chrono::high_resolution_clock::now();

      std::vector<std::pair<int, int>> s = std::vector<std::pair<int, int>>();
      // std::cout << "SU: " << std::endl;
      for (int i = 0; i < nodes.size(); i++) {
        //   std::cout << nodesOrdered2[i]->getID() << ":" <<
        //   nodesOrdered2[i]->timeStep << " ";
        s.push_back(std::pair<int, int>(nodes[i]->getID(), nodes[i]->timeStep));
      }
      // std::cout << std::endl;
      planifications.push_back(s);

      for (preNode *node : nodes) {
        node->setNumNodesLeft(node->getNumSucLeft());
      }
      int maxRP = 0;
      int rp = 0;
      std::vector<int> pressure = std::vector<int>(nodes.size(), 0);
      for (preNode *node : nodes) {
        int i = node->timeStep - 1;
        int j = i + 1;
        for (preEdge *suc : node->getSuccessors()) {
          int k = suc->getNode()->timeStep - 1;
          if (k > j) {
            j = k;
          }
        }
        for (int k = i; k < j; k++) {
          pressure[k] += node->getDestSize();
        }
      }
      // std::cout << "Pressure: ";
      for (int i = 0; i < pressure.size(); i++) {
        // std::cout << pressure[i] << " ";
        if (pressure[i] > maxRP) {
          maxRP = pressure[i];
        }
      }
      // std::cout << std::endl;
      // std::cout << "Max RP SU: " << maxRP << std::endl;
      maxRPs.push_back(maxRP);
      times.push_back(
          std::chrono::duration_cast<std::chrono::nanoseconds>(finish - start)
              .count());
    }
    //--------------------------------------------------------------------------------

    // List Scheduling with Sethi Ullman heuristic and RP reduction. Si "List SU
    // RP" esta en algorithms_names, se ejecuta
    if (std::find(algorithms_names.begin(), algorithms_names.end(),
                  "List SU RP") != algorithms_names.end()) {

      // List Scheduling with Sethi Ullman heuristic and RP reduction
      SethiUllmanQueue suq = SethiUllmanQueue(&graphAux);

      // Execution of the algorithm
      auto start = std::chrono::high_resolution_clock::now();
      std::vector<preNode *> nodesOrdered = suq.listSchedulingRP();
      auto finish = std::chrono::high_resolution_clock::now();

      std::vector<std::pair<int, int>> s = std::vector<std::pair<int, int>>();
      // std::cout << "SU RP: " << std::endl;
      for (int i = 0; i < nodesOrdered.size(); i++) {
        //   std::cout << nodesOrdered3[i]->getID() << ":" <<
        //   nodesOrdered3[i]->timeStep << " ";
        s.push_back(std::pair<int, int>(nodesOrdered[i]->getID(),
                                        nodesOrdered[i]->timeStep));
      }
      // std::cout << std::endl;
      planifications.push_back(s);

      for (preNode *node : nodesOrdered) {
        node->setNumNodesLeft(node->getNumSucLeft());
      }
      int maxRP = 0;
      int rp = 0;
      std::vector<int> pressure = std::vector<int>(nodesOrdered.size(), 0);
      for (preNode *node : nodesOrdered) {
        int i = node->timeStep - 1;
        int j = i + 1;
        for (preEdge *suc : node->getSuccessors()) {
          int k = suc->getNode()->timeStep - 1;
          if (k > j) {
            j = k;
          }
        }
        for (int k = i; k < j; k++) {
          pressure[k] += node->getDestSize();
        }
      }
      // std::cout << "Pressure: ";
      for (int i = 0; i < pressure.size(); i++) {
        // std::cout << pressure[i] << " ";
        if (pressure[i] > maxRP) {
          maxRP = pressure[i];
        }
      }
      // std::cout << std::endl;

      // std::cout << "Max RP SU RP: " << maxRP << std::endl;
      maxRPs.push_back(maxRP);
      times.push_back(
          std::chrono::duration_cast<std::chrono::nanoseconds>(finish - start)
              .count());
    }

    //--------------------------------------------------------------------------------

    // B&B for Latency. Si "List ILP" esta en algorithms_names, se ejecuta
    if (std::find(algorithms_names.begin(), algorithms_names.end(),
                  "List ILP") != algorithms_names.end()) {
      // List Scheduling with Latency heuristic
      LatencyListScheduler ls = LatencyListScheduler(&graphAux);
      int ciclos4 = 0;

      // Execution of the algorithm
      auto start = std::chrono::high_resolution_clock::now();
      std::vector<preNode *> nodes = ls.listScheduling(ciclos4);
      auto finish = std::chrono::high_resolution_clock::now();

      std::vector<std::pair<int, int>> s = std::vector<std::pair<int, int>>();
      // std::cout << "List: " << std::endl;
      for (int i = 0; i < nodes.size(); i++) {
        //   std::cout << nodesOrdered4[i]->getID() << ":"
        //             << nodesOrdered4[i]->timeStep << " ";
        s.push_back(std::pair<int, int>(nodes[i]->getID(), nodes[i]->timeStep));
      }

      // std::cout << std::endl;
      planifications.push_back(s);

      for (preNode *node : nodes) {
        node->setNumNodesLeft(node->getNumSucLeft());
      }
      int maxRP = 0;
      int rp = 0;
      std::vector<int> pressure = std::vector<int>(nodes.size(), 0);
      for (preNode *node : nodes) {
        int i = node->timeStep - 1;
        int j = i + 1;
        for (preEdge *suc : node->getSuccessors()) {
          int k = suc->getNode()->timeStep - 1;
          if (k > j) {
            j = k;
          }
        }
        for (int k = i; k < j; k++) {
          pressure[k] += node->getDestSize();
        }
      }

      // std::cout << "Pressure: ";
      for (int i = 0; i < pressure.size(); i++) {
        // std::cout << pressure[i] << " ";
        if (pressure[i] > maxRP) {
          maxRP = pressure[i];
        }
      }
      // std::cout << std::endl;
      // std::cout << "Max RP List: " << maxRP3 << std::endl;
      // std::cout << std::endl;
      maxRPs.push_back(maxRP);
      times.push_back(
          std::chrono::duration_cast<std::chrono::nanoseconds>(finish - start)
              .count());
    }
    // std::cout << "B&B: " << maxRP1 << " SU: " << maxRP << " SU RP: " <<
    // maxRP2
    //           << std::endl;

    if ((maxRPs[0] != maxRPs[1]) || (maxRPs[0] != maxRPs[2])) {
      index++;
      if (index == 10) {
        std::cout << "Grafo: " << graphAux.getKey() << std::endl;
        save("graph.json", graphAux, planifications, algorithms_names, maxRPs);
      }
    }
    planificationsVector.push_back(planifications);
    maxRPsVector.push_back(maxRPs);
    timesVector.push_back(times);
  }

  saveResults("results.json", graphs, planificationsVector, algorithms_names,
              maxRPsVector, files, optimalRPs, timesVector);

  std::cout << "Fin del programa" << std::endl;

  return 0;
}
