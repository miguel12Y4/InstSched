#include "pruebas.h"
#include <cstring>
#include <fstream>
#include <iostream>
#include <random>
#include <unordered_set>
#include <vector>

// Calcular presion de registros dada un vector de nodos
std::vector<unsigned char> calculatePressure(std::vector<preNode *> nodes,
                                             bool includeLatency, int ciclos) {
  std::vector<unsigned char> pressure(includeLatency ? ciclos : nodes.size(),
                                      0);
  for (int i = 0; i < nodes.size(); i++) {
    preNode *node = nodes[i];
    int timeStep = node->timeStep - 1; // TODO: ver si hay que restar 1
    int deadTime = timeStep;
    for (preEdge *edge : node->getSuccessors()) {
      if (edge->getNode()->timeStep > deadTime) {
        deadTime = edge->getNode()->timeStep;
      }
    }
    if (node->getSuccessors().size() == 0) {
      if (includeLatency) {
        for (int j = timeStep; j < timeStep + node->getLatency(); j++) {
          pressure[j] += node->getDestSize();
        }
      } else {
        pressure[timeStep] = node->getDestSize();
      }
    } else {
      for (int j = timeStep; j < deadTime; j++) {
        pressure[j] += node->getDestSize();
      }
    }
  }
  return pressure;
}

// Funcion para verificar si el grado es aciclico
bool isAcyclicUtil(preNode *v, bool visited[], bool *recStack) {
  if (visited[v->getID()] == false) {
    // Mark the current node as visited and part of recursion stack
    visited[v->getID()] = true;
    recStack[v->getID()] = true;

    // Recur for all the vertices adjacent to this vertex
    std::vector<preEdge *> successors = v->getSuccessors();
    for (int i = 0; i < successors.size(); i++) {
      preNode *successor = successors[i]->getNode();
      if (!visited[successor->getID()] &&
          !isAcyclicUtil(successor, visited, recStack))
        return false; // Si se detecta un ciclo, devuelve false.
      else if (recStack[successor->getID()])
        return false; // Si se detecta un ciclo, devuelve false.
    }
  }
  recStack[v->getID()] = false; // remove the vertex from recursion stack
  return true; // Si no se detecta ningún ciclo, devuelve true.
}

DDG generateAcyclicGraph(int numNodes, int numEdges) {
  if (numEdges > numNodes * (numNodes - 1) / 2) {
    throw std::invalid_argument("Too many edges for the given number of nodes. "
                                "Can't guarantee an acyclic graph.");
  }

  DDG graph;

  // 1. Crear nodos aleatorios
  for (int i = 0; i < numNodes; i++) {
    graph.addNode(new preNode(i, rand() % 4 + 1, rand() % 10 + 1));
  }

  std::vector<preNode *> nodes = graph.getNodes();

  // 2. Conectar los nodos de manera que no formen un ciclo
  int edgesAdded = 0;
  while (edgesAdded < numEdges) {
    int i = rand() % numNodes;
    int j = rand() % numNodes;

    // Asegurarse de que i < j para mantenerlo acíclico
    if (i >= j) {
      std::swap(i, j);
    }

    // Evitar la adición de múltiples bordes entre los mismos nodos
    if (i != j && !graph.edgeExists(nodes[i], nodes[j])) {
      graph.addEdge(nodes[i], nodes[j], nodes[i]->getLatency(), DepType::RAW);
      edgesAdded++;
    }
  }

  // 3. Verificar que el grafo resultante es acíclico
  for (int i = 0; i < graph.getRootNodes().size(); i++) {
    bool *visited = new bool[numNodes];
    bool *recStack = new bool[numNodes];
    for (int j = 0; j < numNodes; j++) {
      visited[j] = false;
      recStack[j] = false;
    }

    if (isAcyclicUtil(graph.getRootNodes()[i], visited, recStack)) {
      // std::cout << "Graph is acyclic" << std::endl;
    } else {
      // std::cout << "Graph contains cycle" << std::endl;
    }
    delete[] visited;
    delete[] recStack;
  }

  return graph;
}

// n: numero de nodos, k: numero de raices, d: nodos internos
DDG generarGrafo(int n, int k, int d) {
  if ((n - 2 * k) % d != 0) {
    n = n + d - (n - 2 * k) % d;
    // std::cout << "n: " << n << std::endl;
  }

  DDG G;

  // crear nodos
  for (int i = 0; i < n; i++) {
    int latency = rand() % 5 + 1;
    int destSize = rand() % 10 + 1;
    G.addNode(new preNode(i, destSize, latency));
  }

  std::vector<preNode *> roots;
  std::vector<preNode *> leaves;
  for (int i = 0; i < k; i++) {
    roots.push_back(G.getNodes()[i]);
    leaves.push_back(G.getNodes()[n - i - 1]);
  }

  // Enlazar raíces con nodos
  for (int j = k; j < k + d; j++) {
    for (int i = 0; i < k; i++) {
      G.addEdge(roots[i], G.getNodes()[j], roots[i]->getLatency(),
                DepType::RAW);
      // std::cout << i << " " << j << std::endl;
    }
  }

  // Enlazar nodos con otros nodos
  int des = 0;
  for (int i = k; i < n - k - d; i++) {
    int dd = d - des;
    for (int j = 0; j < d; ++j) {
      if (i + dd + j >= n || i >= n) {
        break;
      }
      G.addEdge(G.getNodes()[i], G.getNodes()[i + dd + j],
                G.getNodes()[i]->getLatency(), DepType::RAW);
      // std::cout << i << " " << i + dd + j << std::endl;
    }
    des += 1;
    if (des == d) {
      des = 0;
    }
  }

  // Enlazar nodos con hojas
  des = 0;
  for (int i = n - k - d; i < n - k; i++) {
    int dd = d - des;
    for (int j = 0; j < k; ++j) {
      if (i + dd + j >= n || i >= n) {
        break;
      }
      G.addEdge(G.getNodes()[i], G.getNodes()[i + dd + j],
                G.getNodes()[i]->getLatency(), DepType::RAW);
      // std::cout << i << " " << i + dd + j << std::endl;
    }
    des += 1;
    if (des == d) {
      des = 0;
    }
  }
  return G;
}

void save(std::string filename, DDG graph,
          std::vector<std::vector<std::pair<int, int>>> nodesOrdered,
          std::vector<std::string> names, std::vector<int> maxRP) {

  std::ofstream file;
  file.open(filename);
  file << "{" << std::endl;
  file << "\"Nodes\":[" << std::endl;
  // Datos de cada nodo
  for (int i = 0; i < graph.getNodes().size(); i++) {
    preNode *node = graph.getNodes()[i];
    file << "{" << std::endl;
    file << "\"id\":" << node->getID() << "," << std::endl;
    file << "\"latency\":" << node->getLatency() << "," << std::endl;
    file << "\"dest_size\":" << node->getDestSize() << "" << std::endl;
    file << "}";
    if (i < graph.getNodes().size() - 1) {
      file << ",";
    }
  }
  file << "]," << std::endl;

  file << "\"Edges\":[" << std::endl;
  // Crear sethi ullman queue solo para obtener el número de registros sethi
  // ullman
  std::string strEdges = "";
  for (int i = 0; i < graph.getNodes().size(); i++) {
    preNode *node = graph.getNodes()[i];
    for (int j = 0; j < node->getSuccessors().size(); j++) {
      preEdge *edge = node->getSuccessors()[j];
      strEdges += "{\n";
      strEdges += "\"i\":" + std::to_string(node->getID()) + ",\n";
      strEdges += "\"j\":" + std::to_string(edge->getNode()->getID()) + "\n";
      strEdges += "},";
    }
  }
  // Eliminar la última coma
  strEdges = strEdges.substr(0, strEdges.size() - 1);
  file << strEdges;
  file << "]," << std::endl;
  file << "\"Data\":[" << std::endl;
  for (int i = 0; i < maxRP.size(); i++) {
    file << "{" << std::endl;
    file << "\"name\":\"" << names[i] << "\"," << std::endl;
    file << "\"maxRP\":" << maxRP[i] << std::endl;
    file << "}";
    if (i < maxRP.size() - 1) {
      file << ",";
    }
  }
  file << "]," << std::endl;
  
  file << "\"Results\":[" << std::endl;

  // Escribir planificaciones { "name": { "nodes": [ (node_id, cycle) ] } }
  int i = 0;
  for (std::vector<std::pair<int, int>> nodes : nodesOrdered) {
    file << "{" << std::endl;
    file << "\"name\":\"" << names[i] << "\"," << std::endl;
    file << "\"nodes\":[";
    for (int i = 0; i < nodes.size(); i++) {
      file << "[" << nodes[i].first << "," << nodes[i].second << "]";
      if (i < nodes.size() - 1) {
        file << ",";
      }
    }
    file << "]" << std::endl;
    if (i < nodesOrdered.size() - 1) {
      file << "}," << std::endl;
    } else {
      file << "}" << std::endl;
    }
    i++;
  }

  file << "]" << std::endl;

  file << "}" << std::endl;
  file.close();
}


/*
def insert(r1=5, r2=50, n_iter=5, nro_grafos=10):

    insertados = 0
    no_insertados = 0

    nros = [ i for i in range(r1, r2+1) ]

    for nro_nodes in nros:
        rango1 = nro_nodes - 1
        rango2 = (nro_nodes * (nro_nodes - 1) // 2)
        chunks_edges = (rango2 - rango1) // n_iter
        prev = rango1
        for i in range(n_iter):
            for j in range(nro_grafos * (i+1)):

            int nro_edges = rand() % (prev + chunks_edges - prev + 1) + prev; // rango entre prev y prev + chunks_edges

          std::cout << "N=" << nro_nodes << ", M=" << nro_edges << std::endl;
          DDG nuevo_grafo = generateAcyclicGraph(nro_nodes, nro_edges);
          grafos.push_back(nuevo_grafo);
        prev = prev + chunks_edges;


                nro_edges = random.randint(prev, prev + chunks_edges)
                print(f"rango: {prev} - {prev + chunks_edges}")
                print(f"rango: {rango1} - {rango2}")
                print(f"N={nro_nodes}, M={nro_edges}")
                nuevo_grafo = generar_grafo_aciclico(nro_nodes, nro_edges)

                #Buscar si existe un grafo con la misma llave
                resultados = collection.find({"Key": nuevo_grafo.key})
                encontrado = False
                for resultado in resultados:
                    if resultado:
                        # print("Ya existe la key")
                        if True: # Verificar si el grafo es el mismo
                            iguales = True
                            for e in resultado["Edges"]:
                                is_new = False
                                for edge in nuevo_grafo.edges:
                                    if e["i"] == edge.i and e["j"] == edge.j:
                                        is_new = True
                                if not is_new:
                                    # print(f"La arista {e['i']}-{e['j']} no existe en el grafo nuevo")
                                    iguales = False
                                    break
                            if iguales:
                                # print("El grafo es el mismo")
                                encontrado = True
                        
                if encontrado:
                    # print("El grafo ya existe")
                    print("-")
                    no_insertados += 1
                else:
                    collection.insert_one({
                        "Nodes": [vars(node) for node in nuevo_grafo.nodes],
                        "Edges": [vars(edge) for edge in nuevo_grafo.edges],
                        "Key": nuevo_grafo.key,
                    })
                    insertados += 1
                    print("+")
            prev = prev + chunks_edges
    print(f"Se insertaron {insertados} grafos")
    print(f"No se insertaron {no_insertados} grafos")

*/

//Función en python para generar grafos aciclicos
std::vector<DDG> generarGrafosAciclicos(int r1, int r2, int n_iter, int nro_grafos) {
  std::vector<DDG> grafos;
  std::vector<int> nros;
  for (int i = r1; i <= r2; i++) {
    nros.push_back(i);
  }

  for (int nro_nodes : nros) {
    
    int rango1 = nro_nodes - 1; //Rango de aristas desde nro_nodes hasta nro_nodes * (nro_nodes - 1) / 2
    int rango2 = (nro_nodes * (nro_nodes - 1) / 2);

    int chunks_edges = (rango2 - rango1) / n_iter;
    int chunks = nro_grafos * (n_iter + 1);
    int prev = rango1;
    for (int i = 0; i < n_iter; i++) {
      prev = rango1;
      for (int j = 0; j < nro_grafos * (i + 1); j++) {
        std::cout << std::endl;
        std::cout << "rango: " << rango1 << " - " << rango2 << std::endl;
        std::cout << "rango: " << prev << " - " << prev + chunks_edges << std::endl;
        
        int rr1 = prev;
        int rr2 = prev + chunks_edges;
        if (rr2 > rango2) {
          rr2 = rango2;
        }
        // Random entre rr1 y rr2
        std::cout << "rr: " << rr1 << " - " << rr2 << std::endl;
        int nro_edges = rand() % (rr2 - rr1 + 1) + rr1;
       
        std::cout << "N=" << nro_nodes << ", M=" << nro_edges << std::endl;
        DDG nuevo_grafo = generateAcyclicGraph(nro_nodes, nro_edges);
        grafos.push_back(nuevo_grafo);
        prev = prev + rr2;
      }
    }
  }
  return grafos;
}
#include <sstream>
#include <iterator>
#include <string>

DDG loadGraph(std::string filename) {
  std::ifstream file(filename);
  std::string str((std::istreambuf_iterator<char>(file)),
                  std::istreambuf_iterator<char>());
  file.close();

  DDG graph;

  //format file: n,m,latency,dest_size,latency,dest_size,...,i,j,i,j,i,j,...,
  std::vector<std::string> lines;
  std::string line = "";

  // Dividir la cadena por comas
  std::istringstream ss(str);
  std::string token;

  while (std::getline(ss, token, ',')) {
    lines.push_back(token);
  }
  int numNodes = 0;
  int numEdges = 0;
  numNodes = std::stoi(lines[0]);
  numEdges = std::stoi(lines[1]);
  int idx = 0;
  //map de nodos
  int latency = 0;
  int destSize = 0;
  std::unordered_map<int, preNode *> nodes;
  for (int i = 2; i < 2 + numNodes * 2; i += 2) {
    latency = std::stoi(lines[i]); //TODO:, por ahora se ignora, se consira latencia 1
    // latency = 1;
    destSize = std::stoi(lines[i + 1]);
    preNode* n =  new preNode(idx, destSize, latency);
    nodes[idx] = n;
    graph.addNode(n);
    idx++;
  }
  // aristas
  int i = 0;
  int j = 0;
  int init = 2 + numNodes * 2;
  for (int w = init; w < init + numEdges * 2; w += 2) {
    i = std::stoi(lines[w]);
    j = std::stoi(lines[w + 1]);
    graph.addEdge(nodes[i], nodes[j], nodes[i]->getLatency(), DepType::RAW);
  }

  return graph;
}
