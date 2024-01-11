from ortools.sat.python import cp_model
import json
import os
import time

# Clase Nodo
class Node:
    def __init__(self, id, latency, defsize):
        self.id = id
        self.latency = latency
        self.defsize = defsize
        self.predecessors = []
        self.successors = []

# Clase Graph
class Graph:
    def __init__(self, nodes):
        #Ordenar los nodos por id
        self.nodes = nodes
        self.nodes.sort(key=lambda x: x.id)


# Leer el archivo de entrada
def read():
    namefile = "graph.json"
    with open(namefile) as file:
        data = json.load(file)
        
        # Leer los nodos
        nodes = []
        for node in data["Nodes"]:
            nodes.append(Node(node["id"], node["latency"], node["dest_size"]))
        
        # Leer los arcos i->j
        for edge in data["Edges"]:
            nodes[edge["i"]].successors.append(nodes[edge["j"]])
            nodes[edge["j"]].predecessors.append(nodes[edge["i"]])
        
        # Crear el grafo
        graph = Graph(nodes)
        return graph

    
def loadGrapFromTxt(filename):
    file = open(filename, "r")
    str = file.read()
    file.close()
    
    lines = str.split(",")
    numNodes = int(lines[0])
    numEdges = int(lines[1])
    idx = 0
    #map de nodos
    nodes = {}
    for i in range(2, 2 + numNodes * 2, 2):
        # latency = std::stoi(lines[i]); //TODO:, por ahora se ignora, se consira latencia 1
        latency = 1
        destSize = int(lines[i + 1])
        nodes.update({idx: [destSize, latency]})
        idx += 1
    # aristas
    init = 2 + numNodes * 2
    edges = []
    for w in range(init, init + numEdges * 2, 2):
        i = int(lines[w])
        j = int(lines[w + 1])
        edges.append([i, j])
    
    # Crear los nodos
    nodes_objects = []
    for i in range(numNodes):
        nodes_objects.append(Node(i, nodes[i][1], nodes[i][0]))
    
    # Crear los arcos
    for edge in edges:
        nodes_objects[edge[0]].successors.append(nodes_objects[edge[1]])
        nodes_objects[edge[1]].predecessors.append(nodes_objects[edge[0]])
    
    # Crear el grafo
    graph = Graph(nodes_objects)
    return graph
    


def solver(graph):
    # Crea el modelo de restricciones.
    model = cp_model.CpModel()

    # Suponiendo que N es el número de nodos/tareas.
    N = len(graph.nodes)
    # print("N: ", N)

    # Crear las variables del modelo.
    x = {}  # x[i][j] es una variable booleana que es verdadera si la tarea i se programa en el tiempo j.
    for i in range(N):
        for j in range(N):
            x[i, j] = model.NewBoolVar(f'x[{i},{j}]')

    # Añade las restricciones de un único bit por fila y por columna.
    for i in range(N):
        model.Add(sum(x[i, j] for j in range(N)) == 1)

    for j in range(N):
        model.Add(sum(x[i, j] for i in range(N)) == 1)

    # Añade las restricciones de precedencia.
    nodes = graph.nodes
    
   # Restricciones de precedencia
    for i in range(N):
        node = nodes[i]
        # print("node.successors: ", node.successors)
        for succ in node.successors:
            j = succ.id
            model.Add(sum([ (x[i, k] - x[j, k])*k for k in range(N)]) < 0)
    
    
     # oc[i][k] es una variable booleana que representa el OR sufijo de x[i][k] por fila.
    oc = {}
    for i in range(N):
        for k in range(N):
            oc[i, k] = model.NewBoolVar(f'oc[{i},{k}]')

    # Definir las restricciones para oc[i][k]
    for i in range(N):
        # Caso base: oc[i][N-1] es igual a x[i][N-1].
        oc[i, N-1] = x[i, N-1]
        # Caso recursivo: oc[i][k] es True si x[i][k] es True o si oc[i][k+1] es True.
        for k in range(N-2, -1, -1):  # Comienza en N-2 y va hacia atrás hasta 0.
            #P1
            model.AddBoolOr([x[i, k], oc[i, k+1]]).OnlyEnforceIf(oc[i, k])
            #P2
            model.AddBoolAnd([x[i, k].Not(), oc[i, k+1].Not()]).OnlyEnforceIf(oc[i, k].Not())
            
    
    # Matriz de bits de vida de nodos l[i][k] con i = nodo y k = tiempo
    l = { (i, k): model.NewBoolVar(f'l[{i},{k}]') for i in range(N) for k in range(N)}
    for i in range(N):
        for k in range(N):
            ors = []
            for succ in nodes[i].successors:
                j = succ.id
                ors.append(oc[j, k] - oc[i, k])
            #P1
            model.Add(sum(ors) > 0).OnlyEnforceIf(l[i, k])
            #P2
            model.Add(sum(ors) <= 0).OnlyEnforceIf(l[i, k].Not())
            
    
    
    max_presurre = sum([node.defsize for node in nodes])
    
    # for i in range(N):
    #     print("node: ", i, "defsize: ", nodes[i].defsize)
    
    s = [ model.NewIntVar(0, max_presurre, f's[{k}]') for k in range(N)]
    for k in range(N):
        ors = []
        for i in range(N):
            ors.append(l[i, k] * nodes[i].defsize)
        model.Add(sum(ors) == s[k])
        
    
    # Crear una variable para el valor maximo de presión
    max_pressure = model.NewIntVar(0, max_presurre, 'max presurre')
    # Encontrar el máximo
    for k in range(N):
        pressure = sum([l[i, k] * nodes[i].defsize for i in range(N)])
        model.Add(max_pressure >= pressure)
    # Minimizar el máximo
    model.Minimize(max_pressure)


    # model.Minimize(max([ max([l[i, k] * nodes[i].defsize for i in range(N)]) for k in range(N)]))
    
    # Crea un solver y resuelve el modelo.
    solver = cp_model.CpSolver()
    status = solver.Solve(model)

    # Verifica si se encontró una solución y, si es así, imprime los resultados.
    if status == cp_model.OPTIMAL or status == cp_model.FEASIBLE:
        # print('Solution:')
        # for i in range(N):
        #     for j in range(N):
        #         if solver.Value(x[i, j]):
        #             print(f'N {i} t: {j}')
        # print("ox:")
        # for i in range(N):
        #     for j in range(N):
        #         print(solver.Value(oc[i, j]), end=" ")
        #     print()

        # print("l:")
        # for i in range(N):
        #     for j in range(N):
        #         print(solver.Value(l[i, j]), end=" ")
        # #     print()
        # # print("s:")
        # for j in range(N):
        #     print(solver.Value(s[j]), end=" ")
        planification = []
        for j in range(N):
            time_j = 0
            for i in range(N):
                if solver.Value(x[i, j]):
                    time_j = i + 1
                    break
            planification.append([j, time_j])
        planification.sort(key=lambda x: x[1])
        max_pressure = solver.Value(max_pressure)
        
        max_pressure = max_pressure
        print(type(max_pressure))
        
        return planification, max_pressure
    else:
        print('No solution found.')

if __name__ == '__main__':
    # grafo = read()
    path = "/home/miguel/codigo/Tesis/Codigo/implementación/src/"
    path_folder = path+"grafos"
    path_result = path+"results.json"
    #read json
    results = []
    with open(path_result) as file:
        results = json.load(file)
    
    files = []
    for i, file in enumerate(os.listdir(path_folder)):
        print("i: ", i)
        if file.endswith(".txt"):
            files.append(file)
    
            grafo2 = loadGrapFromTxt(path_folder + "/" + file)
    
    
        # for node in grafo.nodes:
        #     print(node.id, node.latency, node.defsize)
        
        # for node in grafo.nodes:
        #     print(node.id, "predecessors: ", end="")
        #     for pred in node.predecessors:
        #         print(pred.id, end=" ")
        #     print()
        #     print(node.id, "successors: ", end="")
        #     for succ in node.successors:
        #         print(succ.id, end=" ")
        #     print()
                
        # print()
        #tiempo milisegundos
        start_time = time.time_ns()
        planification, max_pressure = solver(grafo2)
        end_time = time.time_ns()
        
        # print("planification: ", planification)
        # print("max_pressure: ", max_pressure)
        
        for result in results:
            if result["file"] == file:
                for r in result["results"]:
                    algorithm = r["algorithm"]
                    if algorithm == "Solver RP": #Actualizar
                        r["planification"] = planification
                        r["maxRP"] = max_pressure,
                        r["time"] = int((end_time - start_time))
                        break
                #sino existe el algoritmo, agregarlo
                result["results"].append({
                    "algorithm": "Solver RP",
                    "planification": planification,
                    "maxRP": max_pressure,
                    "time": int((end_time - start_time)) 
                })
                break
    #write json
    with open(path_result, 'w') as file:
        json.dump(results, file, indent=2)