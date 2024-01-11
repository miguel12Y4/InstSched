import pymongo
import networkx as nx
import random
import json

# Definir la clase Nodo
class Nodo:
    def __init__(self, id, latency, dest_size):
        self.id = id
        self.latency = latency
        self.dest_size = dest_size
        self.successors = []
        self.predecessors = []
        
    #eqaul
    def __eq__(self, other):
        return self.id == other.id and self.latency == other.latency and self.dest_size == other.dest_size
    
    def add_successor(self, node):
        self.successors.append(node)
        node.predecessors.append(self)
    
    def add_predecessor(self, node):
        self.predecessors.append(node)
        node.successors.append(self)
    

# Definir la clase Arista
class Arista:
    def __init__(self, i, j):
        self.i = i
        self.j = j
    # Sobrecargar el operador == para poder comparar aristas
    def __eq__(self, other):
        return self.i == other.i and self.j == other.j
    

# Definir la clase Grafo
class Grafo:
    def __init__(self, nodes, edges):
        self.nodes = nodes
        self.edges = edges

        # Crear una matriz de adyacencia
        num_nodes = len(nodes)
        self.matriz_adyacencia = [[0] * num_nodes for _ in range(num_nodes)]
        for edge in edges:
            self.matriz_adyacencia[edge.i][edge.j] = 1
        
        str_matriz_key = ""
        edges.sort(key=lambda x: x.i)
        for i in range(len(nodes)):
            for j in range(len(nodes)):
                if i == j:
                    str_matriz_key += "0"
                else:
                    is_edge = False
                    for edge in edges:
                        if edge.i == i and edge.j == j:
                            is_edge = True
                    if is_edge:
                        str_matriz_key += "1"
                    else:
                        str_matriz_key += "0"
            str_matriz_key += "|"
        
        self.key = str_matriz_key

# Función para generar un grafo acíclico en Python
def generar_grafo_aciclico(num_nodes, num_edges):
    if num_edges > num_nodes * (num_nodes - 1) / 2:
        raise ValueError("Demasiadas aristas para el número de nodos dado. No se puede garantizar un grafo acíclico.")

    nodes = []
    edges = []

    # 1. Crear nodos aleatorios
    for i in range(num_nodes):
        nodes.append(Nodo(i, random.randint(1, 4), random.randint(1, 10)))

    # 2. Conectar los nodos de manera que no formen un ciclo
    edges_added = 0
    while edges_added < num_edges:
        i = random.randint(0, num_nodes - 1)
        j = random.randint(0, num_nodes - 1)

        # Asegurarse de que i < j para mantenerlo acíclico
        if i >= j:
            i, j = j, i

        # Evitar la adición de múltiples bordes entre los mismos nodos
        if i != j and Arista(i, j) not in edges:
            edges.append(Arista(i, j))
            edges_added += 1

    # 3. Verificar que el grafo resultante es acíclico
    visited = [False] * num_nodes
    rec_stack = [False] * num_nodes
    for root_node in range(num_nodes):
        if not visited[root_node]:
            if not is_acyclic_util(root_node, visited, rec_stack, nodes, edges):
                # print("El grafo contiene un ciclo")
                pass

    return Grafo(nodes, edges)

# Función auxiliar para verificar si el grafo es acíclico
def is_acyclic_util(node_id, visited, rec_stack, nodes, edges):
    visited[node_id] = True
    rec_stack[node_id] = True

    for edge in edges:
        if edge.i == node_id:
            next_node_id = edge.j
            if not visited[next_node_id]:
                if not is_acyclic_util(next_node_id, visited, rec_stack, nodes, edges):
                    return False
            elif rec_stack[next_node_id]:
                return False

    rec_stack[node_id] = False
    return True

# Función para generar un grafo con parámetros n, k y d
def generar_grafo(n, k, d):
    if (n - 2 * k) % d != 0:
        n = n + d - (n - 2 * k) % d

    nodes = []
    edges = []

    # Crear nodos
    for i in range(n):
        latency = random.randint(1, 5)
        dest_size = random.randint(1, 10)
        nodes.append(Nodo(i, latency, dest_size))

    roots = nodes[:k]
    leaves = nodes[-k:]

    # Enlazar raíces con nodos
    for j in range(k, k + d):
        for i in range(k):
            edges.append(Arista(roots[i].id, nodes[j].id))

    # Enlazar nodos con otros nodos
    des = 0
    for i in range(k, n - k - d):
        dd = d - des
        for j in range(d):
            if i + dd + j >= n or i >= n:
                break
            edges.append(Arista(nodes[i].id, nodes[i + dd + j].id))
        des += 1
        if des == d:
            des = 0

    # Enlazar nodos con hojas
    des = 0
    for i in range(n - k - d, n - k):
        dd = d - des
        for j in range(k):
            if i + dd + j >= n or i >= n:
                break
            edges.append(Arista(nodes[i].id, nodes[i + dd + j].id))
        des += 1
        if des == d:
            des = 0

    return Grafo(nodes, edges)

# Ejemplo de uso
# grafo = generar_grafo(14, 1, 5)  # Generar un grafo con 10 nodos, 2 raíces y 3 nodos internos
# grafo = generar_grafo_aciclico(10, 15)  # Generar un grafo acíclico con 10 nodos y 15 aristas
# print("Nodos:")
# for node in grafo.nodes:
#     print(f"ID: {node.id}, latency: {node.latency}, Dest Size: {node.dest_size}")
# print("\nAristas:")
# for edge in grafo.edges:
#     print(f"De {edge.i} a {edge.j}")

# # Acceder a la matriz de adyacencia
# print("\nMatriz de Adyacencia:")
# for fila in grafo.matriz_adyacencia:
#     print(fila)



# Conectarse a la base de datos MongoDB
client = pymongo.MongoClient("mongodb://localhost:27017/", ssl=False)
db = client["mi_base_de_datos"]

# Crear una colección (equivalente a una tabla en SQL)
collection = db["grafos"]



def generate(r1=5, r2=50, n_iter=5, nro_grafos=10):

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

def save():
    # Leer un objeto de la colección y guardarlo como json usando " en vez de ' e identado con 4 espacios
    resultado = collection.find_one({})
    resultado["_id"] = str(resultado["_id"])
    print(resultado)
    with open("grafo.json", "w") as f:
        json.dump(resultado, f, indent=4)
    

def select_and_save(path_folder="/home/miguel/codigo/Tesis/Codigo/implementación/src/grafos/"):
    
    #verificar si existe la carpeta
    import os
    if not os.path.exists(path_folder):
        os.makedirs(path_folder)
    
    # rnd = random.randint(0, resultado)
    #entre 10 y 15
    # resultados = collection.find({"Nodes": {"$size": {"$gte": 10, "$lte": 11}}})
    resultados = collection.aggregate([
        {
            "$addFields": {
                "tamañoNodes": { "$size": "$Nodes" },
                # "tamañoEdges": { "$size": "$Edges" }
            }
        },
        {
            "$match": {
                "tamañoNodes": { "$gte": 2, "$lte": 12 },
                # "tamañoEdges": { "$gte": 20, "$lte": 25 }
            }
        }
    ])
    
    # print("Resultado: ", len(list(resultados)))
    
    #recorrer todos los documentos de la colección
    for i, resultado in enumerate(resultados):
        print("Grafo " + str(i))
        str_data = ""
        str_data += str(len(resultado["Nodes"])) + ","
        str_data += str(len(resultado["Edges"])) + ","
        for node in resultado["Nodes"]:
            str_data += str(node["latency"]) + ","
            str_data += str(node["dest_size"]) + ","
        for edge in resultado["Edges"]:
            str_data += str(edge["i"]) + ","
            str_data += str(edge["j"]) + ","
        str_data = str_data[:-1]
        #Save as graph.txt
        print("Guardando grafo " + str(i))
        with open(path_folder + "g" + str(i) + ".txt", "w") as f:
            f.write(str_data)


def select(nro_nodes, nro_edges):
    #Leer todos los documentos de la colección que tenga 10 nodos
    resultados = collection.count_documents({"Nodes": {"$size": nro_nodes}})
    print("N=" + str(nro_nodes) + " -> " + str(resultados))
    
    #Leer todos los documentos de la colección que tengan 10 nodos y 20 aristas (Nodes y Edges son listas de diccionarios)
    resultados = collection.count_documents({"Nodes": {"$size": nro_nodes}, "Edges": {"$size": nro_edges}})
    print("N=" + str(nro_nodes) + ", M=" + str(nro_edges) + " -> " + str(resultados))
    
    #Contar todos los documentos de la colección
    resultado = collection.count_documents({})
    print("Total= " + str(resultado))


def read_json(path):
    # Leer graph.json
    with open(path) as json_file:
        data = json.load(json_file)
    
    Nodes = []
    Edges = []
    for node in data["Nodes"]:
        Nodes.append(Nodo(node["id"], node["latency"], node["dest_size"]))
    for edge in data["Edges"]:
        Edges.append(Arista(edge["i"], edge["j"]))
    
    g = Grafo(Nodes, Edges)
    results = []
    names = []
    if 'Results' not in data:
        return g, results, names
    for p in data['Results']:
        array = {}
        nodes = p['nodes']
        names.append(p['name'])
        # print(p['name'])
        for node_data in nodes:
            node_id, cycle = node_data
            array.update({node_id: cycle})
        results.append(array)
    
    d = []
    for dd in data["Data"]:
        d.append({
            "name": dd["name"],
            "maxRP":dd["maxRP"],
        })
    print("Data: ", d)
    return g, results, names, d

def save_in_mongo(g, results, names, data):
    #actualizar documento cuya key sea igual a la key del grafo (si existe)
    # collection.insert_one({
    #     "Nodes": [vars(node) for node in g.nodes],
    #     "Edges": [vars(edge) for edge in g.edges],
    #     "Key": g.key,
    #     "Results": [
    #         {
    #             "name": names[i],
    #             "nodes": results[i]
    #         } for i in range(len(results))
    #     ]
    # })
    print(g.key)
    print("Actualizando documento...")
    count = collection.count_documents({"Key": g.key})
    if count == 0:
        print("Insertando documento...")
        collection.insert_one({
            "Nodes": [vars(node) for node in g.nodes],
            "Edges": [vars(edge) for edge in g.edges],
            "Key": g.key,
            "Results": [
                {
                    "name": names[i],
                    "nodes": results[i]
                } for i in range(len(results))
            ],
            "Data": data
        })
        print("Documento insertado")
    else:
        print("Actualizando documento...")
        print("results: ", results)
        # Actualizar documento con key "Results"
        print("key: ", g.key)
        collection.update_one(
            {"Key": g.key},
            {
                "$addToSet": {
                    "Data": {
                        "$each": [{ "name": names[i], "maxRP": data[i]["maxRP"] } for i in range(len(data))]
                    },
                    "Results": {
                        "$each": [
                            {
                                "name": names[i],
                                "nodes": [[j[0], j[1]] for j in results[i].items()]
                            } for i in range(len(results))
                        ]
                    }
                }
            },
            upsert=True  # Crea el documento si no existe
        )

        print("Documento actualizado")
    
# insert()
# select()


#flag --insert --select

import sys

def print_help():
    print("Uso:")
    print("Comando para generar grafos de [rango1, rango2] nodos y [n, random(n, n(n-1)/2)] aristas:")
    print("python script.py --generate-dags [range1] [range2] [iter] [nro]")
    print("Ejemplo: python script.py --generate-dags 5 50 5 10")
    print("range1: Número mínimo de nodos")
    print("range2: Número máximo de nodos")
    print("iter: Número de iteracion i de aristas e_i = [n, n(n-1)/2] aristas por cada n_j = [rango1, rango2] nodos")
    print("nro: Número de conjuntos de nodos")
    print()
    print("Comando para obtener DAGs de la base de datos y guardarlos en una carpeta:")
    print("python script.py --select-save-dataset [path]")
    print("Ejemplo: python script.py --select-save-dataset ~/Codigo/implementación/src/grafos/")
    print("path: Ruta de la carpeta donde se guardarán los grafos")
    print()
    
if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Falta el comando")
        sys.exit(1)

    if sys.argv[1] == "--generate-dags":
        if len(sys.argv) != 6:
            print("Número incorrecto de argumentos para --generate-dags")
            sys.exit(1)

        r1 = int(sys.argv[2])
        r2 = int(sys.argv[3])
        n_iter = int(sys.argv[4])
        nro_grafos = int(sys.argv[5])
        # Llamar a la función insert con los argumentos
        generate(r1, r2, n_iter, nro_grafos)

    elif sys.argv[1] == "--select-save-dataset":
        if len(sys.argv) == 3:
            select_and_save(sys.argv[-1])
        else:
            select_and_save()
    
    elif sys.argv[1] == "--read-save-mongo":
        if len(sys.argv) == 3:
            grafo, result, names, data = read_json(sys.argv[-1])
        else:
            grafo, result, names, data = read_json("grafo.json")
        save_in_mongo(grafo, result, names, data)
        
    
    # elif sys.argv[1] == "--select":
    #     if len(sys.argv) != 4:
    #         print("Número incorrecto de argumentos para --select")
    #         sys.exit(1)

    #     nro_nodes = int(sys.argv[2])
    #     nro_edges = int(sys.argv[3])
    #     # Llamar a la función select con los argumentos
    #     select(nro_nodes, nro_edges)
    
    
    # elif sys.argv[1] == "--save":
    #     save()
        
        
    elif sys.argv[1] == "--help":
        print_help()
    else:
        print("Comando no reconocido")
        sys.exit(1)
