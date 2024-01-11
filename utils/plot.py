# Test
import json
import sys
import os
import matplotlib.pyplot as plt
import networkx as nx
import numpy as np
import pymongo
import networkx as nx
import random

from script import *

# # Definir la clase Nodo
# class Nodo:
#     def __init__(self, id, latency, dest_size):
#         self.id = id
#         self.latency = latency
#         self.dest_size = dest_size

# # Definir la clase Arista
# class Arista:
#     def __init__(self, i, j):
#         self.i = i
#         self.j = j

# # Definir la clase Grafo
# class Grafo:
#     def __init__(self, nodes, edges):
#         self.nodes = nodes
#         self.edges = edges

#         # Crear una matriz de adyacencia
#         num_nodes = len(nodes)
#         self.matriz_adyacencia = [[0] * num_nodes for _ in range(num_nodes)]
       
#         # Llenar la matriz de adyacencia
#         for edge in edges:
#             self.matriz_adyacencia[edge.i][edge.j] = 1
            


def build_graph_from_json(data):
    nodes_dict = [0] * len(data['Nodes'])
    # { "enumerateRP": { "nodes": [ (node_id, cycle) ] } }
    
    # Creando objetos Node a partir de la lista 'Nodes'
    for node_data in data['Nodes']:
        node = Nodo(node_data['id'], node_data['latency'], node_data['dest_size'])
        nodes_dict[node.id] = node
        
    
    # Agregando aristas a los nodos
    edges = []
    for edge_data in data['Edges']:
        i = edge_data['i']
        j = edge_data['j']
        nodes_dict[i].successors.append(nodes_dict[j])
        nodes_dict[j].predecessors.append(nodes_dict[i])
        edges.append(Arista(i, j))
    
    results = []
    if 'Results' not in data:
        return nodes_dict, results
    
    names = []
    for p in data['Results']:
        array = {}
        nodes = p['nodes']
        names.append(p['name'])
        # print(p['name'])
        for node_data in nodes:
            node_id, cycle = node_data
            array.update({node_id: cycle})
        results.append(array)
    
    g = Grafo(nodes_dict, edges)
    return g, results, names



def calculate_pressure(grafo, timesteps, ciclos):
    nodes = grafo.nodes
    edges = grafo.edges
    pressure = [0] * (ciclos+1)
    # print(pressure)
    # print(timesteps)
    for i, node in enumerate(nodes):
        time_step = timesteps[(node.id)]  # TODO: ver si hay que restar 1 ya que el array pressure empieza en 0
        last_use = time_step
        for edge in edges:
            if edge.i == node.id:
                if timesteps[edge.j] > last_use:
                    last_use = timesteps[edge.j]
        
        succesors = sum([1 if edge.i == node.id else 0 for edge in edges])
        if succesors == 0:
            pressure[time_step]+= node.dest_size
        else:
            for j in range(time_step, last_use):
                pressure[j] += node.dest_size
    return pressure

def calculate_pressure_with_latency(grafo, timesteps):
    nodes = grafo.nodes
    
    order = [ k for k, v in sorted(timesteps.items(), key=lambda item: item[1])]
    print(order)
    pred_left = { node.id: len(node.predecessors) for node in nodes}
    
    ready = [ node.id for node in nodes if len(node.predecessors) == 0]
    active = []
    # { node_id: usos }
    lives = { node.id: len(node.successors) for node in nodes}
    is_live = { node.id: False for node in nodes}
    
    pressures = []
    cycle = 1
    while len(ready) > 0 or len(active) > 0:
        if len(order) > 0:
            node_id = order[0] 
            node = nodes[node_id]
        
            if node_id in ready:
                order.pop(0)
                ready.remove(node_id)
                active.append({node_id: cycle + node.latency - 1}) #TODO: ver si hay que restar 1 ya que el array pressure empieza en 0
                
                is_live[node_id] = True
                for edge in node.predecessors:
                    lives[edge.id] -= 1
                    if lives[edge.id] == 0:
                        is_live[edge.id] = False
                
                print(f"nodo {node_id} listo en ciclo {cycle}")
        
        active.sort(key=lambda x: list(x.values())[0]) #ordenar por el tiempo de finalizacion, el primero es el que termina primero
        
        print(f"activos: {active}")
        ids = [ k for k, v in lives.items() if is_live[k] ]
        print(f"ids: {ids}")
        pressure = sum([nodes[node_id].dest_size for node_id in  ids])
        pressures.append(pressure)
        
        while len(active) > 0 and list(active[0].values())[0] <= cycle:
            print(f"nodo {list(active[0].keys())[0]} termino en ciclo {cycle}")
            act = active.pop(0)
            act_id =  list(act.keys())[0]
            node_act = nodes[act_id]
            for node in node_act.successors:
                pred_left[node.id] -= 1
                if pred_left[node.id] == 0:
                    ready.append(node.id)
        cycle += 1
    return pressures

#TODO: terminar
#dado un nodo y un timestep, retorna un array con los nodos y el timestep pero considerando el latency
def update_timestep(nodes, timesteps):
    #obtener roots
    # times = {}
    # for node in nodes:
    #     if len(node.predecessors) == 0:
    #         update_timesteps(node, timesteps[node.id], times)
    #recorrer nodos en orden original
    n = [ [node, timesteps[node.id]] for node in nodes]
    n.sort(key=lambda x: x[1])
    funct_units = 4
    min_ciclo = { node_id: time for node_id, time in timesteps.items()}
    
    ciclo = 1
    node_index = 0
    while node_index < len(n):
        if n[node_index][1] == ciclo:
            ciclo = n[node_index][1]

def update_timesteps(node, time, timesteps={}):
    if (node.id not in timesteps) or (node.id in timesteps and timesteps[node.id] < time):
        timesteps.update({node.id: time})
    
    for edge in node.successors:
        update_timesteps(edge.node, time + node.latency, timesteps)



# Función para crear el grafo a partir de los datos
def create_graph(graph_data):
    G = nx.DiGraph()
    for node_data in graph_data['Nodes']:
        G.add_node(node_data['id'])
    for edge_data in graph_data['Edges']:
        G.add_edge(edge_data['i'], edge_data['j'])
    return G

# Función para mostrar el grafo
def draw_graph(G):
    pos = nx.spring_layout(G)
    nx.draw(G, pos, with_labels=True, node_size=500, node_color='lightblue', font_size=10, font_color='black')
    plt.title("Grafo del problema")
    plt.savefig("graph.pdf", format="pdf")
    plt.show()


# Función para crear y mostrar graficos de presión en un mismo grafico. poner cuaddricula de uno en uno
def plot_pressure(pressures, names, allinone=True, recursive=False):
    if not allinone:
        for i, pressure in enumerate(pressures):
            plot_pressure([pressure], [names[i]], True, True)
        plt.title("Planificaciones RP vs Ciclos")
        plt.legend()
        plt.savefig(f"{title}.pdf", format="pdf")
        plt.show()
        return
    title = names[0] if len(names) == 1 else "Planificaciones RP vs Ciclos"
    plt.figure(figsize=(12, 6))
    # plt.subplot(1, 2, 1)
    plt.xlim(0, max(max_cycle, max([len(p) for p in pressures])) + 1)
    plt.ylim(0, max([max(p) for p in pressures]) + 1)
    #cuadricula
    # plt.xticks(range(max_cycle + 1))
    # plt.yticks(range(max([max(p) for p in pressures]) + 1))
    
    #mostrar numeros en los ejes de 5 en 5
    plt.xticks(np.arange(0, max_cycle + 1, 1))
    plt.yticks(np.arange(0, max([max(p) for p in pressures]) + 1, 5))
    #cuadrados
    plt.grid(axis='y', which='major', linestyle='--', linewidth='0.5', color='gray')
    colors = ['g', 'b', 'r', 'c', 'm', 'y', 'k']
    
    bar_width = 0.1
    spacing = 0.01
    for i, pressure in enumerate(pressures):
        secuencia = [r[1] for r in results[i].items()]
        secuencia.sort()
        
        # Calcular la posición de las barras para este conjunto de datos
        x_positions = np.arange(len(pressure)) + i * (bar_width + spacing)
        
        # Crear el gráfico de barras
        # plt.bar(x_positions, pressure, width=bar_width, label=f"{names[i]}: {max(pressure)}", color=colors[i])
        # con lineas
        plt.plot(x_positions, pressure, label=f"{names[i]}: {max(pressure)}", color=colors[i])
        
        # Etiquetas para cada punto
        # for j, x in enumerate(x_positions):
        #     plt.text(x, pressure[j], pressure[j], ha='center', va='bottom')
        
        # Líneas desde el último punto hasta y=0
        # plt.plot([x_positions[-1] + bar_width / 2, x_positions[-1] + bar_width / 2], [0, max(pressure)], linestyle='--', color=colors[i])

        
        #linea desde el ultimo punto hasta y=0
        # plt.plot([secuencia[-1]-1, secuencia[-1]-1], [0, max(pressure)], linestyle='--', color=colors[i])
        plt.plot([0, len(pressure)+1], [max(pressure), max(pressure)], linestyle='--', color=colors[i])
    
    if not recursive:
        plt.title(title)
        plt.legend()
        plt.savefig(f"{title}.pdf", format="pdf")
        plt.show()
    
    
#recibir como argumento el nombre del archivo json con el path relativo
path = sys.argv[1]
# path = "grafo.json"
abs_path = os.path.abspath(path)

# Leer graph.json
with open(abs_path) as json_file:
    data = json_file.read()
graph_data = json.loads(data)
graph, results, names = build_graph_from_json(graph_data)

pressures = []
max_cycle = 0
print(max_cycle)
for r in results:
    secuencia = [ r[1] for r in r.items()]
    secuencia.sort()
    if secuencia[-1] > max_cycle:
        max_cycle = secuencia[-1]

for r in results:
#Calcular presion
    mx_cycle = max(r.values())
    pressure = calculate_pressure(graph, r, mx_cycle)
    # pressure = calculate_pressure_with_latency(graph, r)
    # print("q-Presion")
    print(pressure)
    pressures.append(pressure)
   
# Plotear presiones de todos los resultados



    

# # Crear el grafo
G = create_graph(graph_data)

# # Dibujar el grafo
draw_graph(G)

import math

def circular_grpah(nodes, edges, results, name, r=2):
    sec = results['nodes'] # [ (node_id, cycle) ]

    sorted_sec = sorted(sec, key=lambda x: x[1])

    # crear n-agono de la secuencia de nodos

    n = len(sec)
    
    center = (0, 0)
    points = []
    for i in range(n):
        x = center[0] + r * math.cos(2 * math.pi * i / n)
        y = center[1] + r * math.sin(2 * math.pi * i / n)
        points.append((x, y))

    # mapear nodos a circulo
    pos = {}
    for i, node in enumerate(sorted_sec):
        pos[node[0]] = points[i]

    # conectar con flechas los puntos de la secuencia
    flechas = [] # [ (source, target) ] 
    for i in range(n):
        #ver edges que salen de sorted_sec[i][0]
        for edge in edges:
            if edge['i'] == sorted_sec[i][0]:
                flechas.append((points[i], pos[edge['j']]))

    # dibujar puntos y flechas
    plt.xlim(-3, 3)
    plt.ylim(-3, 3)
    plt.title(name)
    plt.plot([x for x, y in points], [y for x, y in points], marker='o', linestyle='-', color='b')
    for i in range(n):
        plt.text(points[i][0], points[i][1], sorted_sec[i][0])
    for source, target in flechas:
        plt.arrow(source[0], source[1], target[0] - source[0], target[1] - source[1], length_includes_head=True, head_width=0.1)
    plt.show()


# print("Presiones")
plot_pressure(pressures, names, True)
# circular_grpah(graph_data['Nodes'], graph_data['Edges'], graph_data['Results'][0], "RP vs Ciclos")