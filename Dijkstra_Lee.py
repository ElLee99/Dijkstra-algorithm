# -*- coding: utf-8 -*-
"""
Created on Tue May 31 10:00:53 2022

@author: Johan Lee
"""


import random
import sys
import networkx as nx
import matplotlib.pyplot as plt

class Graph(object):
    
    def __init__(self, nodes, init_graph):
        self.nodes = nodes
        self.graph = self.construct_graph(nodes, init_graph)
        
    def construct_graph(self, nodes, init_graph): #Este metoso comprueba que el grafo sea simetrico
        graph = {}
        for node in nodes:
            graph[node] = {}
        graph.update(init_graph)
        for node, edges in graph.items():
            for adjacent_node, value in edges.items():
                if graph[adjacent_node].get(node, False) == False:
                    graph[adjacent_node][node] = value
                    
        return graph
    
    
    def Nodes(self): #Devuelve los nodos del grafo
        return self.nodes
    
    
    def NodeNeighbors(self, node): #Devuelve los vecinos de un nodo
        connections = []
        for out_node in self.nodes:
            if self.graph[node].get(out_node, False) != False:
                connections.append(out_node)
        return connections
    
    
    def ValueBetween(self, node1, node2): #Devuelve el valor de una arista entre dos nodos
        return self.graph[node1][node2]



def DijkstraAlgorithm(Graph, StartNode):
    UnvisitedNodes = list(Graph.Nodes())
    ShortestPath = {} #Costo actualizado de visitar cada nodo
    PreviousNode = {} #Camino mas corto conocido a un nodo no encuentrado aun

    MaxValue = sys.maxsize    
    for Node in UnvisitedNodes:
        ShortestPath[Node] = MaxValue #Inicializa el valor infinito para los nodods no visitados  
    ShortestPath[StartNode] = 0 #Empieza el valor del nodo con 0
    
    while UnvisitedNodes: #Visita el nodo con menor valor
        CurrentMinNode = None
        for Node in UnvisitedNodes:
            if CurrentMinNode == None:
                CurrentMinNode = Node
            elif ShortestPath[Node] < ShortestPath[CurrentMinNode]:
                CurrentMinNode = Node
                
        Neighbors = Graph.NodeNeighbors(CurrentMinNode) #Actualiza el valor de los nodos vecinos
        for Neighbor in Neighbors:
            TentativeValue = ShortestPath[CurrentMinNode] + Graph.ValueBetween(CurrentMinNode, Neighbor)
            if TentativeValue < ShortestPath[Neighbor]:
                ShortestPath[Neighbor] = TentativeValue
                PreviousNode[Neighbor] = CurrentMinNode #Actualiza el mejor camino al nodo actual

        UnvisitedNodes.remove(CurrentMinNode) #Remueve la bandera de los nodos visitados    
    return PreviousNode, ShortestPath



def print_result(PreviousNodes, ShortestPath, StartNode, TargetNode):
    path = []
    node = TargetNode
    
    while node != StartNode:
        path.append(node)
        node = PreviousNodes[node]
    path.append(StartNode)
    
    print("El trayecto más barato entre:\n\n\t", StartNode, "(", StartNode, ").", 
    "\n\t-\n\t", TargetNode, "(", TargetNode, ").\n")
    print("Ruta: "," > ".join(reversed(path)), "\n       [{} dolares].".format(ShortestPath[TargetNode]))
    return path

nodes = ["GDL", "CDMX", "PHOENIX", "MIAMI", "DALLAS", "SAN FRANCISCO", 
         "LA", "SEATTLE","CHICAGO","TOKYO"]




init_graph = {}
for node in nodes:
    init_graph[node] = {}
    

#Da los valores de las aristas de manera random
precio = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
for x in range(16):
    precio[x] = random.randint(0,1000)
  
    
init_graph["GDL"]["CDMX"] = precio[0]
init_graph["GDL"]["MIAMI"] = precio[1]
init_graph["GDL"]["PHOENIX"] = precio[2]
init_graph["CDMX"]["DALLAS"] = precio[3]
init_graph["CDMX"]["MIAMI"] = precio[4]
init_graph["PHOENIX"]["SAN FRANCISCO"] = precio[5]
init_graph["PHOENIX"]["LA"] = precio[6]
init_graph["MIAMI"]["SAN FRANCISCO"] = precio[7]
init_graph["DALLAS"]["SEATTLE"] = precio[8]
init_graph["DALLAS"]["CHICAGO"] = precio[9]
init_graph["DALLAS"]["TOKYO"] = precio[10]
init_graph["DALLAS"]["SAN FRANCISCO"] = precio[11]
init_graph["SAN FRANCISCO"]["TOKYO"] = precio[12]
init_graph["LA"]["TOKYO"] = precio[13]
init_graph["SEATTLE"]["TOKYO"] = precio[14]
init_graph["CHICAGO"]["TOKYO"] = precio[15]

    

Graph = Graph(nodes, init_graph)
PreviousNodes, ShortestPath = DijkstraAlgorithm(Graph = Graph, StartNode = "GDL") 
#En la siguiente linea se escriben los nodos de la ubicación en donde estamos a donde queremos ir
Path = print_result(PreviousNodes, ShortestPath, "GDL", "TOKYO")
#print(Path)


G = nx.Graph()

#Arreglo de ifs para  agregar los nodos y cambiar el color del nodo en caso de que el camino más corto sea por ahí 

if "GDL" in Path:
    G.add_node("GDL",pos=(0,5), gen=1)
else:
    G.add_node("GDL",pos=(0,5), gen=0)

if "CDMX" in Path:
    G.add_node("CDMX",pos=(6,10),gen=1)
else:
    G.add_node("CDMX",pos=(6,10),gen=0)

if "PHOENIX" in Path:
    G.add_node("PHOENIX",pos=(6,0),gen=1)
else:
    G.add_node("PHOENIX",pos=(6,0),gen=0)

if "DALLAS" in Path:
    G.add_node("DALLAS",pos=(12,10),gen=1)
else:
    G.add_node("DALLAS",pos=(12,10),gen=0)
    
if "MIAMI" in Path:
    G.add_node("MIAMI",pos=(6,5),gen=1)
else:
    G.add_node("MIAMI",pos=(6,5),gen=0)

if "LA" in Path:
    G.add_node("LA",pos=(12,0),gen=1)
else:
    G.add_node("LA",pos=(12,0),gen=0)
      
if "SEATTLE" in Path:
    G.add_node("SEATTLE",pos=(18,10),gen=1)
else:
    G.add_node("SEATTLE",pos=(18,10),gen=0)

if "CHICAGO" in Path:
    G.add_node("CHICAGO",pos=(20,15),gen=1)
else:
    G.add_node("CHICAGO",pos=(20,15),gen=0) 
    
if "SAN FRANCISCO" in Path:
    G.add_node("SF",pos=(12,5),gen=1)
else:
    G.add_node("SF",pos=(12,5),gen=0)
    
if "TOKYO" in Path:
    G.add_node("TOKYO",pos=(20,5),gen=1)
else:
    G.add_node("TOKYO",pos=(20,5),gen=0)



#Agregamos las aritas entre los nodos
G.add_edge("GDL", "CDMX", energy = precio[0])
G.add_edge("GDL", "MIAMI", energy = precio[1])
G.add_edge("GDL", "PHOENIX", energy = precio[2])
G.add_edge("CDMX", "DALLAS", energy = precio[3])
G.add_edge("CDMX", "MIAMI", energy = precio[4])
G.add_edge("PHOENIX", "SF", energy = precio[5])
G.add_edge("PHOENIX", "LA", energy = precio[6])
G.add_edge("MIAMI", "SF", energy = precio[7])
G.add_edge("DALLAS", "SEATTLE", energy = precio[8])
G.add_edge("DALLAS", "CHICAGO", energy = precio[9])
G.add_edge("DALLAS", "TOKYO", energy = precio[10])
G.add_edge("DALLAS", "SF", energy = precio[11])
G.add_edge("SF", "TOKYO", energy = precio[12])
G.add_edge("LA", "TOKYO", energy = precio[13])
G.add_edge("SEATTLE", "TOKYO", energy = precio[14])
G.add_edge("CHICAGO", "TOKYO", energy = precio[15])


color_map=nx.get_node_attributes(G,"gen")

for key in color_map:
    if color_map[key] == 1:
        color_map[key] = "red"
    if color_map[key] == 0:
        color_map[key] = "gray"

#Tomamos todos los elementos necesarios para determinar posición, color y etiquetas
path_colors = [color_map.get(node) for node in G.nodes()]
pos = nx.get_node_attributes(G,'pos')
labels = nx.get_edge_attributes(G,'energy')

#Agregamos todos los elementos necesaarios para graficar y luego se grafica
nx.draw_networkx_edge_labels(G,pos,edge_labels=labels)
nx.draw(G, pos, with_labels=True, node_color=path_colors)
plt.show()

