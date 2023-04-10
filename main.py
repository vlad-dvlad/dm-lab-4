from matrix import *
import numpy as np

matr = np.matrix(matrix_graph)
print(f"Матриця ваг: \n{matr}\n")

class Graph:

    def __init__(self, graph):
        self.graph = graph
        self.ROW = len(graph)

    def bfs(self, s, t, parent):  # Пошук в ширину

        visited = [False] * (self.ROW)

        queue = []

        queue.append(s)
        visited[s] = True


        while queue:


            u = queue.pop(0)

            for ind, val in enumerate(self.graph[u]):
                if visited[ind] == False and val > 0:

                    queue.append(ind)
                    visited[ind] = True
                    parent[ind] = u
                    if ind == t:
                        return True

        return False

    def ford_fulkerson(self, source, sink):  # Алгоритм Форда-Фалкерсона

        parent = [-1] * (self.ROW)

        max_flow = 0

        while self.bfs(source, sink, parent):

            path_flow = float("Inf")
            s = sink
            while (s != source):
                path_flow = min(path_flow, self.graph[parent[s]][s])
                s = parent[s]

            max_flow += path_flow

            v = sink
            while (v != source):
                u = parent[v]
                self.graph[u][v] -= path_flow
                self.graph[v][u] += path_flow
                v = parent[v]

        return max_flow


graph = Graph(matrix_graph)  # матриця

source = 0
sink = 7

print("Максимальний потік === %d " % graph.ford_fulkerson(source, sink))
