#ifndef GRAPH_H
#define GRAPH_H

#include <vector>
#include <map>
#include <iostream>

struct Node {
  long id;
  double latitude;
  double longitude;
};

class Graph {
 private:
  // Lista de adj, o id do no e a lista de nbr
  std::map<long, std::vector<std::pair<long,long>>> adjList;

  std::map<long, Node> nodes;
 public:
  // Função para carregar o mapa
  void loadFromOSM(std::string filename);

  std::vector<long> dijkstra(long sourceNode, long endNode);

  std::vector<long> aStar(long sourceNode, long endNode);

  double getDistance(long nodeA, long nodeB);
};

#endif