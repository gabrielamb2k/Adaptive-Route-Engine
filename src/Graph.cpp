#include "Graph.h"
#include <cmath>
#include <queue>
#include <iostream>
#include <vector>
#include <algorithm>

void Graph::loadFromOSM(std::string filename) { std::cout << "Carregando mapa: " << filename << std::endl; }

std::vector<long> Graph::dijkstra(long sourceNode, long endNode) {
  if (sourceNode == endNode) {
    return {sourceNode};
  }

  std::priority_queue<std::pair<long, long>, std::vector<std::pair<long, long>>, std::greater<std::pair<long, long>>>
      pq;

  std::map<long, long> dist;
  std::map<long, long> parent;
  pq.push({0, sourceNode});
  parent[sourceNode] = -1;
  std::vector<long> path;
  while (!pq.empty()) {
    auto [distNow, node] = pq.top();
    if (node == endNode) break;
    pq.pop();
    if (dist.count(node) && dist[node] < distNow) continue;

    for (auto const& vizinho : Graph::adjList[node]) {
      auto [nbr, edgeWeigh] = vizinho;
      if (dist.count(node) || dist[node] + edgeWeigh < dist[nbr]) {
        dist[nbr] = dist[node] + edgeWeigh;
        parent[nbr] = node;
        pq.push({dist[nbr], nbr});
      }
    }
  }

  if (!dist.count(endNode)) {
    std::cout << "Caminho não encontrado" << std::endl;
    return {};
  }

  long atual = endNode;
  while (atual != -1) {
    path.push_back(atual);
    atual = parent[atual];
  }

  std::reverse(path.begin(), path.end());

  return path;
}

std::vector<long> Graph::aStar(long sourceNode, long endNode) {
  if (sourceNode == endNode) {
    return {sourceNode};
  }

  std::map<long, double> potencial;
  std::vector<long> path;
  std::priority_queue<std::pair<double, long>, std::vector<std::pair<double, long>>, std::greater<std::pair<double, long>>>
      pq;

  std::map<long, long> dist;
  std::map<long,double> distHeuristic;
  std::map<long, long> parent;
  double startH = getDistance(sourceNode, endNode);
  dist[sourceNode]=0;
  pq.push({startH, sourceNode});
  distHeuristic[sourceNode]=startH;
  parent[sourceNode] = -1;
  while (!pq.empty()) {
    auto [distNow, node] = pq.top();
    if (node == endNode) break;
    pq.pop();
    if (dist.count(node) && distNow > distHeuristic[node] + 1e-9) continue;
    
    if (potencial.find(node) == potencial.end()) {
        potencial[node] = getDistance(node, endNode);
    }

    for (auto const& vizinho : Graph::adjList[node]) {
      auto [nbr, edgeWeigh] = vizinho;
      potencial[nbr] = getDistance(nbr, endNode);
      double newDist = dist[node]+edgeWeigh + potencial[nbr];
      if (!dist.count(nbr) || newDist < distHeuristic[nbr]) {
        dist[nbr] = dist[node]+edgeWeigh;
        distHeuristic[nbr]=newDist;
        parent[nbr] = node;
        pq.push({distHeuristic[nbr], nbr});
      }
    }
  }

  if (dist[endNode] == 0) {
    std::cout << "Caminho não encontrado" << std::endl;
    return {};
  }

  long atual = endNode;
  while (atual != -1) {
    path.push_back(atual);
    atual = parent[atual];
  }

  std::reverse(path.begin(), path.end());

  return path;
}

inline double toRadians(double degrees) { return degrees * (M_PI / 180.0); }

double Graph::getDistance(long nodeA, long nodeB) {
  Node node1 = Graph::nodes[nodeA];
  Node node2 = Graph::nodes[nodeB];

  const double R = 6371.0;

  double phi1 = toRadians(node1.latitude);
  double phi2 = toRadians(node2.latitude);
  double dphi = toRadians(node2.latitude - node1.latitude);
  double dlambda = toRadians(node2.longitude - node1.longitude);

  double sin_dphi = std::sin(dphi / 2.0);
  double sin_dlambda = std::sin(dlambda / 2.0);

  double a = (sin_dphi * sin_dphi) + std::cos(phi1) * std::cos(phi2) * (sin_dlambda * sin_dlambda);

  a = std::min(1.0, std::max(0.0, a));

  double c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));

  return R * c;
}