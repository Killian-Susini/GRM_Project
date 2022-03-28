#include <iostream>
#include <vector>
#include <list>
#include <queue>
#include <limits>
#include <unordered_map>

#pragma once
class Graph
{
public:
    Graph();
    void addNode(int vertex);
    void addEdge(int vertex_u, int vertex_v, int weight);
    void printAdjacencyList();
    int max_flow(int vertex_source, int vertex_sink);

    int nbNodes;
    std::vector< std::vector<std::pair<int, int>> > adjacencyList;
    std::vector< int > vertices;
    std::unordered_map<int, int> vertex2index;

private:
    void push(int u, int v, std::vector<int>& excess, const std::vector <std::vector < int >>& C, std::vector <std::vector < int >>& F);
    void relabel(int u, std::vector<int>& height, const std::vector <std::vector < int >>& C, const std::vector <std::vector < int >>& F);
    void discharge(int u, std::vector<int>& excess, std::vector<int>& height, std::vector<int>& seen, const std::vector <std::vector < int >>& C, std::vector <std::vector < int >>& F);
};

