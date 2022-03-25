// GRM_Project.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <vector>
#include <queue>
#include <unordered_map>

class Graph
{
public:
	Graph();
	void addNode(int vertex);
	void addEdge(int vertex_u, int vertex_v, int weight);
	void printAdjacencyList();
	int max_flow(int vertex_source, int vertex_sink);

	int nbNodes;
	std::vector< std::vector<std::pair<int,int>> > adjacencyList;
	std::vector< int > vertices;
	std::unordered_map<int, int> vertex2index;

private:

};

Graph::Graph()
{
	nbNodes = 0;
}


void Graph::addNode(int vertex)
{
	if (vertex2index.count(vertex) > 0)// if already exist, do nothing
		return;
	vertex2index[vertex] = nbNodes;
	vertices.push_back(vertex);
	std::vector<std::pair<int, int>> row;
	adjacencyList.push_back(row);

	nbNodes++;
}

void Graph::addEdge(int vertex_u, int vertex_v, int weight)
{
	if (vertex_u == vertex_v) {
		throw "Self loops are not allowed";
	}

	auto itt_u = vertex2index.find(vertex_u);
	auto itt_v = vertex2index.find(vertex_v);

	int u;
	int v;

	if (itt_u == vertex2index.end()) {
		addNode(vertex_u);
		u = vertex2index[vertex_u];
	}
	else {
		u = itt_u->second;
	}

	if (itt_v == vertex2index.end()) {
		addNode(vertex_v);
		v = vertex2index[vertex_v];
	}
	else {
		v = itt_v->second;
	}

	adjacencyList[u].push_back(std::pair<int, int>(v, weight)); 
	//adjacencyList[v].push_back(std::pair<int, int>(u, weight));
}

void Graph::printAdjacencyList() 
{
	for (const auto vertex_u: vertices) 
	{
		std::cout << vertex_u << " -> ";
		auto u = vertex2index[vertex_u];
		auto edge_list = adjacencyList[u];
		for (const auto v_weight : edge_list) 
		{
			std::cout << '(' << v_weight.first << ',' << v_weight.second << ") ";
		}
		std::cout << std::endl;
	}
}

int Graph::max_flow(int vertex_source, int vertex_sink)
{


	return 0;
}


int main()
{
	auto G = Graph();

	G.addEdge(0, 1, 1);
	G.addEdge(1, 2, 5);
	G.addEdge(2, 3, 4);
	G.addEdge(3, 4, 123);
	G.addEdge(3, 5, 113);
	G.addEdge(3, 1, 13);

	G.printAdjacencyList();

    return 0;
}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
