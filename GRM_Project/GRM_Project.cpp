// GRM_Project.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <vector>
#include <list>
#include <queue>
#include <limits>
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
	void push(int u, int v, std::vector<int>& excess,const std::vector <std::vector < int >>& C, std::vector <std::vector < int >>& F) {
		int max_possible = std::min(excess[u], C[u][v]-F[u][v]);
		excess[u] -= max_possible;
		excess[v] += max_possible;
		F[u][v] += max_possible;
		F[v][u] -= max_possible;
	}
	void relabel(int u, std::vector<int>& height, const std::vector <std::vector < int >>& C, const std::vector <std::vector < int >>& F) {
		// we will set the height of u to be at least the one more the height of the min vertex to which we can push some flow
		int min_height = INT_MAX;
		auto F_u = F[u];
		auto C_u = C[u];
		for (int v = 0; v < F_u.size(); v++) {
			if (C_u[v] - F_u[v] > 0) { // there is an edge in the residual graph from u to v
				min_height = std::min(min_height, height[v]);
				height[u] = min_height + 1;
			}
		}
	}
	void discharge(int u, std::vector<int>& excess, std::vector<int>& height, std::vector<int>& seen, const std::vector <std::vector < int >>& C, std::vector <std::vector < int >>& F) {
		
		size_t n = excess.size();
		while (excess[u] > 0) { // while there is still excess left
			if (seen[u] < n) { // if we haven't seen all the nodes
				int v = seen[u];
				if ((C[u][v] - F[u][v]) && (height[u] > height[v])) { //if we can push
					push(u, v, excess, C, F);
				}
				else { //move on
					seen[u]++;
				}
			}
			else { 
				// still have excess but nowhere to push it, need a relabel, then reset cursor
				relabel(u, height, C, F);
				seen[u] = 0; 
			}
		}
	}
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
			std::cout << '(' << vertices[v_weight.first] << ',' << v_weight.second << ") ";
		}
		std::cout << std::endl;
	}
}

int Graph::max_flow(int vertex_source, int vertex_sink)
{
	auto itt_s = vertex2index.find(vertex_source);
	auto itt_t = vertex2index.find(vertex_sink);

	int s;
	int t;

	if (itt_s == vertex2index.end()) 
		throw "Source vertex does not exist";
	s = itt_s->second;

	if (itt_t == vertex2index.end()) 
		throw "Sink vertex does not exist";
	
	t = itt_t->second;

	if (s == t)
		throw "Source and Sink are the same";


	size_t n = vertices.size();


	// for now let's do adjacency matrix style
	//Capacity and Flow (Residual == C-F) 
	std::vector <std::vector < int >> C, F;

	for (auto edgeList : adjacencyList) 
	{
		F.push_back(std::vector< int >(n, 0));
		auto capacity = std::vector< int >(n, 0);
		for (auto edge : edgeList) {
			capacity[edge.first] = edge.second;
		}
		C.push_back(capacity);
	}

	for (int i = 0; i < n; i++)
	{
		for (int j = 0; j < n; j++) {
			std::cout << C[i][j] << " ";
		}
		std::cout << std::endl;
	}



	auto excess = std::vector<int>(n, 0);
	auto height = std::vector<int>(n, 0);
	auto seen   = std::vector<int>(n, 0);



	std::list<int> active_list;

	for (int i = 0; i < n; i++)
		if (i != s && i != t)
			active_list.push_back(i);

	
	// let the source have infinite excess and be of highest height
	excess[s] = INT_MAX;
	height[s] = n;
	
	// push all you can to neighboor nodes
	for (int v = 0; v < n; v++)
		push(s, v, excess, C, F);

	auto u = active_list.begin();
	while (u != active_list.end()) {
		int old_height = height[*u];
		discharge(*u, excess, height, seen, C, F);
		if (height[*u] > old_height) {
			//relabel to front style 
			active_list.splice(active_list.begin(), active_list, u);
		}
		else {
			u++;
		}
	}
	int flow = 0;
	for (int u = 0; u < n; u++) {
		flow += F[u][t];
	}
	return flow;
}


int main()
{
	auto G = Graph();

	G.addEdge(0, 1, 15);
	G.addEdge(0, 2, 4);
	G.addEdge(1, 3, 12);
	G.addEdge(2, 4, 10);
	G.addEdge(3, 2, 3);
	G.addEdge(4, 1, 5);
	G.addEdge(3, 5, 8);
	G.addEdge(4, 5, 10);

	G.printAdjacencyList();

	int max_flow = G.max_flow(0,5);
	std::cout << max_flow << std::endl;
	

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
