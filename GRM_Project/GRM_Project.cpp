// GRM_Project.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <vector>
#include <list>
#include <queue>
#include <limits>
#include <unordered_map>

#include "Graph.h"


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
