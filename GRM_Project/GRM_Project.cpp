// GRM_Project.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <vector>
#include <list>
#include <queue>
#include <limits>
#include <unordered_map>

#include <opencv2/opencv.hpp>

#include "Graph.h"
#include "GridPrimalDual.h"





int main(int argc, char* argv[])
{
    // pixel labelling with pott model energy, see parmi04.pdf, "An Experimental Comparison ofMin - Cut / Max - Flow Algorithms for Energy Minimization in Vision"









    /*auto G = Graph();

    G.addEdge(1, 2, 10);
    G.addEdge(1, 3, 5);
    G.addEdge(1, 4, 15);
    G.addEdge(2, 3, 4);
    G.addEdge(2, 5, 9);
    G.addEdge(2, 6, 15);
    G.addEdge(3, 4, 4);
    G.addEdge(3, 6, 8);
    G.addEdge(4, 7, 30);
    G.addEdge(5, 6, 15);
    G.addEdge(5, 8, 10);
    G.addEdge(6, 7, 15);
    G.addEdge(6, 8, 10);
    G.addEdge(7, 3, 6);
    G.addEdge(7, 8, 10);

    G.printAdjacencyList();

    int max_flow = G.max_flow(1,8);
    std::cout << max_flow << std::endl;


    return 0;*/
    if (argc != 2) {
        std::cout << "usage: .exe Path/To/Image" << std::endl;
        return -1;
    }
    cv::Mat image = cv::imread(argv[1], cv::IMREAD_GRAYSCALE);

    if (image.empty())
    {
        std::cout << "Image Note Found!!!" << std::endl;
        return -1;
    }
    std::cout << image.size().height << " " << image.size().width << " " << image.depth() << std::endl;


    //cv::imshow("window title", image);

    //cv::waitKey(0);

    auto GPD = GridPrimalDual(image, 256);
    GPD.printPrimalDual();


    // Define Markov Random Field Model






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
