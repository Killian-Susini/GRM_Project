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


#include <stdio.h>


int main(int argc, char* argv[])
{
    // pixel labelling with pott model energy, see parmi04.pdf, "An Experimental Comparison ofMin - Cut / Max - Flow Algorithms for Energy Minimization in Vision"

    /*
    auto* G = new Graph(4, 6);
    G->add_terminal_cap(0, true, 10);
    G->add_terminal_cap(1, false, 1);
    G->add_terminal_cap(2, false, 1);
    G->add_terminal_cap(3, false, 5);
    G->add_arc_pair(0, 1, 10, 0);
    G->add_arc_pair(2, 1, 0, 5);
    G->add_arc_pair(2, 3, 10, 0);


    int flow = G->max_flow();
    std::cout << "Flow = " << flow << std::endl;


    delete G;
    return 0;
    */
    /// try only 4 levels

    

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

    cv::resize(image, image, cv::Size(10, 10));
    std::cout << image.size().height << " " << image.size().width << " " << image.depth() << std::endl;

    //cv::imshow("window title", image);

    //cv::waitKey(0);

    auto GPD = GridPrimalDual(image, 256, 200);
    //GPD.printPrimalDual();

    GPD.optimize();
    
    // make image from x
    cv::Mat greyImgForVecCopy = cv::Mat(cv::Size(GPD.rows, GPD.columns), CV_8U);
    auto vec = std::vector<uint8_t>(GPD.rows * GPD.columns, 0);
    for (int row = 0; row < GPD.rows; row++)
    {
        for (int column = 0; column < GPD.columns; column++)
        {
            vec[GPD.columns * row + column] = GPD.x[row][column];
        }
    }

    std::memcpy(greyImgForVecCopy.data, vec.data(), vec.size() * sizeof(uint8_t));

    cv::imshow("Grey Image Vec Copy", greyImgForVecCopy);

    cv::imshow("window title", image);

    cv::waitKey(0);
    cv::destroyAllWindows();
    

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
