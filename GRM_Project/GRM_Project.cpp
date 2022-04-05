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
#include "Stereo.h"

#include <stdio.h>


int main(int argc, char* argv[])
{
    if (argc == 3) {


        cv::Mat left = cv::imread(argv[1], cv::IMREAD_GRAYSCALE);
        if (left.empty())
        {
            std::cout << "Image Note Found!!! " << argv[1] << std::endl;
            return -1;
        }
        cv::Mat right = cv::imread(argv[2], cv::IMREAD_GRAYSCALE);
        if (right.empty())
        {
            std::cout << "Image Note Found!!! " << argv[2] << std::endl;
            return -1;
        }
        int rows = left.rows;
        int cols = left.cols;

        Stereo stereo = Stereo(5, 16);
        auto res = stereo.ssd(left, right);

        cv::Mat disp = cv::Mat(cv::Size(cols, rows), CV_8U);
        std::memcpy(disp.data, res.data(), res.size() * sizeof(uint8_t));


        auto GPD_stereo = GridPrimalDual(disp, 16, 1, 5, 16, false);
        //GPD.printPrimalDual();


        GPD_stereo.optimize();

        // make image from x
        cv::Mat optimized = cv::Mat(cv::Size(GPD_stereo.columns, GPD_stereo.rows), CV_8U);
        auto opti_vec = std::vector<uint8_t>(GPD_stereo.rows * GPD_stereo.columns, 0);
        for (int row = 0; row < GPD_stereo.rows; row++)
        {
            for (int column = 0; column < GPD_stereo.columns; column++)
            {
                opti_vec[GPD_stereo.columns * row + column] = GPD_stereo.x[row][column] * 16;
                res[GPD_stereo.columns * row + column] = res[GPD_stereo.columns * row + column] * 16;
            }
        }

        std::memcpy(optimized.data, opti_vec.data(), opti_vec.size() * sizeof(uint8_t));
        std::memcpy(disp.data, res.data(), res.size() * sizeof(uint8_t));



        cv::imwrite("orig.png", disp);
        cv::imwrite("opti.png", optimized);


        cv::waitKey(0);
        cv::destroyAllWindows();





        return 0;
    } else if (argc != 2) {
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

    //cv::resize(image, image, cv::Size(256,256));
    //cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);
    std::cout << image.size().height << " " << image.size().width << " " << image.depth() << std::endl;

    //cv::imshow("window title", image);

    //cv::waitKey(0);

    auto GPD = GridPrimalDual(image, 256, 1, 200, 10000, true);
    //GPD.printPrimalDual();


    GPD.optimize();
    
    // make image from x
    cv::Mat greyImgForVecCopy = cv::Mat(cv::Size(GPD.columns, GPD.rows), CV_8U);
    auto vec = std::vector<uint8_t>(GPD.rows * GPD.columns, 0);
    for (int row = 0; row < GPD.rows; row++)
    {
        for (int column = 0; column < GPD.columns; column++)
        {
            vec[GPD.columns * row + column] = GPD.x[row][column];
        }
    }

    std::memcpy(greyImgForVecCopy.data, vec.data(), vec.size() * sizeof(uint8_t));

    cv::imwrite("smoothed_pep.png", greyImgForVecCopy);

    cv::imwrite("orig_pep.png", image);

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
