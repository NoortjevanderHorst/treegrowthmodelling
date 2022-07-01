//
// Created by noort on 23/01/2022.
//

#include <iostream>

#include "growth_viewer.h"

// todo: put every definition used by multiple classes here if possible


int main(int argc, const char *argv[]) {
    std::cout << "--- running GTree ---" << std::endl;

    if (argc == 1) {
        GrowthViewer viewer;
        viewer.run();
        return EXIT_SUCCESS;
    }

    std::cerr << "An error occurred while running GTree." << std::endl;

    return EXIT_FAILURE;
}