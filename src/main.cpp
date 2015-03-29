#include <iostream>
#include <stdio.h>
#include <cstdlib>
#include "ik.h"


int main( int argc, char **argv )
{
    if ( argc < 4 ) {
        std::cout << "usage : cmd x y z\n";
        exit(1);
    }   

    double x = strtod(argv[1], NULL);
    double y = strtod(argv[2], NULL);
    double z = strtod(argv[3], NULL);

    double theta1 = 0, theta2 = 0, theta3 = 0;

    if ( getTheta(x, y, z, theta1, theta2, theta3) == 0 ) {
        printf("[%f, %f, %f] -> [%f, %f, %f]\n", x, y, z, theta1, theta2, theta3);
    }

    return 0;
}
