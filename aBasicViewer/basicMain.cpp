#include "aBasicViewer.h"

int main(int argc, char** argv)
{
    ABasicViewer viewer;
    viewer.init(argc, argv);
    viewer.run();
    return 0;
}

