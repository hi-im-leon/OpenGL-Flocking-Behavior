#include "RotationsViewer.h"
#include "aRotation.h"

int main(int argc, char** argv)
{
	mat3 m;
    RotationsViewer viewer;
	viewer.init(argc, argv);
	viewer.run();
	return 0;
}

