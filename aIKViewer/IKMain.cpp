#include "IKViewer.h"

int main(int argc, char** argv)
{
	IKViewer viewer;
	viewer.init(argc, argv);
    viewer.loadMotion("../motions/Beta/Beta.bvh");
	viewer.run();
	return 0;
}

