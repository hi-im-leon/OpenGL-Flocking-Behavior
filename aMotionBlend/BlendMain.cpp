#include "BlendViewer.h"

int main(int argc, char** argv)
{
    BlendViewer viewer;
	 viewer.init(argc, argv);
    viewer.loadMotion1("../motions/Beta/walking.bvh");
    viewer.loadMotion2("../motions/Beta/jump.bvh");
    viewer.loadDir("../motions/Beta/");
    viewer.blend();

	viewer.run();
	return 0;
}

