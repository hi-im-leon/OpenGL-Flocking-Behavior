project(FKViewer)
include_directories(../libsrc/animation ../libsrc/ui ../3rdparty/AntTweakBar/include)
link_directories(/usr/X11R6/lib libsrc/animation libsrc/ui ../lib ../3rdparty/AntTweakBar/lib)
add_executable(FKViewer BVHViewer.cpp BVHMain)
target_link_libraries(FKViewer animationfk animationbasic animationcurve animationscurve ui AntTweakBar GLEW glut GL GLU X11)

add_executable(FKViewer-Soln BVHViewer.cpp BVHMain)
target_link_libraries(FKViewer-Soln animationfk-soln animationbasic-soln animationcurve-soln animationscurve-soln ui AntTweakBar GLEW glut GL GLU X11)

