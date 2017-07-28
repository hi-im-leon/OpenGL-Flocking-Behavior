project(IKViewer)
include_directories(../libsrc/animation ../libsrc/ui ../3rdparty/AntTweakBar/include)
link_directories(/usr/X11R6/lib libsrc/animation libsrc/ui ../lib ../3rdparty/AntTweakBar/lib)
add_executable(IKViewer IKViewer.cpp IKMain.cpp)
target_link_libraries(IKViewer animationik animationfk animationbasic animationcurve animationscurve ui AntTweakBar GLEW glut GL GLU X11)

add_executable(IKViewer-Soln IKViewer.cpp IKMain.cpp)
target_link_libraries(IKViewer-Soln animationik-soln animationfk-soln animationbasic-soln animationcurve-soln animationscurve-soln ui AntTweakBar GLEW glut GL GLU X11)

add_executable(IKSimple IKSimple.cpp IKSimpleMain.cpp)
target_link_libraries(IKSimple animationik-soln animationfk-soln animationbasic-soln animationcurve-soln animationscurve-soln ui AntTweakBar GLEW glut GL GLU X11)