project(BlendViewer)
include_directories(../libsrc/animation ../libsrc/ui ../3rdparty/AntTweakBar/include)
link_directories(/usr/X11R6/lib ../lib libsrc/animation libsrc/ui ../3rdparty/AntTweakBar/lib)
add_executable(BlendViewer BlendViewer.cpp BlendMain.cpp)
target_link_libraries(BlendViewer animationblend animationfk animationbasic animationcurve animationscurve ui AntTweakBar GLEW glut GL GLU X11)

add_executable(BlendViewer-Soln BlendViewer.cpp BlendMain.cpp)
target_link_libraries(BlendViewer-Soln animationblend-soln animationfk-soln animationbasic-soln animationcurve-soln animationscurve-soln ui AntTweakBar GLEW glut GL GLU X11)

