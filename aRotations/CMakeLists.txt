project(Rotations)
include_directories(../libsrc/animation ../libsrc/ui ../3rdparty/AntTweakBar/include)
link_directories(/usr/X11R6/lib libsrc/animation libsrc/ui ../3rdparty/AntTweakBar/lib)
add_executable(Rotations rotationsMain.cpp RotationsViewer.cpp)
target_link_libraries(Rotations animationbasic animationcurve animationscurve ui AntTweakBar GLEW glut GL GLU X11)

