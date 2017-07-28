project(CurveEditor)
include_directories(../libsrc/animation ../libsrc/ui )
link_directories(/usr/X11R6/lib ../lib ../libsrc/animation ../libsrc/ui)
add_executable(CurveEditor curveMain.cpp CurveViewer.cpp)
target_link_libraries(CurveEditor animationbasic animationcurve ui AntTweakBar GLEW glut GL GLU X11)

add_executable(CurveEditor-Soln curveMain.cpp CurveViewer.cpp)
target_link_libraries(CurveEditor-Soln animationbasic-soln animationcurve-soln ui AntTweakBar GLEW glut GL GLU X11)
