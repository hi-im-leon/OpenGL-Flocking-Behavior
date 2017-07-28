project(BasicViewer)
include_directories(../libsrc/animation ../libsrc/ui)
link_directories(/usr/X11R6/lib /usr/local/lib libsrc/animation libsrc/ui)
add_executable(BasicViewer basicMain.cpp)
target_link_libraries(BasicViewer animationbasic ui AntTweakBar GLEW glut GL GLU X11)
install(TARGETS BasicViewer DESTINATION ${PROJECT_SOURCE_DIR}/bin)

