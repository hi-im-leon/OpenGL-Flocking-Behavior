set(SRC 
   aBasicViewer.cpp  
   aCamera.cpp	  
   aTimer.cpp
   aOsUtils.cpp
   )

add_library(ui SHARED ${SRC})

if (WIN32)
    target_link_libraries(HelloGL glew32 glut32)
    link_directories(3rdparty/GL)
    install (FILES 
      3rdparty/GL/glew32.dll 
      3rdparty/GL/glut32.dll 
      DESTINATION ${CMAKE_BINARY_DIR}/bin)
elseif (UNIX)
    include_directories(../animation)
    install (TARGETS ui DESTINATION ${CMAKE_SOURCE_DIR}/bin)
endif()

