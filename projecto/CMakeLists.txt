cmake_minimum_required (VERSION 2.6)
project (Thesis)

# Defines the folder where not standard cmake files are stored
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")

include_directories("/usr/local/include")
link_directories("/usr/local/lib")

find_package(PCL 1.3 REQUIRED COMPONENTS common io)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(libfreenect REQUIRED)
link_directories(${LIBFREENECT_LIBRARY_DIRS})
include_directories(${LIBFREENECT_INCLUDE_DIRS})

find_package(GLUT)
link_directories(${GLUT_LIBRARY_DIRS})

find_package(OpenGL)
link_directories(${OPENGL_LIBRARY_DIRS})

# Add subdirectories
add_subdirectory(KinectDevice)

add_executable(thesis main.cpp)

target_link_libraries(thesis ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES}
	${LIBFREENECT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES})
