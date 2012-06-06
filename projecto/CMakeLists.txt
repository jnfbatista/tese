cmake_minimum_required (VERSION 2.6)
project (Thesis)

#set (CMAKE_CXX_FLAGS "-m32")
#set (CMAKE_C_FLAGS "-m32")

# Defines the folder where not standard cmake files are stored
# set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")


find_package(PCL 1.3  REQUIRED )

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(GLUT)
link_directories(${GLUT_LIBRARY_DIRS})

find_package(OpenGL)
link_directories(${OPENGL_LIBRARY_DIRS})

#find_package(OpenCV REQUIRED)
#link_directories(${OpenCV_LIBRARY_DIRS})

find_package(Boost COMPONENTS system filesystem REQUIRED)

find_package(Qt4 REQUIRED)
link_directories(${Qt4_LIBRARY_DIRS})

include(${QT_USE_FILE})

#add_subdirectory(processfunc)

# Add subdirectories
add_subdirectory(PCLWorker)
add_subdirectory(Message)
add_subdirectory(MessageServer)
add_subdirectory(gui)

add_executable(thesis main.cpp)

target_link_libraries(thesis ${PCL_LIBRARIES}
	${OPENGL_LIBRARIES} ${GLUT_LIBRARIES} 
	#${OpenCV_LIBS} 
	PCLWorker 
	Message	MessageServer
	Gui
	${QT_LIBRARIES}
	${Boost_FILESYSTEM_LIBRARY}
	${Boost_SYSTEM_LIBRARY}
	pthread
	)
