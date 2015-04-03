cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

FIND_PACKAGE(OpenGL)
FIND_PACKAGE(GLUT)
FIND_PACKAGE(GLEW)


IF(GLEW_FOUND)
   SET(tiney_LIBS ${tiney_LIBS} ${GLEW_LIBRARY} )
ENDIF(GLEW_FOUND)

IF(OPENGL_FOUND)
   SET(tiney_LIBS ${tiney_LIBS} ${OPENGL_LIBRARIES} )
ENDIF(OPENGL_FOUND)

IF(GLUT_FOUND)
   SET(tiney_LIBS ${tiney_LIBS} ${GLUT_LIBRARIES} )
ENDIF(GLUT_FOUND)

include_directories (
	${OPENGL_INCLUDE_DIR}
	#${Box2D_SOURCE_DIR}
	../
	/usr/include/ni
	/usr/include/nite
)


#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(pointing_gesture src/pointing_gesture.cpp)
include_directories(/usr/include/ni /usr/include/nite)
rosbuild_add_executable(pointing_gesture pointing_gesture.cpp SceneDrawer.cpp)
#target_link_libraries(example ${PROJECT_NAME})
TARGET_LINK_LIBRARIES(pointing_gesture ${tiney_LIBS} OpenNI)

