CMAKE_MINIMUM_REQUIRED(VERSION 3.1 FATAL_ERROR)

ADD_EXECUTABLE(demo1 ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp)
TARGET_LINK_LIBRARIES(demo1 pathfinder)
FILE(COPY ${CMAKE_CURRENT_SOURCE_DIR}/map.txt DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

MESSAGE(WARNING "Please copy map.txt to the executable directory")
