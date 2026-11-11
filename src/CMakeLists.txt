set(SOURCES socket_client.cpp robot_client.cpp simulator_client.cpp)

add_library(${LIBRARY_NAME} OBJECT ${SOURCES})
target_include_directories(${LIBRARY_NAME} PUBLIC ${PROJECT_SOURCE_DIR}/include)