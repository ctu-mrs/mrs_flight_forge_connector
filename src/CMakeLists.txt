set(SOURCES connection.cpp robot_client.cpp simulator_client.cpp)

add_library(${LIBRARY_NAME} OBJECT ${SOURCES})
target_include_directories(${LIBRARY_NAME} PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(${LIBRARY_NAME} PUBLIC Threads::Threads)
if (WIN32)
    target_link_libraries(${LIBRARY_NAME} PUBLIC ws2_32)
endif()
